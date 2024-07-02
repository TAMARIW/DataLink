/**
 * @file gatewayGeneric.cc
 * @date 2012/05/01 7:07
 * @author Sergio Montenegro, Uni Würzburg
 *         update 2022/08/03: SM : Support for router networks
 *
 * @brief gateway for middleware
 *
 */
#include <stdint.h>


#include "application.h"
#include "gateway/gateway.h"
#include "reserved_application_ids.h"
#include "subscriber.h"
#include "thread.h"

#include "gateway_fast.hpp"


namespace RODOS {

extern int32_t myNodeNr; // normally use getNodeNumber(), but gateway is intrinsic function
extern int64_t numberOfReceivedMsgsFromNetwork;

/**************** Transmitter part of the gateway   ******************/

GatewayFast::GatewayFast(Linkinterface* linkinterface_, bool forwardall_, bool enable_) :
    Subscriber(defaultGatewayTopic, nopPutter, "Gateway", true),
    StaticThread<>("gateway", NETWORKREADER_PRIORITY) {
    this->linkinterface = linkinterface_;
    this->forwardAll = forwardall_;
    this->linkIdentifier = linkinterface_->getLinkdentifier();
    enable(enable_);

    getTopicsToForwardFromOutside=true;
    externalsubscribers.init();
}

int32_t GatewayFast::numberOfNodes = 0;
SeenNode GatewayFast::seenNodes[MAX_NUMBER_OF_NODES];
Semaphore GatewayFast::seenNodesProtector;

bool GatewayFast::messageSeen(NetworkMessage& msg) {
    //SeenNode key;
    SeenNode* node;



    seenNodesProtector.enter();
    /*===========qsort and binsearch version, disabled for now ========================
    key.nodeID=msg.senderNode;

    node= (SeenNode*)bsearch(&key,seenNodes,numberOfNodes,sizeof(SeenNode),compare_SeenNode);

    if(!node) {
        RODOS_ASSERT_IFNOT_RETURN(numberOfNodes < MAX_NUMBER_OF_NODES, false);
        key.lastMsgTime=msg.sentTime-1;
        seenNodes[numberOfNodes]=key;
        numberOfNodes++;

        qsort(seenNodes,numberOfNodes,sizeof(SeenNode),compare_SeenNode);
        node= (SeenNode*)bsearch(&key,seenNodes,numberOfNodes,sizeof(SeenNode),compare_SeenNode);
    }

    */

    //=============== simple search =======================================
       node=0;
       for(int i=0;i< numberOfNodes;i++){

           if(seenNodes[i].nodeID == msg.get_senderNode()){
               node = &seenNodes[i];
               break;
           }
       }

       if(!node){
           RODOS_ASSERT_IFNOT_RETURN((numberOfNodes < MAX_NUMBER_OF_NODES), false);
           
           node=&seenNodes[numberOfNodes];
           numberOfNodes++;

           node->nodeID = msg.get_senderNode();
           node->lastMsgTime = msg.get_sentTime()-1;
       }

       // ==================================================


    int64_t msgSentTime = msg.get_sentTime();
    if(node->lastMsgTime >= msgSentTime) {
        if(node->lastMsgTime-msgSentTime > 5*SECONDS) {
            node->lastMsgTime= msgSentTime;
            seenNodesProtector.leave();
            return false;
        }
        seenNodesProtector.leave();
        return true;
    } else {
        node->lastMsgTime= msgSentTime;
        seenNodesProtector.leave();
        return false;
    }

}

/** Forward the message to the interface **/

uint32_t GatewayFast::put(const uint32_t topicId, const size_t len, void* data, const NetMsgInfo& netMsgInfo) {
    if(!isEnabled) return 0;
    // if(topicId == 0) return 0;

    // Deprecated but still in use
    if(!forwardAll) {
        if(topicId !=0 && !externalsubscribers.find(topicId)) { return 0; }
    }

    networkOutProtector.enter();
    {
        prepareNetworkMessage(networkOutMessage,topicId,data,len, netMsgInfo);
        sendNetworkMessage(networkOutMessage);
    }
    networkOutProtector.leave();
    return 1;
}



void GatewayFast::sendNetworkMessage(NetworkMessage& msg) {
    //When implemented, buffering of the outgoing Messages should be done here.

    /*if(!forwardAll){
        if(msg.topicId !=0 && !externalsubscribers.find(msg.topicId)){
            return;
        }
    }*/

    networkOutProtector.enter(); // Also lock here if this function gets called from outside
    linkinterface->sendNetworkMsg(msg);
    networkOutProtector.leave();

}


/**************** Receiver part of the gateway   ********************/


void GatewayFast::setTopicsToForward(TopicListReport* topicList) {
    getTopicsToForwardFromOutside=false;
    externalsubscribers = *(TopicListReport*)topicList;
}

void GatewayFast::addTopicsToForward(TopicListReport* topicsWanted_) {
    getTopicsToForwardFromOutside=false;
    TopicListReport *topicsWanted = (TopicListReport*)topicsWanted_;
    for(uint32_t i = 0; i < topicsWanted->numberOfTopics; i++) {
        externalsubscribers.add(topicsWanted->topicList[i]);
    }
}

void GatewayFast::addTopicsToForward(TopicInterface* topicId1, TopicInterface* topicId2, TopicInterface* topicId3, TopicInterface* topicId4) {
    getTopicsToForwardFromOutside=false;

    externalsubscribers.add(topicId1->topicId);
    if(topicId2) { externalsubscribers.add(topicId2->topicId); }
    if(topicId3) { externalsubscribers.add(topicId3->topicId); }
    if(topicId4) { externalsubscribers.add(topicId4->topicId); }
}

void GatewayFast::resetTopicsToForward() {
    getTopicsToForwardFromOutside=false;
    externalsubscribers.init();
}



/*************************************************************************/

void GatewayFast::AnalyseAndDistributeMessagesFromNetwork() {

    if(networkInMessage.get_senderNode() == myNodeNr) {
        return;
    } // ***** discard messages from self

    if(!networkInMessage.isCheckSumOk()) {
        // PRINTF("Message dropped: Wrong Checksum %x != %x\n", networkInMessage.get_checksum(), networkInMessage.calculateCheckSum());
        return;
    }
    if(messageSeen(networkInMessage)) return;

    networkInMessage.dec_maxStepsToForward();
    { PRIORITY_CEILER_IN_SCOPE();  numberOfReceivedMsgsFromNetwork++; }

    uint32_t topicId           = networkInMessage.get_topicId();

    if(topicId == 0 && getTopicsToForwardFromOutside) { // This is a topic report of expected topics in network.
        if(linkinterface->isBroadcastLink) {
            addTopicsToForward((TopicListReport*)networkInMessage.userDataC); // for broadcast networks
        } else {
            setTopicsToForward((TopicListReport*)networkInMessage.userDataC); // for intelligent networks-Switches
        }
        getTopicsToForwardFromOutside=true;

    } else if(topicId > 0) {
        /** now distribute locally (if not from self and not topicreports) **/


        NetMsgInfo msgInfo;
        msgInfo.linkId         = linkIdentifier;
        msgInfo.sentTime       = networkInMessage.get_sentTime();
        msgInfo.senderNode     = networkInMessage.get_senderNode();
        msgInfo.senderThreadId = networkInMessage.get_senderThreadId();
        msgInfo.receiverNode   = networkInMessage.get_receiverNode();
        msgInfo.receiverNodesBitMap = networkInMessage.get_receiverNodesBitMap();
        msgInfo.messageType    = (NetMsgType)networkInMessage.get_type();

        ITERATE_LIST(TopicInterface, TopicInterface::topicList) {
            if(iter->topicId == topicId) {
                iter->publish(networkInMessage.userDataC, false, &msgInfo);
            }
        } // search all local topics

        //Publish for Routers to forward
        ((TopicInterface*)&defaultRouterTopic)->publish(&networkInMessage,false,&msgInfo);
    }
}

/*************************************************************************/
/**
 * one thread to distribute internally the incoming messages
 */



void GatewayFast::run() {
    bool     didSomething;
    int32_t  realMsgSize;


    linkinterface->init();

    while(1) {
        linkinterface->suspendUntilDataReady(NOW()+ 10*MILLISECONDS);

        didSomething=true;
        while(didSomething) {
            didSomething=false;

            if(linkinterface->getNetworkMsg(networkInMessage, realMsgSize)) {

                if(realMsgSize < 0) {// The physical layer does not provide a "real" msg size by its own, but relies on the message header
                    AnalyseAndDistributeMessagesFromNetwork();
                } else {
                    if(realMsgSize >= ((int32_t)sizeof(NetworkMessage)-MAX_NETWORK_MESSAGE_LENGTH)) {
                        if(realMsgSize == (int32_t)networkInMessage.numberOfBytesToSend()) {
                            AnalyseAndDistributeMessagesFromNetwork();
                        } else {
                            PRINTF("Message dropped because expected size(%d) != real size(%d)\n",(int)networkInMessage.numberOfBytesToSend(),(int)realMsgSize);
                        }
                    } else {
                        PRINTF("Message dropped because received is even smaller than the header\n");
                    }
                }

                didSomething=true;
            }

        }
    }
} // run

}

