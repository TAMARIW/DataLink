#include <stdint.h>
#include <string>

#include "rodos.h"

//#include "gateway/router.h"
//#include "gateway/gateway.h"

#include "exclusive_router.hpp"

#include "udp_ipc.hpp"

#include "Datastruct.h"


//Debug settings
#define ORPE_NETWORK_WIDE false //Setting this to true will enable datalink to communicate with ORPE running on any device. This requires only one instance of ORPE to be running on the network.
#define DATALINK_DEBUG_MESSAGES

//Settings for the ORPE datalink.
#define DATALINK_ORPETELEMETRY_CHANNEL          5120 //The channel used to send telemetry data to the datalink.
#define DATALINK_ORPETELECOMMAND_CHANNEL        5121 //The channel used to recieve telecommands from the datalink.
#define DATALINK_ORPESTATE_CHANNEL              5122 //The channel used to recieve telecommands from the datalink.

#define DATALINK_ORPETELEMETRY_SELF_TOPICID     1300 //ORPE pose estimations from the same satellite. (From itsself)
#define DATALINK_ORPETELECOMMAND_SELF_TOPICID   1301 //ORPE telecommands to ORPE on same satellite. (To itsself)
#define DATALINK_ORPESTATE_SELF_TOPICID         1302 //ORPE state from the same satellite. (From itsself)

#define DATALINK_ORPETELEMETRY_TGT_TOPICID      1310 //ORPE pose estimations from the target satellite. (From other satellite)
#define DATALINK_ORPETELECOMMAND_TGT_TOPICID    1311 //ORPE telecommands to the other satellite. (To other satellite) 
#define DATALINK_ORPESTATE_TGT_TOPICID          1312 //ORPE pose estimations from the target satellite. (From other satellite)

#define DATALINK_ORPETELEMETRY_INTER_TOPICID    1400 //ORPE pose estimations from the target satellite. (From other satellite) Used for inter communication.
#define DATALINK_ORPETELECOMMAND_INTER_TOPICID  1401 //ORPE telecommands to the other satellite. (To other satellite) Used for inter communication.
#define DATALINK_ORPESTATE_INTER_TOPICID        1402 //ORPE pose estimations from the target satellite. (From other satellite) Used for inter communication.


// Gateway setup
UDPInOut udp(-50000);
LinkinterfaceUDP linkinterface(&udp);
Gateway wifi_gateway(&linkinterface);

HAL_UART uart(UART_IDX4);
LinkinterfaceUART uart_linkinterface(&uart, 115200);
Gateway stm32_gateway(&uart_linkinterface);

//Gateway router 
ExclusiveRouter gatewayRouter(true, &stm32_gateway, &wifi_gateway);

//Topics for communication with ORPE
Topic<OrpeTelemetry> orpeSelfTmtTopic(DATALINK_ORPETELEMETRY_SELF_TOPICID, "ORPE Self telemetry");
Topic<ORPECommand> orpeSelfCmdTopic(DATALINK_ORPETELECOMMAND_SELF_TOPICID, "ORPE Self telecommand");
Topic<ORPEState_t> orpeSelfSttTopic(DATALINK_ORPESTATE_SELF_TOPICID, "ORPE Self state");

Topic<OrpeTelemetry> orpeTgtTmtTopic(DATALINK_ORPETELEMETRY_TGT_TOPICID, "ORPE TGT telemetry");
Topic<ORPECommand> orpeTgtCmdTopic(DATALINK_ORPETELECOMMAND_TGT_TOPICID, "ORPE TGT telecommand");
Topic<ORPEState_t> orpeTgtSttTopic(DATALINK_ORPESTATE_TGT_TOPICID, "ORPE TGT state");

Topic<OrpeTelemetry> orpeIntTmtTopic(DATALINK_ORPETELEMETRY_INTER_TOPICID, "ORPE INTER telemetry");
Topic<ORPECommand> orpeIntCmdTopic(DATALINK_ORPETELECOMMAND_INTER_TOPICID, "ORPE INTER telecommand");
Topic<ORPEState_t> orpeIntSttTopic(DATALINK_ORPESTATE_INTER_TOPICID, "ORPE INTER state");


//Topic for testing performance
#define TEST_DATA_SIZE 100 - 4 - 8
struct DataPacketDummy {
    int64_t sendTime;
    uint32_t id;
    uint8_t data[TEST_DATA_SIZE];
};
Topic<DataPacketDummy> oncomingData(31000, "Oncoming data testing");
Topic<DataPacketDummy> outgoingData(31001, "Outgoing data testing");

void resenderFunc(DataPacketDummy &data) {
    outgoingData.publish(data);
}

SubscriberReceiver<DataPacketDummy> resendSubr(oncomingData, resenderFunc, "Datalink perf testing subr");


class DatalinkPerfSender : StaticThread<> {
private:

    int64_t interval_ = 10*MILLISECONDS;

public:

    void init() override {

        wifi_gateway.addTopicsToForward(&outgoingData);

    }


    void run() override {

        DataPacketDummy data;
        for (int i = 0; i < TEST_DATA_SIZE; i++) {
            data.data[i] = i+1;
        }
        data.id = 0;


        int64_t nextRun = NOW();

        while (1) {

            data.sendTime = NOW();

            outgoingData.publish(data);
            data.id++;

            nextRun += interval_;
            suspendCallerUntil(nextRun);

        }

    }

} perfSender;



class DatalinkPerfRecv : StaticThread<> {
private:

    CommBuffer<DataPacketDummy> datapacketBuf_;
    Subscriber datapacketSub_;

    

public:

    DatalinkPerfRecv() :
        datapacketSub_(oncomingData, datapacketBuf_)
    {}

    void init() override {

        //Perf testing
        wifi_gateway.addTopicsToForward(&oncomingData);
        wifi_gateway.addTopicsToForward(&outgoingData);

        gatewayRouter.addTopicToExclude(31000);
        gatewayRouter.addTopicToExclude(31001);

    }


    void run() override {

        DataPacketDummy data, dataRcv;
        for (int i = 0; i < TEST_DATA_SIZE; i++) {
            data.data[i] = i+1;
        }

        uint32_t packetCounter = 0;
        uint32_t packetMisses = 0;
        uint32_t packetCorruption = 0;

        double totalRRT = 0;

        int64_t testBegin = NOW();

        int32_t lastID = -1;

        while (1) {

            outgoingData.publish(data);
            packetCounter++;

            int64_t start = NOW();
            bool newData = false;
            while (!(newData = datapacketBuf_.getOnlyIfNewData(dataRcv)) && NOW() - start < 1*SECONDS) 
                yield();
            
            int64_t time = NOW();
            int64_t timeSinceStart = time - testBegin;

            bool incorrect = false;
            if (newData) {

                for (int i = 0; i < TEST_DATA_SIZE; i++) {

                    if (dataRcv.data[i] != data.data[i]) {
                        incorrect = true;
                        break;
                    }

                }

            } else {
                PRINTF("TIMEOUT! Looks like a packet was lost.\n");
                packetMisses++;
            }

            //Count the number of missed packets if any.
            if (newData && dataRcv.id != lastID+1) {
                packetMisses += dataRcv.id - lastID - 1;
            }

            if (incorrect) {
                packetCorruption++;
                PRINTF("INCORRECT! Looks like the data was corrupted.\n");
            }

            double rrt = (double)(time - dataRcv.sendTime)/MILLISECONDS;

            if(newData && !incorrect && dataRcv.id == lastID+1) 
                totalRRT += rrt;

            if (packetCounter%100 == 0) {   

                float avgLatency = totalRRT/(packetCounter-packetMisses-packetCorruption)/2;

                PRINTF("\nPackets: %d \nTime: %.3fs \nLatency: %fms \nLosses: %d \nCorrupt: %d\n", packetCounter, float(double(timeSinceStart)/SECONDS), avgLatency, packetMisses, packetCorruption);

            }

            lastID = dataRcv.id;

        }

    }

} datalinkManagment;


/**
 * This class takes care of starting ORPE.
 */
class ORPEStartup : public StaticThread<> {
private:

    CommBuffer<ORPECommand> datalinkORPEStartup_;
    Subscriber datalinkORPEStartupSub_;

public:

    ORPEStartup() :
        datalinkORPEStartupSub_(orpeSelfCmdTopic, datalinkORPEStartup_)
    {}


    void init() override {

    }

    void run() override {

        ORPECommand cmd;

        while (1) { 

            if (datalinkORPEStartup_.getOnlyIfNewData(cmd) && cmd.command == ORPECommandType_t::ORPECommandType_Startup) {

                datalinkORPEStartupSub_.enable(false); //Disable otherwise an old command to startup received after this point could cause ORPE to startup again.
                std::system("~/orpetmw/build/ORPE"); //Will block until ORPE shuts down.
                datalinkORPEStartupSub_.enable(true);

            }

            suspendCallerUntil(NOW() + 100*MILLISECONDS);

        }

    }


} orpeStartup;


/**
 * Communicates with the ORPE process via the udpipc API.
*/
class ORPEDatalink : public StaticThread<> {
private:

    //IPC to communicate with ORPE
    UdpIpc<OrpeTelemetry> orpeEstIPC_;
    UdpIpc<ORPECommand> orpeCmdIPC_;
    UdpIpc<ORPEState_t> orpeSttIPC_;

    //Subscriber buffers for receiving commands and telemetries from self and target.
    RODOS::CommBuffer<ORPECommand> cmdSelfBuf_; //Commands to this satellite ORPE from stm32
    RODOS::CommBuffer<ORPECommand> cmdTgtBuf_; //Commands to tgt satellite ORPE from stm32
    RODOS::CommBuffer<ORPECommand> cmdIntBuf_; //Commands to self satellite ORPE from tgt
    RODOS::CommBuffer<OrpeTelemetry> tmtIntBuf_; //Tmt to stm32 from tgt satellite ORPE
    RODOS::CommBuffer<ORPEState_t> sttIntBuf_; //Tmt to stm32 from tgt satellite ORPE
    
    //Subscribers for buffers
    RODOS::Subscriber cmdSelfSubr_;
    RODOS::Subscriber cmdTgtSubr_;
    RODOS::Subscriber cmdIntSubr_;
    RODOS::Subscriber tmtIntSubr_;
    RODOS::Subscriber sttIntSubr_;

public: 

    ORPEDatalink() : 
        cmdSelfSubr_(orpeSelfCmdTopic, cmdSelfBuf_),
        cmdTgtSubr_(orpeTgtCmdTopic, cmdTgtBuf_),
        cmdIntSubr_(orpeIntCmdTopic, cmdIntBuf_),
        tmtIntSubr_(orpeIntTmtTopic, tmtIntBuf_),
        sttIntSubr_(orpeIntSttTopic, sttIntBuf_)
    {}


    void init() override {

        //These topics should not be routed between the gateways!
        gatewayRouter.addTopicToExclude(DATALINK_ORPETELEMETRY_SELF_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPETELECOMMAND_SELF_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPESTATE_SELF_TOPICID);

        gatewayRouter.addTopicToExclude(DATALINK_ORPETELEMETRY_TGT_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPETELECOMMAND_TGT_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPESTATE_TGT_TOPICID);

        gatewayRouter.addTopicToExclude(DATALINK_ORPETELEMETRY_INTER_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPETELECOMMAND_INTER_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPESTATE_INTER_TOPICID);

        //Comms with STM32
        stm32_gateway.addTopicsToForward(&orpeSelfTmtTopic);
        stm32_gateway.addTopicsToForward(&orpeSelfCmdTopic);
        stm32_gateway.addTopicsToForward(&orpeSelfSttTopic);

        stm32_gateway.addTopicsToForward(&orpeTgtTmtTopic);
        stm32_gateway.addTopicsToForward(&orpeTgtCmdTopic);
        stm32_gateway.addTopicsToForward(&orpeTgtSttTopic);

        //Intercomms
        wifi_gateway.addTopicsToForward(&orpeIntTmtTopic);
        wifi_gateway.addTopicsToForward(&orpeIntCmdTopic);
        wifi_gateway.addTopicsToForward(&orpeIntSttTopic);

    }

    void run() override {

        orpeEstIPC_.init(DATALINK_ORPETELEMETRY_CHANNEL, ORPE_NETWORK_WIDE);
        orpeCmdIPC_.init(DATALINK_ORPETELECOMMAND_CHANNEL, ORPE_NETWORK_WIDE);
        orpeSttIPC_.init(DATALINK_ORPESTATE_CHANNEL, ORPE_NETWORK_WIDE);

        while (1) {
            
            ORPECommand orpeCmd;
            OrpeTelemetry orpeData;
            ORPEState_t orpeState;
            
            // Forward ORPE telemetry to STM32 and Intercomms
            if (orpeEstIPC_.receiveData(orpeData)) {

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE tele to STM32 and Intercom\n");
                #endif

                orpeSelfTmtTopic.publish(orpeData);

                tmtIntSubr_.enable(false);
                orpeIntTmtTopic.publish(orpeData);
                tmtIntSubr_.enable(true);

            }

            // Forward ORPE state to STM32 and Intercomms
            if (orpeSttIPC_.receiveData(orpeState)) {

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE state to STM32 and Intercom\n");
                #endif

                orpeSelfSttTopic.publish(orpeState);

                sttIntSubr_.enable(false);
                orpeIntSttTopic.publish(orpeState);
                sttIntSubr_.enable(true);

            }

            // Forward ORPE telemetry from intercomms to stm32
            if (tmtIntBuf_.getOnlyIfNewData(orpeData)) {

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE tele from intercom to stm32\n");
                #endif

                orpeTgtTmtTopic.publish(orpeData);
            }

            // Forward Commands from intercomms to self commands (Needed for ORPE startup class to also capture messages)
            if (cmdIntBuf_.getOnlyIfNewData(orpeCmd)) { //Self (from STM32) commands will be used if receiving commands from self and target at the same time.

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE commands from intercomms to ORPE\n");
                #endif

                orpeSelfCmdTopic.publish(orpeCmd);
            }

            // Forward Commands from stm32 to ORPE
            if (cmdSelfBuf_.getOnlyIfNewData(orpeCmd)) { //Self (from STM32) commands will be used if receiving commands from self and target at the same time.

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE commands from stm32 to ORPE\n");
                #endif

                orpeCmdIPC_.sendData(orpeCmd);
            }

            // Forward Commands from stm32 for target to intercomms
            if (cmdTgtBuf_.getOnlyIfNewData(orpeCmd)) {

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE commands from stm32 to intercomms\n");
                #endif

                cmdIntSubr_.enable(false); 
                orpeIntCmdTopic.publish(orpeCmd);
                cmdIntSubr_.enable(true);
                
            }

            // Forward State telemetry from ORPE to STM32 and intercomms
            /*if (sttIntBuf_.getOnlyIfNewData(orpeState)) {  

                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE state to stm32 and intercomms\n");
                #endif

                orpeSelfSttTopic.publish(orpeState);

                sttIntSubr_.enable(false);
                orpeIntSttTopic.publish(orpeState);
                sttIntSubr_.enable(true);

            }*/

            // Forward ORPE state from intercomms to stm32
            if (sttIntBuf_.getOnlyIfNewData(orpeState)) {
                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE state from intercomms to stm32\n");
                #endif
                orpeTgtSttTopic.publish(orpeState);
            }
            
            //Force udp gateway to update to make sure they capture new messages.
            //wifi_gateway.resume();

            // Delay should be a low enough max latency while also consuming a low amount of cpu for busy waiting.
            suspendCallerUntil(NOW() + 10*MILLISECONDS);

        }

    }


} orpeDatalink;



/**
 * A class for testing the datalink. Recieves and prints ORPE data
*/
class DatalinkTestingRecieve : public StaticThread<> {
private:

    CommBuffer<OrpeTelemetry> orpeTeleBuf_;
    Subscriber orpeTeleSubr_;


public: 

    DatalinkTestingRecieve() : 
        orpeTeleSubr_(orpeIntTmtTopic, orpeTeleBuf_) 
    {}


    void init() override {

    }

    void run() override {



        while (1) {

            OrpeTelemetry orpeTele;
            if (orpeTeleBuf_.getOnlyIfNewData(orpeTele)) {

                std::string ledIDString = "";
                for (int i = 0; i < 16; i++) {
                    
                    ledIDString += std::to_string(i) + ": " + std::to_string(orpeTele.ledIDCount[i]) + "\n\t";
                    
                }

                PRINTF("Telemetry \npos: %f, %f, %f\nrot: %f, %f, %f\nPoints: %d\nIDs: \t %s", orpeTele.px, orpeTele.py, orpeTele.pz, orpeTele.ax, orpeTele.ay, orpeTele.az, orpeTele.numPoints, ledIDString.c_str());
            }

            PRINTF("TEST\n");
            
            suspendCallerUntil(NOW() + 1*MILLISECONDS);

        }

    }


};
//DatalinkTestingRecieve datalinkTestingRecieve;
