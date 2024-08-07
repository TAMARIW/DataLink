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

//Settings for the datalink wifi
#define DATALINK_ENABLE_WIFI_AP                 2100 //Topic used to enable or disable the wifi access point
#define DATALINK_ENABLE_WIFI_CONNECT            2101 //Topic used to connect or disconnect wifi connection    


// Gateway setup
UDPInOut udp(-50000);
LinkinterfaceUDP linkinterface(&udp);
Gateway udp_gateway(&linkinterface);

HAL_UART uart(UART_IDX4);
LinkinterfaceUART uart_linkinterface(&uart, 115200);
Gateway uart_gateway(&uart_linkinterface);

//Gateway router 
ExclusiveRouter gatewayRouter(true, &uart_gateway, &udp_gateway);

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

//Topics used for controlling the wifi
Topic<bool> datalinkEnableWiFiAP(DATALINK_ENABLE_WIFI_AP, "Datalink enable wifi AP");
Topic<bool> datalinkEnableWiFiConnect(DATALINK_ENABLE_WIFI_CONNECT, "Datalink enable wifi connect");


/**
 * The following functions take care of the wifi control
 */
void datalinkWiFiAPFunc(bool& enable) {

    if (enable) {
        std::system("sudo nmcli connection add con-name 'Hotspot' \\ifname wlan0 type wifi slave-type bridge master bridge0 \\wifi.mode ap wifi.ssid TMWNetwork wifi-sec.key-mgmt wpa-psk \\wifi-sec.proto rsn wifi-sec.pairwise ccmp \\wifi-sec.psk TMWNetwork");
    } else {
        std::system("sudo nmcli con down Hotspot");
    }

}
SubscriberReceiver<bool> datalinkWiFiAPSubscriber(datalinkEnableWiFiAP, datalinkWiFiAPFunc);

void datalinkWiFiConnectFunc(bool& enable) {

    if (enable) {
        std::system("sudo nmcli dev wifi connect TMWNetwork password TMWNetwork");
    } else {
        std::system("sudo nmcli con down TMWNetwork");
    }

}
SubscriberReceiver<bool> datalinkWiFiConnectSubscriber(datalinkEnableWiFiConnect, datalinkWiFiConnectFunc);

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

        gatewayRouter.addTopicToExclude(DATALINK_ENABLE_WIFI_AP);
        gatewayRouter.addTopicToExclude(DATALINK_ENABLE_WIFI_CONNECT);

        //Comms with STM32
        uart_gateway.addTopicsToForward(&orpeSelfTmtTopic);
        uart_gateway.addTopicsToForward(&orpeSelfCmdTopic);
        uart_gateway.addTopicsToForward(&orpeSelfSttTopic);

        uart_gateway.addTopicsToForward(&orpeTgtTmtTopic);
        uart_gateway.addTopicsToForward(&orpeTgtCmdTopic);
        uart_gateway.addTopicsToForward(&orpeTgtSttTopic);

        //Intercomms
        udp_gateway.addTopicsToForward(&orpeIntTmtTopic);
        udp_gateway.addTopicsToForward(&orpeIntCmdTopic);
        udp_gateway.addTopicsToForward(&orpeIntSttTopic);

        //WiFi control
        uart_gateway.addTopicsToForward(&datalinkEnableWiFiAP);
        uart_gateway.addTopicsToForward(&datalinkEnableWiFiConnect);

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
            //udp_gateway.resume();

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

        
        datalinkEnableWiFiAP.publish(true);

        suspendCallerUntil(NOW() + 20*SECONDS);

        datalinkEnableWiFiAP.publish(false);
        datalinkEnableWiFiConnect.publish(true);

        suspendCallerUntil(END_OF_TIME);
        

    }


};
DatalinkTestingRecieve datalinkTestingRecieve;
