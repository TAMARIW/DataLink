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
#define DATALINK_ENABLE_WIFI_CONNECT            2101 //Topic used to connect or disconnect wifi connection    

#define DATALINK_HEARTBEAT                      2202 //Topic used to send the datalink heartbeat
#define DATALINK_HEARTBEAT_INTER                2203 //Topic used to send the datalink heartbeat over wifi


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

//Topic used for controlling the wifi
Topic<bool> datalinkEnableWiFiConnect(DATALINK_ENABLE_WIFI_CONNECT, "Datalink enable wifi connect");

//Topic used for the datalink heartbeat
Topic<int64_t> datalinkHeartbeat(DATALINK_HEARTBEAT, "Datalink heartbeat");
Topic<int64_t> datalinkHeartbeatInter(DATALINK_HEARTBEAT_INTER, "Datalink heartbeat intercomms");

void datalinkWiFiConnectFunc(bool& enable) {

    if (enable) {
        PRINTF("Enabling wifi connect\n");
        //std::system("sudo iwconfig wlan0 mode ad-hoc essid TMWNetwork key off");
        //std::system("sudo ifconfig wlan0 192.168.10.1 netmask 255.255.255.0 up");
        std::system("sudo ifconfig wlan0 up");
    } else {
        PRINTF("Disabling wifi connect\n");
        std::system("sudo ifconfig wlan0 down");
    }

}
//SubscriberReceiver<bool> datalinkWiFiConnectSubscriber(datalinkEnableWiFiConnect, datalinkWiFiConnectFunc);


/**
 * This class takes care of controlling the wifi.
 */
class WiFiControl : public StaticThread<> {
private: 

    CommBuffer<bool> datalinkWiFiConnectBuf_;
    Subscriber datalinkWiFiConnectSub_;


public:

    WiFiControl() :
        datalinkWiFiConnectSub_(datalinkEnableWiFiConnect, datalinkWiFiConnectBuf_)
    {}

    void init() override {

        //WiFi control
        stm32_gateway.addTopicsToForward(&datalinkEnableWiFiConnect);

        gatewayRouter.addTopicToExclude(DATALINK_ENABLE_WIFI_CONNECT);

    }

    void run() override {

        bool enable;

        datalinkWiFiConnectBuf_.getOnlyIfNewData(enable);

        while (1) {

            if (datalinkWiFiConnectBuf_.getOnlyIfNewData(enable)) {
                datalinkWiFiConnectFunc(enable);
                //suspendCallerUntil(NOW() + 3000*MILLISECONDS);
            }

            suspendCallerUntil(NOW() + 100*MILLISECONDS);

        }

    }

} wifiControl;

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

    RODOS::CommBuffer<int64_t> heartbeatBuf_; //Heartbeat buffer
    RODOS::CommBuffer<int64_t> heartbeatInterBuf_; //Heartbeat buffer for intercomms    
    
    //Subscribers for buffers
    RODOS::Subscriber cmdSelfSubr_;
    RODOS::Subscriber cmdTgtSubr_;
    RODOS::Subscriber cmdIntSubr_;
    RODOS::Subscriber tmtIntSubr_;
    RODOS::Subscriber sttIntSubr_;
    RODOS::Subscriber heartbeatSubr_;
    RODOS::Subscriber heartbeatInterSubr_;

public: 

    ORPEDatalink() : 
        cmdSelfSubr_(orpeSelfCmdTopic, cmdSelfBuf_),
        cmdTgtSubr_(orpeTgtCmdTopic, cmdTgtBuf_),
        cmdIntSubr_(orpeIntCmdTopic, cmdIntBuf_),
        tmtIntSubr_(orpeIntTmtTopic, tmtIntBuf_),
        sttIntSubr_(orpeIntSttTopic, sttIntBuf_),
        heartbeatSubr_(datalinkHeartbeat, heartbeatBuf_),
        heartbeatInterSubr_(datalinkHeartbeatInter, heartbeatInterBuf_)
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

        gatewayRouter.addTopicToExclude(DATALINK_HEARTBEAT);
        gatewayRouter.addTopicToExclude(DATALINK_HEARTBEAT_INTER);

        //Comms with STM32
        stm32_gateway.addTopicsToForward(&orpeSelfTmtTopic);
        stm32_gateway.addTopicsToForward(&orpeSelfCmdTopic);
        stm32_gateway.addTopicsToForward(&orpeSelfSttTopic);

        stm32_gateway.addTopicsToForward(&orpeTgtTmtTopic);
        stm32_gateway.addTopicsToForward(&orpeTgtCmdTopic);
        stm32_gateway.addTopicsToForward(&orpeTgtSttTopic);

        stm32_gateway.addTopicsToForward(&datalinkHeartbeat);

        //Intercomms
        wifi_gateway.addTopicsToForward(&orpeIntTmtTopic);
        wifi_gateway.addTopicsToForward(&orpeIntCmdTopic);
        wifi_gateway.addTopicsToForward(&orpeIntSttTopic);

        wifi_gateway.addTopicsToForward(&datalinkHeartbeatInter);

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

            // Forward ORPE state from intercomms to stm32
            if (sttIntBuf_.getOnlyIfNewData(orpeState)) {
                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding ORPE state from intercomms to stm32\n");
                #endif
                orpeTgtSttTopic.publish(orpeState);
            }

            // Forward Heartbeat to intercomms
            int64_t heartbeat;
            if (heartbeatBuf_.getOnlyIfNewData(heartbeat)) {
                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding heartbeat to intercomms\n");
                #endif
                heartbeatInterSubr_.enable(false);
                datalinkHeartbeatInter.publish(heartbeat);
                heartbeatInterSubr_.enable(true);
            }

            // Forward Heartbeat from intercomms to stm32
            if (heartbeatInterBuf_.getOnlyIfNewData(heartbeat)) {
                #ifdef DATALINK_DEBUG_MESSAGES
                PRINTF("Forwarding heartbeat from intercomms to stm32\n");
                #endif
                heartbeatSubr_.enable(false);
                datalinkHeartbeat.publish(heartbeat);
                heartbeatSubr_.enable(true);
            }

            
            //Force udp gateway to update to make sure they capture new messages.
            //wifi_gateway.resume();

            // Delay should be a low enough max latency while also consuming a low amount of cpu for busy waiting.
            suspendCallerUntil(NOW() + 10*MILLISECONDS);

        }

    }


} orpeDatalink;
