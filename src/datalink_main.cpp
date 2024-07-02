#include <stdint.h>

#include "rodos.h"

//#include "gateway/router.h"
//#include "gateway/gateway.h"

#include "exclusive_router.hpp"
#include "gateway_fast.hpp"

#include "udp_ipc.hpp"

#include "Datastruct.h"


//Debug settings
#define ORPE_NETWORK_WIDE false //Setting this to true will enable datalink to communicate with ORPE running on any device. This requires only one instance of ORPE to be running on the network.


//Settings for the ORPE datalink.
#define DATALINK_ORPETELEMETRY_CHANNEL          5120 //The channel used to send telemetry data to the datalink.
#define DATALINK_ORPETELECOMMAND_CHANNEL        5121 //The channel used to recieve telecommands from the datalink.

#define DATALINK_ORPETELEMETRY_SELF_TOPICID     1300 //The channel where ORPE pose estimations from the same satellite is published. (From itsself)
#define DATALINK_ORPETELECOMMAND_SELF_TOPICID   1301 //The channel where ORPE telecommands are published to arrive at its own ORPE. (To itsself)


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


/**
 * Communicates with the ORPE process via the udpipc API.
*/
class ORPEDatalink : public StaticThread<> {
private:

    //IPC to receive ORPE estimations
    UdpIpc<OrpeTelemetry> orpeEstIPC_;
    UdpIpc<ORPECommand> orpeCmdIPC_;

    //Subscriber buffers for receiving commands and telemetries from self and target.
    RODOS::CommBuffer<ORPECommand> cmdSelfBuf_; //Commands to this satellite ORPE
    
    //Subscribers for buffers
    RODOS::Subscriber cmdSelfSubr_;

public: 

    ORPEDatalink() : 
        cmdSelfSubr_(orpeSelfCmdTopic, cmdSelfBuf_)
    {}


    void init() override {

        //These topics should not be routed between the gateways!
        gatewayRouter.addTopicToExclude(DATALINK_ORPETELEMETRY_SELF_TOPICID);
        gatewayRouter.addTopicToExclude(DATALINK_ORPETELECOMMAND_SELF_TOPICID);

        //Comms with STM32
        uart_gateway.addTopicsToForward(&orpeSelfTmtTopic);
        uart_gateway.addTopicsToForward(&orpeSelfCmdTopic);

    }

    void run() override {

        orpeEstIPC_.init(DATALINK_ORPETELEMETRY_CHANNEL, ORPE_NETWORK_WIDE);
        orpeCmdIPC_.init(DATALINK_ORPETELECOMMAND_CHANNEL, ORPE_NETWORK_WIDE);

        while (1) {
            
            ORPECommand orpeCmd;
            OrpeTelemetry orpeData;
            
            // This forwards the udpipc telemetry from ORPE to RODOS system
            if (orpeEstIPC_.receiveData(orpeData)) {
                orpeSelfTmtTopic.publish(orpeData);
            }

            // This forwards the telecommands from the RODOS system to ORPE via udpipc
            if (cmdSelfBuf_.getOnlyIfNewData(orpeCmd)) 
                orpeCmdIPC_.sendData(orpeCmd);

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
        orpeTeleSubr_(orpeSelfTmtTopic, orpeTeleBuf_) 
    {}


    void init() override {

    }

    void run() override {



        while (1) {

            OrpeTelemetry orpeTele;
            if (orpeTeleBuf_.getOnlyIfNewData(orpeTele)) {

                int ledIDs[16];
                for (int i = 0; i < 15; i++) {
                    ledIDs[i] = (orpeTele.ledIDs >> (i * 2)) & 0b00000011;
                }

                std::string ledIDString = "";
                for (int i = 0; i < 15; i++) {
                    if (ledIDs[i] != 0) {
                        ledIDString += std::to_string(i) + ": " + std::to_string(ledIDs[i]) + "\n\t";
                    }
                }

                PRINTF("Telemetry \npos: %f, %f, %f\nrot: %f, %f, %f\nPoints: %d\nIDs: \t %s", orpeTele.px, orpeTele.py, orpeTele.pz, orpeTele.ax, orpeTele.ay, orpeTele.az, orpeTele.numPoints, ledIDString.c_str());
            }
            
            suspendCallerUntil(NOW() + 1*MILLISECONDS);

        }

    }


};
//DatalinkTestingRecieve datalinkTestingRecieve;