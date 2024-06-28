#include <stdint.h>

#include "rodos.h"

#include "gateway/router.h"
#include "gateway/gateway.h"

#include "udp_ipc.hpp"

#include "Datastruct.h"

//#include "ORPE/include/Datastruct.h"

//Settings for the ORPE datalink.
#define DATALINK_ORPETELEMETRY_CHANNEL      5120 //The channel used to send telemetry data to the datalink.
#define DATALINK_ORPETELECOMMAND_CHANNEL    5121 //The channel used to recieve telecommands from the datalink.


// Gateway setup
UDPInOut udp(-50000);
LinkinterfaceUDP linkinterface(&udp);
Gateway udp_gateway(&linkinterface, true);

/*HAL_UART uart(UART_IDX4);
LinkinterfaceUART uart_linkinterface(&uart, 115200);
Gateway uart_gateway(&uart_linkinterface, true);

//Gateway router 
Router gatewayRouter(true, &uart_gateway, &udp_gateway);*/


//Topics for communication with ORPE
Topic<OrpeTelemetry> orpeEstTopic(1300, "ORPE telemetry");
Topic<ORPECommand> orpeCmdTopic(1301, "ORPE telecommand");

//Topics for testing the datalink
Topic<float> datalinkTimeTopic(1501, "Datalink Time Testing");


/**
 * Communicates with the ORPE process via the udpipc API.
*/
class ORPEDatalink : public StaticThread<> {
private:

    //IPC to receive ORPE estimations
    UdpIpc<OrpeTelemetry> orpeEstIPC_;
    UdpIpc<ORPECommand> orpeCmdIPC_;

    //Buffer to receive the telecommands for ORPE
    RODOS::CommBuffer<ORPECommand> cmdBuf_;
    RODOS::Subscriber cmdSubr_;

public: 

    ORPEDatalink() : cmdSubr_(orpeCmdTopic, cmdBuf_) {}


    void init() override {

        udp_gateway.addTopicsToForward(&orpeEstTopic);

    }

    void run() override {

        orpeEstIPC_.init(DATALINK_ORPETELEMETRY_CHANNEL);
        orpeCmdIPC_.init(DATALINK_ORPETELECOMMAND_CHANNEL);

        while (1) {
            
            // This forwards the udpipc telemetry from ORPE to RODOS system
            OrpeTelemetry orpeData;
            if (orpeEstIPC_.receiveData(orpeData)) {
                orpeEstTopic.publish(orpeData);

            }

            // This forwards the telecommands from the RODOS system to ORPE via udpipc
            ORPECommand orpeCmd;
            if (cmdBuf_.getOnlyIfNewData(orpeCmd)) 
                orpeCmdIPC_.sendData(orpeCmd);

            // Delay should be a low enough max latency while also consuming a low amount of cpu for busy waiting.
            suspendCallerUntil(NOW() + 10*MILLISECONDS);

        }

    }


} orpeDatalink;


/**
 * A class for testing the datalink. Sends the time.
*/
class DatalinkTestingSend : public StaticThread<> {
private:


public: 

    DatalinkTestingSend() {}


    void init() override {

        udp_gateway.addTopicsToForward(&datalinkTimeTopic);

    }

    void run() override {



        while (1) {
            
            float time = SECONDS_NOW();
            datalinkTimeTopic.publish(time);
            
            suspendCallerUntil(NOW() + 1*SECONDS);

        }

    }


};
//DatalinkTestingSend datalinkTestingSend;


/**
 * A class for testing the datalink. Recieves the time.
*/
class DatalinkTestingRecieve : public StaticThread<> {
private:

    CommBuffer<float> timeBuf_;
    Subscriber timeSubr_;

    CommBuffer<OrpeTelemetry> orpeTeleBuf_;
    Subscriber orpeTeleSubr_;


public: 

    DatalinkTestingRecieve() : 
        timeSubr_(datalinkTimeTopic, timeBuf_),
        orpeTeleSubr_(orpeEstTopic, orpeTeleBuf_) 
    {}


    void init() override {

        udp_gateway.addTopicsToForward(&datalinkTimeTopic);

    }

    void run() override {



        while (1) {
            
            float time = 0;
            if (timeBuf_.getOnlyIfNewData(time)) {
                PRINTF("TESTING received time: %.2f\n", time);
            }

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

                printf("Telemetry \npos: %f, %f, %f\nrot: %f, %f, %f\nPoints: %d\nIDs: \t %s", orpeTele.px, orpeTele.py, orpeTele.pz, orpeTele.ax, orpeTele.ay, orpeTele.az, orpeTele.numPoints, ledIDString.c_str());
            }
            
            suspendCallerUntil(NOW() + 1*MILLISECONDS);

        }

    }


};
//DatalinkTestingRecieve datalinkTestingRecieve;