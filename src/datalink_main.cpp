#include <stdint.h>

#include "rodos.h"

#include "gateway/router.h"
#include "gateway/gateway.h"

#include "udp_ipc.hpp"

//#include "ORPE/include/Datastruct.h"

//Settings for the ORPE datalink.
#define DATALINK_ORPETELEMETRY_CHANNEL      5120 //The channel used to send telemetry data to the datalink.
#define DATALINK_ORPETELECOMMAND_CHANNEL    5121 //The channel used to recieve telecommands from the datalink.


/// @brief compact form of data from orpe. Used for transfer with other systems.
struct OrpeTelemetry {

    // Estimated pose. p values are the position in mm. a values are rotation in rotation axis (magnetude is angle direction is axis).
    float px, py, pz;
    float ax, ay, az;
    // Frame number.
    uint32_t frameNum;
    // Timestamp in milliseconds in reference to start of ORPE.
    uint32_t timestamp;
    // If the frame estimation is valid
    bool valid;
    // The leds that are currently identified. Each paar of 2 bits correspond to the ID and give the number of Points with this ID. E.g. bits 2 and 3 are 01 means there is 1 point with ID 1. Bits 4 and 5 are 11 then 3 or more points with ID 2.
    uint32_t ledIDs;
    // Total number of points found in the image.
    uint16_t numPoints;
};// __attribute__ ((packed));

/// @brief Command types that can be sent to ORPE
enum ORPECommandType_t : uint8_t {
    ORPECommandType_Start, //Orpe will reset and begin running to identify LEDs and estimate pose.
    ORPECommandType_Stop,  //Orpe will stop and go into a low cpu usage mode awaiting start command.
    ORPECommandType_TakeImage, //Orpe will save the latest image upon receival of command and send back. Takes a normal exposed high res image when orpe not running, otherwise the same image orpe uses for estimation.
    ORPECommandType_TakeImageData //Same as ORPECommandType_TakeImage but will also draw debug info onto the image if orpe is running.
};

/// @brief Container for ORPE commands. What the value means depends on the command.
struct ORPECommand {
    ORPECommandType_t command;
    int32_t value;
};


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
Topic<float> datalinkTimeTopic(1301, "Datalink Time Testing");


/**
 * Communicates with the ORPE process via the udpipc API.
*/
class ORPEDatalink : public StaticThread<> {
private:

    //IPC to receive ORPE estimations
    UdpIpc<OrpeTelemetry> orpeEstIPC_;
    UdpIpc<ORPECommandType_t> orpeCmdIPC_;

    //Buffer to receive the telecommands for ORPE
    RODOS::CommBuffer<ORPECommand> cmdBuf_;
    RODOS::Subscriber cmdSubr_;

public: 

    ORPEDatalink() : cmdSubr_(orpeCmdTopic, cmdSubr_) {}


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

    CommBuffer<ORPEDatalink> orpeTeleBuf_;
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

                std::vector<int> ledIDs;
                for (int i = 0; i < 15; i++) {
                    ledIDs.push_back((orpeTele.ledIDs >> (i * 2)) & 0b00000011);
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