#include <stdint.h>

#include "rodos.h"

#include "gateway/router.h"
#include "gateway/gateway.h"

#include "udp_ipc.hpp"

//#include "ORPE/include/Datastruct.h"


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
HAL_UART uart(UART_IDX4);
LinkinterfaceUART uart_linkinterface(&uart, 115200);
Gateway uart_gateway(&uart_linkinterface, true);

UDPInOut udp(-50000);
LinkinterfaceUDP linkinterface(&udp);
Gateway udp_gateway(&linkinterface, true);

//Gateway router 
Router gatewayRouter(true, &uart_gateway, &udp_gateway);


//Topics for communication with ORPE
Topic<OrpeTelemetry> orpeEstTopic(1342, "ORPE telemetry");


/**
 * Communicates with the ORPE proccess via the udpipc API.
*/
class ORPEDatalink : public StaticThread<> {
private:

    //IPC to receive ORPE estimations
    UdpIpc<OrpeTelemetry> orpeEstIPC_;
    UdpIpc<ORPECommandType_t> orpeCmdIPC_;


public: 

    ORPEDatalink() {}


    void init() override {

        uart_gateway.addTopicsToForward(&orpeEstTopic);

    }

    void run() override {

        orpeEstIPC_.init(10);
        orpeCmdIPC_.init(11);

        while (1) {
            
            OrpeTelemetry orpeData;
            if (orpeEstIPC_.receiveData(orpeData)) {
                orpeEstTopic.publish(orpeData);
            }

        }

    }


} orpeDatalink;