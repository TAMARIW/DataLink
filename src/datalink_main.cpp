#include <stdint.h>

#include "rodos.h"

#include "gateway/router.h"
#include "gateway/gateway.h"

#include "udp_ipc.hpp"

#include "Datastruct.h"


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
Topic<ORPEDatalink> orpeEstTopic;


/**
 * Communicates with the ORPE proccess via the udpipc API.
*/
class ORPEDatalink : public StaticThread<> {
private:

    //IPC to receive ORPE estimations
    UdpIpc<ORPEDatalink> orpeEstIPC_;
    UdpIpc<ORPECommand_t> orpeCmdIPC_;


public: 

    ORPEDatalink();


    void init() override {

        uart_gateway.addTopicsToForward(orpeEstTopic);

    }

    void run() override {

        orpeEstIPC_.init(10);
        orpeCmdIPC_.init(11);

        while (1) {
            
            ORPEDatalink orpeData;
            if (orpeEstIPC_.receiveData(orpeData)) {
                orpeEstTopic.publish(orpeData);
            }

        }

    }


} orpeDatalink;