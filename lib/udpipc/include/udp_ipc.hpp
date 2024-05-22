#ifndef UDP_IPC_HPP
#define UDP_IPC_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h> 
#include <sys/socket.h> 
#include <unistd.h> 


/**
 * @file udp_ipc.hpp
 * @brief This file contains the UPD IPC. This is used by seperate processes to communicate with each other in a single direction.
 * @note Can only be used on the same machine and only supports linux systems.
*/


/**
 * @brief   This class is used to communicate between different processes using UDP on the same machine.
 * @note    Both sender and receiver must have the same datatype and channel. Otherwise errors will occur.
*/
template <typename TYPE>
class UdpIpc {
private:

    int sockfd_;
    sockaddr_in serverAddr_;

    bool initialised_ = false;


public:

    UdpIpc();

    ~UdpIpc();

    /**
     * @brief   Initialises the UPD communication. The port is determined using the channel. Both sender and receiver must have the same port.
     * @note    Errors will occur if different data types are send over the same channel.
     * @param   channel Used to create a port number. Only values > 0 are valid. Values < 1 will deinit. 
     * @returns true if succesfully initialised, false otherwise.
    */
    bool init(int channel);

    /**
     * @brief Will send the given data. 
     * @param dataToSend A reference to the data to be sent.
     * @returns true if successfully sent, false otherwise.
    */
    bool sendData(const TYPE& dataToSend);

    /**
     * @brief Will receive data is any and place into the given data reference.
     * @param dataToReceive A reference to the data object where the received data will be written into.
     * @returns true if data successfully received, false otherwise.
    */
    bool receiveData(TYPE& dataToReceive);


};


template <typename TYPE>
UdpIpc<TYPE>::UdpIpc() {}

template <typename TYPE>
UdpIpc<TYPE>::~UdpIpc() {

    //Close the bound socket if initialised.
    if (initialised_) {
        close(sockfd_);
    }

}

template <typename TYPE>
bool UdpIpc<TYPE>::init(int channel) {

    //Close bound socket if already initialised.
    if (initialised_) {
        close(sockfd_);
    }
    
    //Only initialise if channel number is valid.
    if (channel > 0) {

        //Create the socket 
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) { //Error creating socket

            perror("Error creating socket");
            return false;

        } else { //Bind an address

            //Setup an address.
            auto port = channel + 12311;
            serverAddr_.sin_family = AF_INET;
            serverAddr_.sin_port = htons(port); // Choose a port number
            serverAddr_.sin_addr.s_addr = inet_addr("127.0.0.1"); 

            // Bind the socket to the server address
            if (bind(sockfd_, (struct sockaddr*)&serverAddr_, sizeof(serverAddr_)) == -1) {
                perror("Error binding socket");
                close(sockfd_); //Close the socket if an error occurs during binding.
                return false;
            }

        }

    } else {
        return false;
    }

    return true;

}

template <typename TYPE>
bool UdpIpc<TYPE>::sendData(const TYPE& dataToSend) {

    if (!initialised_)
        return false;

    auto ret = send(sockfd_, &dataToSend, sizeof(TYPE), 0);

    return ret == sizeof(TYPE); //Successfull if number bytes sent is same as size of datatype.

}

template <typename TYPE>
bool UdpIpc<TYPE>::receiveData(TYPE& dataToReceive) {

    if (!initialised_)
        return false;

    auto ret = recv(sockfd_, &dataToReceive, sizeof(TYPE), 0);

    return ret == sizeof(TYPE);

}


#endif // !UDP_IPC_HPP