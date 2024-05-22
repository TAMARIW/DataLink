#include <iostream>
#include <cstring>
#include <arpa/inet.h> // htons, inet_addr
#include <netinet/in.h> // sockaddr_in
#include <sys/types.h> // uint16_t
#include <sys/socket.h> // socket, sendto
#include <unistd.h> // close

#include "../include/Datastruct.h"
#include "ipcRODOS.h"

int sockfd;
sockaddr_in serverAddr;
int sockfdPWR;
sockaddr_in serverAddrPWR;

void sendORPEPowerCommand(bool powerOn) {
    
    // Set up the client address
    sockaddr_in clientAddr;
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(12315); // Same port as the server
    clientAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Loopback address for local communication

    // Send the struct over UDP
    if (sendto(sockfdPWR, &powerOn, sizeof(powerOn), 0, (sockaddr*)&clientAddr, sizeof(clientAddr)) == -1) {
        printf("Error sending data");
    }

}

bool getORPEEstimation(OrpePoseEst& poseEst) {

    socklen_t clientAddrLen = sizeof(OrpePoseEst);

    ssize_t bytesReceived = recvfrom(sockfd, &poseEst, sizeof(OrpePoseEst), 0,
                                     (struct sockaddr*)&serverAddr, &clientAddrLen);

    if (bytesReceived == -1) {
        //perror("Error receiving data");
        return false;
    }

    return true;
    
}

void ipcInit() {

    // Create a socket for communication
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("Error creating socket");
        return;
    }

    // Set up the server address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(41523); // Same port as the sender
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to the server address
    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("Error binding socket");
        close(sockfd);
        return;
    }

    // Create a socket for communication
    sockfdPWR = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfdPWR == -1) {
        perror("Error creating socket");
        return;
    }

    // Set up the server address
    serverAddrPWR.sin_family = AF_INET;
    serverAddrPWR.sin_port = htons(12314); // Choose a port number
    serverAddrPWR.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to the server address
    if (bind(sockfdPWR, (struct sockaddr*)&serverAddrPWR, sizeof(serverAddrPWR)) == -1) {
        perror("Error binding socket");
        close(sockfdPWR);
        return;
    }
    
}

void ipcDeinit() {
    
    close(sockfd);

}