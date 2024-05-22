#ifndef IPC_H
#define IPC_H

#include <stdint.h>

#include "../include/Datastruct.h"

/**
 * @brief Sends the given pose estimation to the RODOS proccess.
*/
void sendORPEEstimation(OrpePoseEst& poseEst);

/**
 * @brief Gets the latest power command from RODOS proccess.
 * @param powerOn Reference to the power command.
 * @returns true if new, false otherwise.
*/
bool getORPEPowerCommand(bool& powerOn);

/**
 * @brief Initializes the IPC.
*/
void ipcInit();

/**
 * @brief deinitializes the IPC.
*/
void ipcDeinit();


#endif