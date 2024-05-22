#ifndef IPC_H
#define IPC_H

#include <stdint.h>

#include "../include/Datastruct.h"

/**
 * @brief Sends the given power command to the ORPE proccess.
*/
void sendORPEPowerCommand(bool powerOn);

/**
 * @brief Gets the latest pose estimation.
 * @param poseEst Reference to the pose estimation.
 * @returns true if new, false otherwise.
*/
bool getORPEEstimation(OrpePoseEst& poseEst);

/**
 * @brief Initializes the IPC.
*/
void ipcInit();

/**
 * @brief deinitializes the IPC.
*/
void ipcDeinit();


#endif