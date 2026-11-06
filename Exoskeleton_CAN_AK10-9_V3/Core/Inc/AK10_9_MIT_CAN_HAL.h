#ifndef AK10_9_MIT_CAN_HAL_H
#define AK10_9_MIT_CAN_HAL_H
#include "main.h"
#include "AK10_9_MIT_CAN_TX.h"
/* Application provides CAN_HandleTypeDef hcan1; pass &hcan1 in user argument. */
int AK10_9_TX_HAL_QueueFrame(const AK10_9_MIT_Frame *frame, void *user);
#endif
