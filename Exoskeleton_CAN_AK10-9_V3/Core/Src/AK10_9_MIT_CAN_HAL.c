#include "AK10_9_MIT_CAN_HAL.h"

/* The only HAL CAN sender in this package. Hardware TX is not even called
 * unless AK10_9_TX_ENABLE has been explicitly enabled and reviewed. */
int AK10_9_TX_HAL_QueueFrame(const AK10_9_MIT_Frame *frame, void *user)
{
#if AK10_9_TX_ENABLE
    CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *)user;
    CAN_TxHeaderTypeDef hdr = {0};
    uint32_t mailbox = 0;
    if (!frame || !hcan || hcan->Instance != CAN1 ||
        hcan->Init.Mode != CAN_MODE_NORMAL || frame->dlc > 8u ||
        frame->extended_id > 0x1FFFFFFFu ||
        HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING ||
        HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u) return 0;
    hdr.ExtId = frame->extended_id;
    hdr.IDE = CAN_ID_EXT;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = frame->dlc;
    hdr.TransmitGlobalTime = DISABLE;
    /* Queue acceptance != bus ACK or motor's confirmed action. */
    return HAL_CAN_AddTxMessage(hcan, &hdr, (uint8_t *)frame->data, &mailbox) == HAL_OK;
#else
    (void)frame; (void)user;
    return 0; /* No CAN API call at all in preview builds. */
#endif
}
