#include "communication.h"
#include "stm32f4xx_hal.h"
#include "configs.h"


HAL_StatusTypeDef communication_init(volatile COMMU_CONFIG* commu_t, CAN_HandleTypeDef* hcan_t){
  commu_t->hcan = hcan_t;
  // Setting up for can receiver filter 
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_FIFO;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdLow = MOTOR_DRIVER_ID << 5;
  sFilterConfig.FilterIdHigh = NAVIGATOR_ID << 5;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_StatusTypeDef HAL_Status=HAL_CAN_ConfigFilter(commu_t->hcan, &sFilterConfig);
  if(HAL_Status!=HAL_OK){
    return HAL_Status;
  }
  // Setting up for can transmitter
  commu_t->TxMeg.IDE = CAN_ID_STD;    // use standard id length
  commu_t->TxMeg.RTR = CAN_RTR_DATA;  // 
  commu_t->TxMeg.DLC = 8;             // 8 bytes per transmit
  commu_t->TxMailbox = CAN_TX_MAILBOX0;
  if(HAL_Status!=HAL_OK){
    return HAL_Status;
  }
  HAL_Status = HAL_CAN_Start(commu_t->hcan);
  HAL_CAN_ActivateNotification(commu_t->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  return HAL_Status;
}

HAL_StatusTypeDef send_msg(volatile COMMU_CONFIG* commu_t, 
    const uint16_t target_id, 
    volatile const COMMU_DATA * const pData){
  static uint8_t raw[8];
  commu_t->TxMeg.StdId = target_id;
  uint16_t FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(commu_t->hcan);
  if (FreeTxNum == 0)
      return HAL_BUSY;
  // Memory copy to buffer
  raw[0] = pData->mode;
  for (int i=1; i<8; i++)
    raw[i] = pData->data.array[i-1];
  
  HAL_StatusTypeDef HAL_RetVal = HAL_CAN_AddTxMessage(commu_t->hcan, 
        &(commu_t->TxMeg), raw, (uint32_t*)(CAN_TX_MAILBOX0));
  return HAL_RetVal;
}

uint8_t poll_for_msg(volatile COMMU_CONFIG* commu_t, volatile COMMU_DATA *pData){
  static uint8_t raw[8];
  if (HAL_CAN_GetRxFifoFillLevel(commu_t->hcan, CAN_FIFO) > 0){
    HAL_CAN_GetRxMessage(commu_t->hcan, CAN_FIFO, &(commu_t->RxMeg), raw);
    pData->mode = (char)raw[0];
    for (int i=1; i<8; i++)
      pData->data.array[i-1] = raw[i];
    return commu_t->RxMeg.DLC;
  }
  return 0;
}
