#include "communication.hpp"
#include "stm32f4xx_hal.h"
#include "configs.h"
#include <string.h>

/* Communicator */
Communicator::Communicator(
CAN_HandleTypeDef* hcan_t, 
uint32_t update_timeout,
uint32_t RxFifo_t,
uint32_t RxMsgPending_t,
uint32_t TargetId_t,
uint32_t TxMailbox_t,
uint32_t BankNum_t, 
uint8_t terminate_id_t){
  hcan = hcan_t;
  RxFifo = RxFifo_t;
  timeout = update_timeout;
  RxMsgPending = RxMsgPending_t;
  TargetId = TargetId_t;
  TxMailbox = TxMailbox_t;
  BankNum = BankNum_t;
  terminate_id = terminate_id_t;
  
  // Setting up for can transmitter
  TxMeg.IDE = CAN_ID_STD;    // use standard id length
  TxMeg.RTR = CAN_RTR_DATA;  // 
  TxMeg.DLC = CAN_DATA_LEN;  // 4 bytes per transmit
  
  // Start can routine
  filter_setup();
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, RxMsgPending);
}

bool Communicator::check_timeout(){
  update_interval = HAL_GetTick() - update_time;
  if (update_interval > timeout)  return true;
  else                            return false;
}

// Call this function when new legal message accepted
void Communicator::update_timer(){
  update_time = HAL_GetTick();
}

/* Filter Setting:
 * Mode: IDMASK
 * FilterBank: 2
 * MaskID: NAVIGATOR_ID */
void Communicator::filter_setup(){
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterFIFOAssignment = RxFifo;
  sFilterConfig.FilterBank = BankNum;
  sFilterConfig.FilterMaskIdHigh = TargetId << (5+IdOffset);
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
}

/* This function check if there is a new message in FIFO
 * and copy them into msg buffer. If the coming message is 
 * the end of the series, it will set `available` to true.
 * It will return the length of received data. */
bool Communicator::get_message(){
  if (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) > 0){
    uint16_t filter = RxMeg.StdId % IdOffset;
    HAL_CAN_GetRxMessage(hcan, RxFifo, &RxMeg, msg_buf[filter].array);
    
    // If current id is the end of series
    if (filter == terminate_id) available = true;
    // Update last_update_time
    update_timer();
    return available;  
  }
  return 0;
}

HAL_StatusTypeDef Communicator::send_message(uint8_t id, COMMU_DATA* const data){
  TxMeg.StdId = (TargetId << IdOffset) + id;
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
      return HAL_BUSY;
  
  HAL_StatusTypeDef HAL_RetVal = HAL_CAN_AddTxMessage(hcan, 
        &TxMeg, data->array, (uint32_t*)(TxMailbox));
  return HAL_RetVal;
}

/* Driver Communicator */
DriverCommunicator::DriverCommunicator(CAN_HandleTypeDef* hcan_t, uint32_t update_timeout)
  :Communicator(hcan_t, update_timeout,
    CAN_DRIVER_FIFO, 
    CAN_DRIVER_MSG_PENDING,
    MOTOR_DRIVER_ID,
    CAN_DRIVER_MAILBOX,
    CAN_DRIVER_BANK, 
    CONF_DRIVER_TERMINATE_ID){
}
    
/* Return true, if received entire series. It will copy 
 * infomations from buffer to the given MOTOR_INFO object.*/
bool DriverCommunicator::get_message(MOTOR_INFO* info){
  Communicator::get_message();
  if (available){
    info->velocity = msg_buf[CONF_DRIVER_MODE_V].fvalue;
    info->current = msg_buf[CONF_DRIVER_MODE_C].fvalue;
    info->motor_state = static_cast<MOTOR_STATE>(msg_buf[CONF_DRIVER_TERMINATE_ID].ivalue);
    available = false;
    return true;
  }
  return false;
}

/* Navigator Communicator */
NavigatorCommunicator::NavigatorCommunicator(CAN_HandleTypeDef* hcan_t, uint32_t update_timeout)
  :Communicator(hcan_t, update_timeout,
    CAN_NAVIGATOR_FIFO, 
    CAN_NAVIGATOR_MSG_PENDING,
    NAVIGATOR_ID,
    CAN_NAVIGATOR_MAILBOX,
    CAN_NAVIGATOR_BANK, 
    CONF_NAVIGATOR_TERMINATE_ID){
}
      
/* Return true, if received entire series. It will copy 
 * infomations from buffer to the given MOTOR_INFO object.*/
bool NavigatorCommunicator::get_message(NAVIGATOR_INFO* info){
  Communicator::get_message();
  if (available){
    if (msg_buf[CONF_NAVIGATOR_TERMINATE_ID].ivalue == NAVIGATOR_PATH){
      info->a = msg_buf[CONF_NAVIGATOR_ID_A].fvalue;
      info->b = msg_buf[CONF_NAVIGATOR_ID_B].fvalue;
      info->vs = msg_buf[CONF_NAVIGATOR_ID_VS].fvalue;
      info->ve = msg_buf[CONF_NAVIGATOR_ID_VE].fvalue;
    }
    else {
      info->x = msg_buf[CONF_NAVIGATOR_ID_X].fvalue;
      info->y = msg_buf[CONF_NAVIGATOR_ID_Y].fvalue;
      info->yaw = msg_buf[CONF_NAVIGATOR_ID_YAW].fvalue;
      info->v = msg_buf[CONF_NAVIGATOR_ID_V].fvalue;
    }
    info->navigator_state = static_cast<NAVIGATOR_STATE>(msg_buf[CONF_NAVIGATOR_TERMINATE_ID].ivalue);
    available = false;
    return true;
  }
  return false;
}
