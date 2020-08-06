#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define CENTRAL_CONTROLLER_ID 1
#define MOTOR_DRIVER_ID 2
#define NAVIGATOR_ID 3

#define CAN_FIFO CAN_RX_FIFO0
#define CAN_DATA_LEN 7

typedef struct{
  char mode;
  union{
    uint8_t array[CAN_DATA_LEN];
    float fvalue;   // 4 bytes
    int32_t ivalue; // 4 bytes
  } data;
} COMMU_DATA;

typedef struct{
  CAN_HandleTypeDef* hcan;
  CAN_TxHeaderTypeDef TxMeg;
  CAN_RxHeaderTypeDef RxMeg;
  uint32_t TxMailbox;
} COMMU_CONFIG;

void communication_init(COMMU_CONFIG* const commu_t, CAN_HandleTypeDef* const hcan_t);

/*  */
HAL_StatusTypeDef send_msg(COMMU_CONFIG* const commu_t, const uint16_t target_id, volatile const COMMU_DATA * const pData);

/* Return num of bytes received */
uint8_t poll_for_msg(COMMU_CONFIG* const commu_t, volatile COMMU_DATA *const pData);
#endif
