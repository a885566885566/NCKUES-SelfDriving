#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define CAN_DRIVER_FIFO CAN_RX_FIFO0
#define CAN_NAVIGATOR_FIFO CAN_RX_FIFO1

#define CAN_DRIVER_MSG_PENDING CAN_IT_RX_FIFO0_MSG_PENDING
#define CAN_NAVIGATOR_MSG_PENDING CAN_IT_RX_FIFO1_MSG_PENDING

#define CAN_DRIVER_MAILBOX CAN_TX_MAILBOX0
#define CAN_NAVIGATOR_MAILBOX CAN_TX_MAILBOX1
#define CAN_DATA_LEN 8

typedef struct{
  char mode;
  union{
    uint8_t array[CAN_DATA_LEN];
    float fvalue;   // 4 bytes
    int32_t ivalue; // 4 bytes
  } data;
} COMMU_DATA;

typedef union{
    uint8_t array[8];
    double fvalue;   // 8 bytes
} COMMU_DATA_DOUBLE;

typedef struct{
  CAN_HandleTypeDef* hcan;
  CAN_TxHeaderTypeDef TxMeg;
  CAN_RxHeaderTypeDef RxMeg;
  uint32_t TxMailbox;
} COMMU_CONFIG;

HAL_StatusTypeDef communication_init(volatile COMMU_CONFIG* const commu_t, CAN_HandleTypeDef* const hcan_t);

/*  */
HAL_StatusTypeDef send_msg(volatile COMMU_CONFIG* const commu_t, const uint16_t target_id, volatile const COMMU_DATA * const pData);

/* Return num of bytes received */
uint8_t poll_for_msg(volatile COMMU_CONFIG* volatile const commu_t, volatile COMMU_DATA *const pData);

HAL_StatusTypeDef commu_navigator_init(volatile COMMU_CONFIG* commu_t, CAN_HandleTypeDef* hcan_t);

HAL_StatusTypeDef send_navigator_msg(volatile COMMU_CONFIG* commu_t);
#endif
