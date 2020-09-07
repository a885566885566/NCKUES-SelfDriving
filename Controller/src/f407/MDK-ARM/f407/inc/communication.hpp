#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "configs.h"

#define CAN_DRIVER_FIFO CAN_RX_FIFO0
#define CAN_NAVIGATOR_FIFO CAN_RX_FIFO1

#define CAN_DRIVER_MSG_PENDING CAN_IT_RX_FIFO0_MSG_PENDING
#define CAN_NAVIGATOR_MSG_PENDING CAN_IT_RX_FIFO1_MSG_PENDING

#define CAN_DRIVER_MAILBOX CAN_TX_MAILBOX0
#define CAN_NAVIGATOR_MAILBOX CAN_TX_MAILBOX1

#define CAN_DRIVER_BANK 0
#define CAN_NAVIGATOR_BANK 2

#define CAN_DATA_LEN 4

typedef union{
  uint8_t array[CAN_DATA_LEN];
  float fvalue;   // 4 bytes
  uint32_t ivalue;
} COMMU_DATA;

class Communicator{
  private:
    // Timeout checker 
    uint32_t update_time;
    uint32_t update_interval;
    uint32_t timeout;

    uint32_t RxFifo;
    uint32_t RxMsgPending;
    uint32_t TargetId;
    uint32_t TxMailbox;
    uint32_t BankNum;
    
    // Number of message in a series
    uint8_t terminate_id;
    static const uint8_t IdOffset = CONF_ID_OFFSET;
    
  protected:
    COMMU_DATA msg_buf[CONF_COMMU_BUF_LEN];
  
    // Communication objects
    CAN_HandleTypeDef* hcan;
    CAN_TxHeaderTypeDef TxMeg;
    CAN_RxHeaderTypeDef RxMeg;
  
    // Call this function when new legal message accepted
    void update_timer();
  
  public:
    bool available;
    Communicator(CAN_HandleTypeDef* hcan_t, 
      uint32_t update_timeout,
      uint32_t RxFifo_t,
      uint32_t RxMsgPending_t,
      uint32_t TargetId_t,
      uint32_t TxMailbox_t,
      uint32_t BankNum_t, 
      uint8_t terminate_id_t);
  
    void reset();
    bool check_timeout();
    // All children class must overwrite this function to set up filter type
    bool get_message();
    HAL_StatusTypeDef send_message(uint8_t id, COMMU_DATA* const data);
    void filter_setup();
};

/* Driver communication */
typedef enum {MOTOR_RESET=0, MOTOR_STANBY} MOTOR_STATE;
typedef struct{
  float velocity;
  float current;
  MOTOR_STATE motor_state;
} MOTOR_INFO;

class DriverCommunicator:public Communicator{
  private:
  protected:
  public:
    DriverCommunicator(CAN_HandleTypeDef* hcan_t, 
      uint32_t update_timeout);
    // Return true, if received entire series.
    bool get_message(MOTOR_INFO* info);
};

/* Navigator communication */
typedef enum {NAVIGATOR_RESET=0, NAVIGATOR_ORIGIN, NAVIGATOR_UPDATE, NAVIGATOR_PATH} NAVIGATOR_STATE;
typedef struct{
  float a;
  float b;
  float vs;
  float ve;
  
  float x;
  float y;
  float yaw;
  float v;
  NAVIGATOR_STATE navigator_state;
} NAVIGATOR_INFO;

class NavigatorCommunicator:public Communicator{
  private:
  protected:
  public:
    NavigatorCommunicator(CAN_HandleTypeDef* hcan_t, 
      uint32_t update_timeout);
    // Return true, if received entire series.
    bool get_message(NAVIGATOR_INFO* info);
};

#endif
