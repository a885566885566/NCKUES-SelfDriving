#ifndef TURNING_CONTROL_H
#define TURNING_CONTROL_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "configs.h"
#include "motor_control.h"
#include "pid_control.h"
#include "utils.h"

/* Mode of turning shaft control */
typedef enum {
  FREE = 0,
  SPEED_CONTROL,
  POSITION_CONTROL
}TURNING_MODE;

typedef struct{
  TURNING_MODE mode;
  float cmdAngle;
  float shaftAngle;
  
  GPIO_TypeDef* zeroPort;
  uint32_t zeroMask;
  
  GPIO_TypeDef* clutchPort;
  uint32_t clutchMask;
  
  volatile uint32_t* motorEncoder;
  volatile uint32_t* shaftEncoder;
  
  volatile MOTOR_CONFIG motor;
  volatile PID_STRUCT PID_motor;
  
  volatile PID_STRUCT PID_turning;
}TURNING_CONFIG;

void turning_init(volatile TURNING_CONFIG *const turning, 
    GPIO_TypeDef* const zPort, 
    const uint32_t zPinMask,
    GPIO_TypeDef* const cPort, 
    const uint32_t cPinMask,
    volatile uint32_t* const motorEnc, 
    volatile uint32_t* const shaftEnc,
    volatile uint32_t* const pwmReg);
/* Zero point initialization */
void turning_prepare(volatile TURNING_CONFIG* const turning, volatile BEEPER_CONFIG* const beeper);

void turning_set_mode(volatile TURNING_CONFIG* const turning, const TURNING_MODE mode);
    
void turning_update(volatile TURNING_CONFIG* const turning);
    
#endif

