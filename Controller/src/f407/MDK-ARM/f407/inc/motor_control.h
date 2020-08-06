#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct{
  double speed;
  int8_t direction;
  int32_t angle;
  int32_t rotation;
  int16_t cmd;
  double acc;
  
  uint32_t preCount;
  int16_t max;
  uint32_t resolution;
  uint16_t dt;
  
  volatile uint32_t* pwmSig;
  GPIO_TypeDef* dirPort;
  uint32_t dirMask;
}MOTOR_CONFIG;

void motor_init(volatile MOTOR_CONFIG* const motor, uint32_t enc, uint32_t res, uint32_t Dt, uint32_t Max);
void motor_output_init(volatile MOTOR_CONFIG* const motor, volatile uint32_t* const cnt, GPIO_TypeDef* const port, const uint32_t pinMask);
void motor_read_speed(volatile MOTOR_CONFIG* const motor, uint32_t encoder);
void motor_drive(volatile MOTOR_CONFIG* const motor);
void motor_emergency_stop(volatile MOTOR_CONFIG* const motor);
#endif
