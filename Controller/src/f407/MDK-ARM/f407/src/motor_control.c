#include "motor_control.h"
#include "stm32f4xx_hal.h"

void motor_output_init(volatile MOTOR_CONFIG* const motor, 
    volatile uint32_t* const cnt, 
    GPIO_TypeDef* const port, 
    const uint32_t pinMask){
  motor->pwmSig = cnt;
  motor->dirPort = port;
  motor->dirMask = pinMask;
}

void motor_init(volatile MOTOR_CONFIG* const motor, 
    uint32_t enc, uint32_t res, 
    uint32_t Dt, uint32_t Max){
      
  motor->direction = 0;
  motor->speed = 0;
  motor->rotation = 0;
  motor->cmd = 0;
  motor->preCount = enc;
  motor->acc = 0;
  
  motor->dt = Dt;
  motor->max = Max;
  motor->resolution = res;
}

void motor_read_speed(volatile MOTOR_CONFIG* const motor, uint32_t encoder){
  int32_t diff = encoder - motor->preCount;
  motor->speed = 1000.0 * diff / motor->resolution / motor->dt;
  motor->preCount = encoder;
}

void motor_drive(volatile MOTOR_CONFIG* const motor){
  if(motor->acc > motor->max) motor->acc = motor->max;
  else if(motor->acc < -motor->max) motor->acc = -motor->max;
  motor->cmd = motor->acc;
  if(motor->cmd > 0){
    HAL_GPIO_WritePin(motor->dirPort, motor->dirMask, GPIO_PIN_SET);
    *(motor->pwmSig) = motor->cmd;
  }
  else{
    HAL_GPIO_WritePin(motor->dirPort, motor->dirMask, GPIO_PIN_RESET);
    *(motor->pwmSig) = -motor->cmd;
  }
}

void motor_emergency_stop(volatile MOTOR_CONFIG* const motor){
  motor->acc = 0;
  motor_drive(motor);
};
