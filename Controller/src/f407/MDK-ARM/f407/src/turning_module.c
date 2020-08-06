#include "turning_module.h"

void turning_init(volatile TURNING_CONFIG *const turning, 
    GPIO_TypeDef* const zPort, 
    const uint32_t zPinMask,
    GPIO_TypeDef* const cPort, 
    const uint32_t cPinMask,
    volatile uint32_t* const motorEnc, 
    volatile uint32_t* const shaftEnc,
    volatile uint32_t* const pwmReg){
    
  // steering encoder, z signal
  turning->zeroPort = zPort;
  turning->zeroMask = zPinMask;
      
  // clutch output signal
  turning->clutchPort = cPort;
  turning->clutchMask = cPinMask;
      
  // mode init
  turning_set_mode(turning, FREE);
  turning->cmdAngle = 0;
      
  // Setup Encoder register address
  turning->motorEncoder = motorEnc;
  turning->shaftEncoder = shaftEnc;
  
  //motor_config, encoder_counter, resolution, dt(ms), max_value
  motor_init(&(turning->motor), *(turning->motorEncoder), 39374, 1, 900);
  motor_output_init(&(turning->motor), pwmReg, 
      CONF_PIN_MOTOR_DIR_PORT, 
      CONF_PIN_MOTOR_DIR_MASK);
  
  // double KP, double KI, double KD, double DT, double upper, double lower, double Decay
  PID_init(&(turning->PID_motor), 10, 0.01, 0, 1, 500, -500, 0.9);
  PID_init(&(turning->PID_turning), 0.2, 0.001, 0, 1, 5, -5, 0.9);
}
    
/* Zero point initialization */
void turning_prepare(volatile TURNING_CONFIG *const turning, 
    volatile BEEPER_CONFIG *const beeper){
  utils_beep_set(beeper, 1, 500, 10);
  
  PIDsetTarget(&(turning->PID_motor), 0.5);
  for(int i=0; i<3000; i++){
    HAL_Delay(1);
  }
  PIDsetTarget(&(turning->PID_motor), 0);
  HAL_Delay(100);
  utils_beep_set(beeper, 1, 100, 2);
  HAL_Delay(400);
  
  PIDsetTarget(&(turning->PID_motor), -0.2);
  while( HAL_GPIO_ReadPin(turning->zeroPort, turning->zeroMask) == GPIO_PIN_RESET );

  PIDsetTarget(&(turning->PID_motor), 0);
  motor_emergency_stop(&(turning->motor));
  // Set turning encoder to middle of its max value
  *(turning->shaftEncoder) = 0x8FFF;
  utils_beep_set(beeper, 1, 100, 2);
  HAL_Delay(1000);
  turning->mode = POSITION_CONTROL;
}

void turning_set_mode(volatile TURNING_CONFIG *const turning, const TURNING_MODE mode){
  if (turning->mode == mode) return;
  if (mode == FREE){
    HAL_GPIO_WritePin(turning->clutchPort, turning->clutchMask, GPIO_PIN_RESET);
    turning->cmdAngle = 0;
  }
  else 
    HAL_GPIO_WritePin(turning->clutchPort, turning->clutchMask, GPIO_PIN_SET);
  turning->mode = mode;
}

void turning_update(volatile TURNING_CONFIG *const turning){
  if (turning->mode != FREE){
    motor_read_speed(&(turning->motor), *(turning->motorEncoder));
    turning->motor.acc += PID_update(&(turning->PID_motor), turning->motor.speed);
    motor_drive(&(turning->motor));
    
    if(turning->mode == POSITION_CONTROL){
      turning->shaftAngle = ((double)0x8FFF - (double)*(turning->shaftEncoder))/CONF_SHAFT_RESOLUTION;
      double Vcmd = -PID_update(&(turning->PID_turning), turning->shaftAngle);
      PIDsetTarget(&(turning->PID_motor), Vcmd);
    }
  }
}
