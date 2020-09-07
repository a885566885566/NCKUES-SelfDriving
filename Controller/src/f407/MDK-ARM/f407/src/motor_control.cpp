#include "motor_control.hpp"
#include "stm32f4xx_hal.h"

Motor::Motor(volatile uint32_t* const cnt, 
          GPIO_TypeDef* const port, 
          const uint16_t pinMask, 
          volatile uint32_t* const motorEnc, 
          double res, 
          uint8_t dt_t)
    :velocityPI(10, 0.01, dt_t, 500, -500){
  // Pin setup
  pwmSig = cnt;
  dirPinPort = port;
  dirPinMask = pinMask;
  motorEncoder = motorEnc;
            
  // Global settings          
  resolution = res;
  dt = dt_t;
  reset();
}

void Motor::reset(){
  encPreCount = static_cast<int32_t>(*motorEncoder);
  velocityPI.reset();
  speed = 0;
}

void Motor::set_speed(double cmd){
  velocityPI.set_target(cmd);
}

double Motor::read_speed(){
  static int32_t diff;
  diff = static_cast<int32_t>(*motorEncoder) - encPreCount;
  speed = 1000.0 * diff / resolution / dt;
  encPreCount = static_cast<int32_t>(*motorEncoder);
  return speed;
}

void Motor::drive(double cmd){
  if(cmd > pwmMax) cmd = pwmMax;
  else if(cmd < -pwmMax) cmd = -pwmMax;
  // Direction 
  if(cmd > 0)
    HAL_GPIO_WritePin(dirPinPort, dirPinMask, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(dirPinPort, dirPinMask, GPIO_PIN_RESET);
  // Pwm
  *(pwmSig) = static_cast<uint32_t>(cmd);
}

void Motor::stop(){
  velocityPI.reset();
  drive(0);
}

void Motor::update(){
  double output = velocityPI.update(read_speed());
  drive(output);
}
