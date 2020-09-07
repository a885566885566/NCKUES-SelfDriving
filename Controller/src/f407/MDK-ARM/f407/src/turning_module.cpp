#include "turning_module.hpp"
#include "stm32f4xx_hal.h"
#include "motor_control.hpp"

TurningSubsystem::TurningSubsystem():
      motor(&(TIM3->CCR1), 
        CONF_PIN_MOTOR_DIR_PORT, 
        CONF_PIN_MOTOR_DIR_MASK, 
        &(TIM2->CNT), 39374, 1), 
      shaftPI(0.2, 0.001, 1, 5, -5){
  // steering encoder, z signal
  zeroPort = CONF_PIN_SHAFT_Z_PORT; // Shaft encoder Z port
  zeroMask = CONF_PIN_SHAFT_Z_MASK; // Shaft encoder Z pin
      
  // clutch output signal
  clutchPort = CONF_PIN_CLUTCH_PORT;  // Clutch output port
  clutchMask = CONF_PIN_CLUTCH_MASK;  // Clutch output pin
      
  // Setup Encoder register address
  shaftEncoder = &(TIM4->CNT);
        
  set_clutch(CLUTCH_OFF);
}
      
void TurningSubsystem::reset(){
  motor.reset();
  shaftPI.reset();
  state_req = STATE_READY;
  set_state(STATE_READY);
}
      
void TurningSubsystem::set_clutch(bool set){
  if (set)
    HAL_GPIO_WritePin(clutchPort, clutchMask, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(clutchPort, clutchMask, GPIO_PIN_RESET);
}

void TurningSubsystem::set_shaft(double angle){
  if (state == STATE_AUTO)
    shaftPI.set_target(angle);
  else 
    shaftPI.set_target(0);
}

void TurningSubsystem::prepare(Buzzer* buz){
  if (set_state(STATE_SYSTEM_UP)){
    buz->set(Buzzer_SHORT, 500, 10);
    /* Zero point initialization */
    // Turn right 
    motor.set_speed(0.5);
    HAL_Delay(3000);
    motor.stop();
    // Turn left
    buz->set(Buzzer_SHORT, 100, 2);
    HAL_Delay(400);
    motor.set_speed(-0.2);
    // Wait for Z signal
    while( HAL_GPIO_ReadPin(zeroPort, zeroMask) == GPIO_PIN_RESET );
    motor.stop();
    buz->set(Buzzer_SHORT, 100, 2);
    HAL_Delay(100);
    set_state(STATE_READY);
  }
}

void TurningSubsystem::update(){
  shaftAngle = static_cast<double>(0x8FFF - static_cast<int32_t>(*(shaftEncoder)))/resolution;
  switch(state){
    case STATE_ERROR:
      // TODO: Handle error problem
      if (state_req == STATE_READY)
        set_state(STATE_READY);
    case STATE_READY:
      if (state_req == STATE_MANUAL)
        set_state(STATE_MANUAL);
      else if (state_req == STATE_AUTO)
        set_state(STATE_AUTO);
    case STATE_RESET:
    case STATE_MANUAL:
      set_clutch(CLUTCH_OFF);
      motor.stop();
      break;
    case STATE_SYSTEM_UP:
      // Velocity control
      set_clutch(CLUTCH_ON);
      break;
    case STATE_AUTO:
      // Position control
      set_clutch(CLUTCH_ON);
      double Vcmd = -shaftPI.update(shaftAngle);
      motor.set_speed(Vcmd);
      break;
  }
  motor.update();
}
