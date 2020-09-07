#include "utils.hpp"
#include "stm32f4xx_hal.h"

Buzzer::Buzzer(GPIO_TypeDef *port, uint16_t pinMask){
  beepPort = port;
  beepMask = pinMask;
  mode = Buzzer_NONE;
}

void Buzzer::set(BuzzerMode mode_t, uint16_t dur_t, uint16_t times_t){
  starting = HAL_GetTick();
  mode = mode_t;
  dur = dur_t;
  times = times_t;
}

void Buzzer::update(){
  // No beeper running
  if(mode == Buzzer_NONE) return;
  uint32_t diff = HAL_GetTick() - starting;
  if (mode == Buzzer_SHORT){
      if( diff > 2 * times * dur )
        mode = Buzzer_NONE;
      else if( (diff / dur)%2 == 1 )
        HAL_GPIO_WritePin(beepPort, beepMask, GPIO_PIN_SET);
      else
        HAL_GPIO_WritePin(beepPort, beepMask, GPIO_PIN_RESET);
  }
}

