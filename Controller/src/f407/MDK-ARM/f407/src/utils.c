#include "utils.h"
#include "stm32f4xx_hal.h"
void utils_beep_init(volatile BEEPER_CONFIG* beeper, GPIO_TypeDef *port, uint32_t pinMask){
  beeper->beepPort = port;
  beeper->beepMask = pinMask;
}
void utils_beep_set(volatile BEEPER_CONFIG* beeper, uint16_t mode, uint16_t dur, uint16_t times){
  beeper->starting = HAL_GetTick();
  beeper->mode = mode;
  beeper->dur = dur;
  beeper->times = times;
}

void utils_beep_update(volatile BEEPER_CONFIG* beeper){
  // No beeper running
  if(beeper->mode == 0) return;
  uint32_t diff = HAL_GetTick() - beeper->starting;
  switch(beeper->mode){
    case 1:
      if( diff > 2 * beeper->times * beeper->dur )
        beeper->mode = 0;
      else if( (diff/beeper->dur)%2 == 1 )
        HAL_GPIO_WritePin(beeper->beepPort, beeper->beepMask, GPIO_PIN_SET);
      else
        HAL_GPIO_WritePin(beeper->beepPort, beeper->beepMask, GPIO_PIN_RESET);
      break;
      
    default:
      break;
  }
};

