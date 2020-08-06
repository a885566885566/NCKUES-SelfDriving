#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct{
  GPIO_TypeDef* beepPort;
  uint32_t beepMask;
  uint32_t starting;
  uint16_t mode;
  uint16_t dur;
  uint16_t times;
}BEEPER_CONFIG;

void utils_beep_init(volatile BEEPER_CONFIG* beeper, GPIO_TypeDef *port, uint32_t pinMask);
void utils_beep_set(volatile BEEPER_CONFIG* beeper, uint16_t mode, uint16_t dur, uint16_t times);
void utils_beep_update(volatile BEEPER_CONFIG* beeper);
#endif
