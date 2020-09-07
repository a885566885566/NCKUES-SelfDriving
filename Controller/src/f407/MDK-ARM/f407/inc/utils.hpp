#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
typedef enum {Buzzer_NONE, Buzzer_SHORT}BuzzerMode;
class Buzzer{
  private:
    GPIO_TypeDef* beepPort;
    uint16_t beepMask;
    uint32_t starting;
  
    BuzzerMode mode;
    uint16_t dur;
    uint16_t times;
  public:
    Buzzer(GPIO_TypeDef *port, uint16_t pinMask);
    void set(BuzzerMode mode_t, uint16_t dur_t, uint16_t times_t);
    void update();
};

typedef struct{
}BEEPER_CONFIG;

void utils_beep_set(volatile BEEPER_CONFIG* beeper, uint16_t mode, uint16_t dur, uint16_t times);
void utils_beep_update(volatile BEEPER_CONFIG* beeper);
#endif
