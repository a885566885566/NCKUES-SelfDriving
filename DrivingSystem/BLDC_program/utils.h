#ifndef UTILS_H
#define UTILS_H

#include "definitions.h"
#include "Arduino.h"

typedef unsigned int uint16_t;
typedef int int16_t;
typedef unsigned long uint32_t;
typedef long int32_t;

typedef struct{
  uint16_t beepPort;
  uint32_t starting;
  uint16_t mode;
  uint16_t dur;
  uint16_t times;
}BEEPER_CONFIG;

void pin_init();
void system_init();
bool system_check();
void motor_key(bool key, bool mode=CONF_MOTOR_CW);
void write_motor(double velocity);
double read_current();

void utils_beep_init(volatile BEEPER_CONFIG* beeper, uint16_t port);
void utils_beep_set(volatile BEEPER_CONFIG* beeper, uint16_t mode, uint16_t dur, uint16_t times);
void utils_beep_update(volatile BEEPER_CONFIG* beeper);
#endif
