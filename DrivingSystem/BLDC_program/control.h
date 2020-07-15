#ifndef CONTROL_H
#define CONTROL_H

#include "definitions.h"
#include "utils.h"
#include "pid_control.h"
#include "encoder.h"

typedef struct{
    volatile PID_STRUCT pid_velocity;
    volatile PID_STRUCT pid_current;
    volatile double current_integrator;
    volatile double current_limitator;
    volatile ENCODER enc;
    volatile uint16_t current_counter;
    volatile uint16_t velocity_counter;
} CONTROL;


void control_init(volatile CONTROL *ctrl);
void control_set_speed(double velocity);
void ISR_enable();
void ISR_disable();

#endif