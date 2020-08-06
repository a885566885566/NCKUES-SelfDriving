#ifndef CONTROL_H
#define CONTROL_H

#include "definitions.h"
#include "utils.h"
#include "pid_control.h"
#include "encoder.h"

typedef enum {
    CONTROL_NONE=0,
    CONTROL_VELOCITY,
    CONTROL_CURRENT
}CONTROL_MODE;

typedef struct{
    volatile CONTROL_MODE mode;
    volatile PID_STRUCT pid_velocity;
    volatile PID_STRUCT pid_current;
    volatile double current_integrator;
    volatile double current_limitator;
    volatile ENCODER enc;
    volatile uint16_t current_counter;
    volatile uint16_t velocity_counter;
} CONTROL;

typedef struct{
    volatile double velocity;
    volatile double current;
    volatile double pwm_cmd;
} MOTOR_STATE;

void control_init(volatile CONTROL *ctrl);
void control_set_mode(volatile CONTROL *ctrl, CONTROL_MODE mode);
void control_set_speed(double velocity);
void control_set_current(double current);
void ISR_enable();
void ISR_disable();

#endif