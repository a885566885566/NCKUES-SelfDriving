#ifndef DEF_H
#define DEF_H

/* ---- OUTPUT ---- */
#define PIN_LA 6
#define PIN_VE 5
#define PIN_TD 4
#define PIN_DIR 7
#define PIN_RELEASE 8
#define PIN_BUZZER A1

/* ---- INPUT ---- */
#define PIN_REV 2
#define PIN_TG 3
#define PIN_A A2
#define PIN_B A3
#define PIN_Z A5
#define PIN_CUR_A A0
#define PIN_CUR_B A4

/* ---- CONFIGURATION ---- */
#define CONF_DEAD_TIME true      // true for 3.8 us, false for 2.6 us
#define CONF_MOTOT_DIR true      // inverse motor direction
#define CONF_LEAD_ANGLE 0        // range from 0 to 58 degree
#define CONF_MOTOR_CW true
#define CONF_MOTOR_CCW false

#define CONF_PWM_LIM 150    // upper and lower bound of velocity pwm out
#define CONF_VELOCITY_RATIO      // omega = ratio * encoder
#define CONF_OMEGA_LIM 1000      // motor angular velocity limit

#define CONF_CURRENT_RATIO 1     // current(A) = ratio * analog read
#define CONF_CURRENT_FILTER_DECAY 0.9
#define CONF_CURRENT_BASELINE 509   // Value for zero current

#define CONF_CURRENT_LIM 0.5
#define CONF_CURRENT_LIM_START 1    // current limit in ampere
#define CONF_CURRENT_LIM_END 1.5      // current limit in ampere
#define CONF_CURRENT_LIM_PARA_A -2    // Use a line to generate a 0~1 decay number
#define CONF_CURRENT_LIM_PARA_B 3     // A=-5=-1/(0.6-0.4), B=3=-A*0.6

#define CONF_ENCODER_RESOLUTION 50.0    // 0.05 rotation/step * 1000 s/ms
#define CONF_ENCODER_TP 0.5      // 500 us, sampling with 2kHz
#define CONF_ENCODER_TS 100      // 50 ms, approximate speed with 10Hz
#define CONF_ENCODER_TS_COUNT 200// 100 ms / 500 us, approximate speed with 10Hz

#define CONF_CONTROL_CURRENT_DT  10     // current feedback loop 10 ms
#define CONF_CONTROL_CURRENT_COUNT  20  // 20 * 500 us = 10 ms
#define CONF_CONTROL_VELOCITY_DT  100   // velocity feedback loop 100 ms
#define CONF_CONTROL_VELOCITY_COUNT 200 // 200 * 500 us = 100 ms

#define CONF_MODE_MOTOR 'M'     // motor, drive motor to specific speed 
#define CONF_MODE_STOP  'S'     // stop, stop and release motor
#define CONF_MODE_READ  'R'     // read, obtain informations from driver

#endif