#ifndef CONFIGS_H
#define CONFIGS_H

#define ENCODER_RESOLUTION 13517

/* Controller GPIO setting */
#define MOTOR_PWM 
#define MOTOR_DIR 
#define MOTOR_ENCODER_A 
#define MOTOR_ENCODER_B 
#define CONF_PIN_SHAFT_Z_PORT GPIOD
#define CONF_PIN_SHAFT_Z_MASK GPIO_PIN_10
#define CONF_PIN_CLUTCH_PORT  GPIOD
#define CONF_PIN_CLUTCH_MASK  GPIO_PIN_15
#define CONF_PIN_MOTOR_DIR_PORT  GPIOD
#define CONF_PIN_MOTOR_DIR_MASK  GPIO_PIN_12
#define CONF_PIN_BUZZER_PORT  GPIOD
#define CONF_PIN_BUZZER_MASK  GPIO_PIN_14

/* Sensor correction constants */
// throttle
#define CONF_THROTTLE_MIN 0
#define CONF_THROTTLE_MAX 2000
// steering shaft
#define CONF_SHAFT_RESOLUTION 33

/* Path tracking configurations */
#define CONF_DRIVER_CYCLE_TIME 100  // ms

/* Communication configuration */
#define CONF_COMMU_MODE_VELOCITY 'V'  // velocity, drive motor to specific speed 
#define CONF_COMMU_MODE_CURRENT 'C'   // current, drive motor to specific current
#define CONF_COMMU_MODE_STOP  'S'     // stop, stop and release motor

// if controllor doesn't receive message from safe time,
// it will forcely turn into different manual mode and reset
// velocity command to 0 
#define CONF_DRIVER_COMMU_SAFE_TIME 100    // ms
#define CONF_NAVIGATOR_COMMU_SAFE_TIME 500    // ms

/* Autopilot configuratiton */

#endif
