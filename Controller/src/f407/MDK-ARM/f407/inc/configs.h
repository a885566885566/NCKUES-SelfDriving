#ifndef CONFIGS_H
#define CONFIGS_H

#define MATH_PI 3.1415926
#define ENCODER_RESOLUTION 13517
#define CONF_PWM_MAXIMA 1000

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
#define CONF_THROTTLE_MIN 1300
#define CONF_THROTTLE_MAX 4096
// steering shaft
#define CONF_SHAFT_RESOLUTION 33

/* Path tracking configurations */
/* Car Model */
#define CONF_CAR_LF 0.7f
#define CONF_CAR_LR 0.6f
#define CONF_CAR_LEN 1.4f
#define CONF_CAR_MAX_STEER 0.436f      //radian, 25 degree
                                // (2 * 0.45 * pi) * (7 /68)
#define CONF_GEAR_RATIO 0.291f*0.516f   // meter/rotation

#define CONF_DRIVER_CYCLE_TIME 100    // ms

/* Communication configuration */
#define CENTRAL_CONTROLLER_ID 1
#define MOTOR_DRIVER_ID 2
#define NAVIGATOR_ID 3

#define CONF_COMMU_BUF_LEN 5
#define CONF_ID_OFFSET 4
#define CONF_DRIVER_TERMINATE_ID 3
#define CONF_NAVIGATOR_TERMINATE_ID 4

#define CONF_NAVIGATOR_ID_A 0
#define CONF_NAVIGATOR_ID_B 1
#define CONF_NAVIGATOR_ID_VS 2
#define CONF_NAVIGATOR_ID_VE 3
#define CONF_NAVIGATOR_ID_X 0
#define CONF_NAVIGATOR_ID_Y 1
#define CONF_NAVIGATOR_ID_YAW 2
#define CONF_NAVIGATOR_ID_V 3

#define CONF_DRIVER_MODE_C 0
#define CONF_DRIVER_MODE_V 1
#define CONF_DRIVER_MODE_STOP 2

#define CONF_COMMU_MODE_VELOCITY 'V'  // velocity, drive motor to specific speed 
#define CONF_COMMU_MODE_CURRENT 'C'   // current, drive motor to specific current
#define CONF_COMMU_MODE_STOP  'S'     // stop, stop and release motor

// if controllor doesn't receive message from safe time,
// it will forcely turn into different manual mode and reset
// velocity command to 0 
#define CONF_DRIVER_COMMU_SAFE_TIME 100    // ms
#define CONF_NAVIGATOR_COMMU_SAFE_TIME 500    // ms

/* Autopilot configuratiton */
#define CONF_CAR_MODEL_UPDATE_DT 0.001f  // s
#endif
