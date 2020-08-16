#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "configs.h"
#include "communication.h"

/* Turning control module */
#include "pid_control.h"
#include "motor_control.h"
#include "turning_module.h"

typedef enum { PILOT_RESET, PILOT_MANUAL, PILOT_AUTO}PILOT_MODE;
typedef enum {EMPTY, PENDING}COMMU_FLAF;


/* Real time physical car states */
typedef struct{
  float velocity_rear;
  float angle_steering;  // degree
  float throttle;        // percentage
} CAR_STATE;

/* Reference trajectory */
typedef struct{
  float a;
  float b;
} TRAJECTORY;

typedef struct{
  PILOT_MODE pilot_mode;
  
  /* Communications */
  COMMU_DATA driver_data;
  COMMU_DATA driver_cmd;
  uint32_t driver_last_update_time; // Record last communicate time stamp
  uint32_t navigator_last_update_time;
  COMMU_FLAF commu_driver_flag;     // Send request to driver
  COMMU_FLAF commu_navigator_flag;  // Send request to navigator
  uint16_t driver_update_interval;
  uint16_t navigator_update_interval;
  
  /* Sensors */
  ADC_HandleTypeDef* throttle_hadc;
  uint32_t ADC_value;
  
  /* Tracking */
  volatile TRAJECTORY traj;
  volatile CAR_STATE car;
  volatile TURNING_CONFIG YawControl;
  
  /* Actuator commands*/
  float cmd_steering;
  float cmd_longitude_vel;
  float cmd_motor_current;
} AUTOPILOT_CONFIG;


void autopilot_init(volatile AUTOPILOT_CONFIG* pilot_t, ADC_HandleTypeDef* adc_t);
void autopilot_set_mode(volatile AUTOPILOT_CONFIG* pilot_t, PILOT_MODE mode);
  
/* Continual read messages from driver, navigator 
 * and other sensors. Just put this function in main 
 * loop, then it can update the velocity information 
 * while any free time available. */
void autopilot_sensor_update(volatile AUTOPILOT_CONFIG* pilot_t);

/* Perform action according to different mode. Put 
 * this function in ISR. */
void autopilot_kernel_update(volatile AUTOPILOT_CONFIG* pilot_t);

/* Perform message sending in main loop. */
void autopilot_commu_update(volatile AUTOPILOT_CONFIG* pilot_t,
    COMMU_CONFIG* commu_drive);
#endif 
