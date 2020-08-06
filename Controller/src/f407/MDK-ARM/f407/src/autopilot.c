#include "autopilot.h"

void autopilot_init(volatile AUTOPILOT_CONFIG* pilot_t, ADC_HandleTypeDef* const adc_t){
  /* Autopilot module init */
  pilot_t->pilot_mode = PILOT_RESET;
  pilot_t->driver_last_update_time = HAL_GetTick();
  pilot_t->navigator_last_update_time = HAL_GetTick();
  pilot_t->throttle_adc = adc_t;
  
  /* Turning module init */
  // turning init including wheel sensor correction,
  // so this function should perform in the last.
  turning_init(&(pilot_t->YawControl), 
    CONF_PIN_SHAFT_Z_PORT, 
    CONF_PIN_SHAFT_Z_MASK, 
    CONF_PIN_CLUTCH_PORT, 
    CONF_PIN_CLUTCH_MASK, 
    &(TIM2->CNT), 
    &(TIM4->CNT), 
    &(TIM3->CCR1));
}

void autopilot_set_mode(volatile AUTOPILOT_CONFIG* const pilot_t, const PILOT_MODE mode){
  if (mode == PILOT_RESET){
    turning_set_mode(&(pilot_t->YawControl), FREE);
    pilot_t->cmd_longitude_vel = 0;
    pilot_t->cmd_motor_current = 0;
  }
  else if (mode == PILOT_MANUAL){
    turning_set_mode(&(pilot_t->YawControl), FREE);
    pilot_t->cmd_longitude_vel = 0;
  }
  else if (mode == PILOT_AUTO){
    turning_set_mode(&(pilot_t->YawControl), POSITION_CONTROL);
    pilot_t->cmd_motor_current = 0;
  }
  pilot_t->pilot_mode = mode;
}

void autopilot_sensor_update(volatile AUTOPILOT_CONFIG* const pilot_t, COMMU_CONFIG * const commu_driver){
  /* CAN BUS */
  if ( poll_for_msg(commu_driver, &(pilot_t->driver_data))>0 ){
    // Get new msg from driver
    switch(pilot_t->driver_data.mode){
      // Receive velocity infomation from driver
      case CONF_COMMU_MODE_VELOCITY:
        pilot_t->car.velocity_rear = pilot_t->driver_data.data.fvalue;
        pilot_t->driver_last_update_time = HAL_GetTick();
        break;
    }
  }
  
  /* STEERING */
  pilot_t->car.angle_steering = pilot_t->YawControl.shaftAngle;
  
  /* THROTTLE */
  HAL_ADC_PollForConversion(pilot_t->throttle_adc, 0xffff);
  static float thr = 0;
  thr = 100 * (thr - CONF_THROTTLE_MIN) / (CONF_THROTTLE_MAX - CONF_THROTTLE_MIN);
  thr = (thr<0)? 0: thr;
  pilot_t->car.throttle = thr;
}

void autopilot_kernel_update(volatile AUTOPILOT_CONFIG* const pilot_t){
  // Driver not responding, switch to manual mode forcely
  static uint32_t now_tick = 0;
  now_tick = HAL_GetTick();
  if (now_tick - pilot_t->driver_last_update_time > CONF_DRIVER_COMMU_SAFE_TIME ||
      now_tick - pilot_t->navigator_last_update_time > CONF_NAVIGATOR_COMMU_SAFE_TIME){
    autopilot_set_mode(pilot_t, PILOT_MANUAL);
  }
  
  if (pilot_t->pilot_mode == PILOT_MANUAL){
    pilot_t->cmd_motor_current = pilot_t->car.throttle;
  }
  else if (pilot_t->pilot_mode == PILOT_AUTO){
    /* Longitude tracking */
    pilot_t->cmd_longitude_vel = 00000000;
    
    /* Lateral tracking */
    PIDsetTarget(&(pilot_t->YawControl.PID_turning), pilot_t->cmd_steering);
  }
  // Set communication flag
  pilot_t->commu_driver_flag = PENDING;
}

void autopilot_commu_update(volatile AUTOPILOT_CONFIG* const pilot_t,
    COMMU_CONFIG* const commu_drive){
  if (pilot_t->commu_driver_flag == PENDING){
    // Send message to driver
    if (pilot_t->pilot_mode == PILOT_MANUAL){
      pilot_t->driver_cmd.mode = CONF_COMMU_MODE_CURRENT;
      pilot_t->driver_cmd.data.fvalue = pilot_t->cmd_motor_current;
    }
    else if (pilot_t->pilot_mode == PILOT_AUTO){
      pilot_t->driver_cmd.mode = CONF_COMMU_MODE_VELOCITY;
      pilot_t->driver_cmd.data.fvalue = pilot_t->cmd_longitude_vel;
    }
    else{ // Other situation
      pilot_t->driver_cmd.mode = CONF_COMMU_MODE_CURRENT;
      pilot_t->driver_cmd.data.fvalue = 0;
    }
    send_msg(commu_drive, MOTOR_DRIVER_ID, &(pilot_t->driver_cmd));
    pilot_t->commu_driver_flag = EMPTY;
  }
  if (pilot_t->commu_navigator_flag == PENDING){
    // Send message to navigator
  }
}

