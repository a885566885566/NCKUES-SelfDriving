#include "autopilot.h"
#include <string.h>
#include <math.h>

void autopilot_init(volatile AUTOPILOT_CONFIG* pilot_t, ADC_HandleTypeDef* const adc_t){
  /* Autopilot module init */
  pilot_t->pilot_mode = PILOT_RESET;
  pilot_t->driver_last_update_time = HAL_GetTick();
  pilot_t->navigator_last_update_time = HAL_GetTick();
  pilot_t->throttle_hadc = adc_t;
  HAL_ADC_Start_IT(adc_t); // Start ADC1 under Interrupt
  
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
  autopilot_set_mode(pilot_t, PILOT_RESET);
}

void autopilot_set_mode(volatile AUTOPILOT_CONFIG* const pilot_t, const PILOT_MODE mode){
  if (mode == PILOT_RESET){
    turning_set_mode(&(pilot_t->YawControl), FREE);
    pilot_t->cmd_longitude_vel = 0;
    pilot_t->cmd_motor_current = 0;
  }
  else if (mode == SYSTEM_UP){
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

void autopilot_sensor_update(volatile AUTOPILOT_CONFIG* const pilot_t){
  /* CAN BUS */
  /*
  if ( poll_for_msg(commu_driver, &(pilot_t->driver_data))>0 ){
    // Get new msg from driver
    switch(pilot_t->driver_data.mode){
      // Receive velocity infomation from driver
      case CONF_COMMU_MODE_VELOCITY:
        pilot_t->car.velocity_rear = pilot_t->driver_data.data.fvalue;
        break;
    }
    //pilot_t->driver_last_update_time = HAL_GetTick();
  }*/
  
  /* STEERING */
  // From degree to radian
  pilot_t->car.angle_steering = pilot_t->YawControl.shaftAngle * MATH_PI / 180.0;
  
  /* THROTTLE */
  // Did in interrupt
  //HAL_ADC_PollForConversion(pilot_t->throttle_adc, 0xffff);
}

void autopilot_kernel_update(volatile AUTOPILOT_CONFIG* const pilot_t){
  // Driver not responding, switch to manual mode forcely
  static uint32_t now_tick = 0;
  now_tick = HAL_GetTick();
  pilot_t->driver_update_interval = now_tick - pilot_t->driver_last_update_time;
  pilot_t->navigator_update_interval = now_tick - pilot_t->navigator_last_update_time;
  if (pilot_t->pilot_mode != RESET && pilot_t->pilot_mode != SYSTEM_UP ){
    if ( pilot_t->driver_update_interval > CONF_DRIVER_COMMU_SAFE_TIME ||
         pilot_t->navigator_update_interval > CONF_NAVIGATOR_COMMU_SAFE_TIME){
    //if ( pilot_t->driver_update_interval > CONF_DRIVER_COMMU_SAFE_TIME ){
      autopilot_set_mode(pilot_t, PILOT_MANUAL);
    }
    
    if (pilot_t->pilot_mode == PILOT_MANUAL){
      pilot_t->cmd_motor_current = pilot_t->car.throttle;
    }
    else if (pilot_t->pilot_mode == PILOT_AUTO){
      /* Longitude tracking */
      pilot_t->cmd_longitude_vel = 00000000;
      
      /* Lateral tracking */
      //PIDsetTarget(&(pilot_t->YawControl.PID_turning), pilot_t->cmd_steering);
    }
    // Set communication flag
    pilot_t->commu_driver_flag = PENDING;
  }
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
    send_msg(commu_drive, CENTRAL_CONTROLLER_ID, &(pilot_t->driver_cmd));
    pilot_t->commu_driver_flag = EMPTY;
  }
  if (pilot_t->commu_navigator_flag == PENDING){
    // Send message to navigator
  }
}

extern uint32_t sid;
uint8_t update_navigator_msg(volatile COMMU_CONFIG* commu_t, volatile TRAJECTORY *traj){
  static COMMU_DATA_DOUBLE raw;
  if (HAL_CAN_GetRxFifoFillLevel(commu_t->hcan, CAN_NAVIGATOR_FIFO) > 0){
    HAL_CAN_GetRxMessage(commu_t->hcan, CAN_NAVIGATOR_FIFO, &(commu_t->RxMeg), raw.array);
    sid = commu_t->RxMeg.StdId;
    switch(commu_t->RxMeg.StdId){
      case (NAVIGATOR_ID << NAVIGATOR_ID_OFFSET) + NAVIGATOR_ID_A:
        traj->a = raw.fvalue;
        break;
      case (NAVIGATOR_ID << NAVIGATOR_ID_OFFSET) + NAVIGATOR_ID_B:
        traj->b = raw.fvalue;
        break;
      case (NAVIGATOR_ID << NAVIGATOR_ID_OFFSET) + NAVIGATOR_ID_C:
        traj->vs = raw.fvalue;
        break;
      case (NAVIGATOR_ID << NAVIGATOR_ID_OFFSET) + NAVIGATOR_ID_D:
        traj->ve = raw.fvalue;
        break;
    }
    return commu_t->RxMeg.DLC;
  }
  return 0;
}

/*
 * Simulation car motion with kinectic car model.
 *              LR * tan(delta)               v
 * beta = atan(-----------------),  vf = ------------
 *                    LEN                 cos(delta)
 *                     
 * local_p = vf * dt * [cos(beta), sin(beta)].T
 *                 | cos(yaw) -sin(yaw) |
 * p(t+1) = p(t) + |                    |
 *                 | sin(yaw)  cos(yaw) |
 *                      v * dt * tan(delta)
 * yaw(t+1) = yaw(t) + ---------------------
 *                            LEN
 */
void autopilot_car_model_predictor(volatile AUTOPILOT_CONFIG* pilot_t){
  static double vf=0;
  // C.G. velocity angle
  (pilot_t->car).beta = atan(CONF_CAR_LR*tan((pilot_t->car).angle_steering)/CONF_CAR_LEN);
  // C.G. velocity 
  vf = (pilot_t->car).velocity_rear / cos((pilot_t->car).angle_steering);
  // Update car position
  (pilot_t->car).x += vf*CONF_CAR_MODEL_UPDATE_DT*(
      cos((pilot_t->car).yaw) * cos((pilot_t->car).beta) 
    - sin((pilot_t->car).yaw) * sin((pilot_t->car).beta)  );
  
  (pilot_t->car).y += vf*CONF_CAR_MODEL_UPDATE_DT*(
      sin((pilot_t->car).yaw) * cos((pilot_t->car).beta) 
    + cos((pilot_t->car).yaw) * sin((pilot_t->car).beta)  );
  
  // Update yaw
  (pilot_t->car).yaw += (pilot_t->car).velocity_rear * CONF_CAR_MODEL_UPDATE_DT* tan((pilot_t->car).angle_steering) / CONF_CAR_LEN;
}

void autopilot_car_model_reset(volatile AUTOPILOT_CONFIG* pilot_t){
  pilot_t->car.x = 0;
  pilot_t->car.y = 0;
  pilot_t->car.yaw = 0;
}
