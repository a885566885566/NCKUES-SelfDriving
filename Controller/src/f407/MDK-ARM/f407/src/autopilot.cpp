#include "autopilot.hpp"
#include <string.h>
#include <math.h>

AutopilotSystem::AutopilotSystem(ADC_HandleTypeDef* adc_t, CAN_HandleTypeDef* hcan_driver, CAN_HandleTypeDef* hcan_navigator):
path_track(),
navigator(hcan_navigator, CONF_NAVIGATOR_COMMU_SAFE_TIME),
engine_sys(hcan_driver),
steering_sys(){
  throttle_hadc = adc_t;
  reset();
  HAL_ADC_Start_IT(adc_t);
}

void AutopilotSystem::prepare(){
}

/* This function should be called in throttle ADC callback */
void AutopilotSystem::throttle_update(uint32_t adc_v){
  if (adc_v < CONF_THROTTLE_MIN) 
    throttle = 0;
  else
    throttle = static_cast<float>(adc_v - CONF_THROTTLE_MIN) / (CONF_THROTTLE_MAX - CONF_THROTTLE_MIN);
}

/*
 * 1. Request all subsystem set to READY 
 */
void AutopilotSystem::reset(){
  throttle = 0;
  engine_sys.reset();
  steering_sys.reset();
  if (state >= STATE_READY)
    state_req = STATE_READY;
}

void AutopilotSystem::updateMsg(){
  // New series of info obtained
  if (navigator.get_message(&info)){
    switch(info.navigator_state){
      case NAVIGATOR_RESET:
        break;
      case NAVIGATOR_PATH:
        path_track.set_path(info.a, info.b, info.vs, info.ve);
        break;
      case NAVIGATOR_ORIGIN:
        path_track.set_origin(info.x, info.y, info.yaw, info.v);
        break;
      case NAVIGATOR_UPDATE:
        path_track.update_local(info.x, info.y, info.yaw, info.v);
        break;
    }
  }
}

/* Check if the state of all subsystem are higher than SYSTEM_UP */
bool AutopilotSystem::subsystem_ready_check(){
  return engine_sys.state >= STATE_SYSTEM_UP
      && steering_sys.state >= STATE_SYSTEM_UP
      && !navigator.check_timeout();
}

/* Check if the state of all subsystem is the same as the specified one */
bool AutopilotSystem::subsystem_state_check(SYSTEM_STATE s){
  return engine_sys.state == s
      && steering_sys.state == s;
}

void AutopilotSystem::update(){
  // If it lost the communication, set back to manual mode forcely.
  if (state >= STATE_READY && navigator.check_timeout())
    set_state(STATE_ERROR);
  
  switch(state){
    case STATE_ERROR:
      reset();  // Request to set all the other subsystem to READY
      if ( subsystem_state_check(STATE_READY) )
        set_state(STATE_READY);
      break;
      
    case STATE_RESET:
    case STATE_SYSTEM_UP:
      if ( subsystem_ready_check() )
        set_state(STATE_READY);
      break;
      
    case STATE_READY:
      // Switch to requested state, if all subsystem swtich successfully
      if (state_req == STATE_MANUAL){
        engine_sys.state_req = STATE_MANUAL;
        steering_sys.state_req = STATE_MANUAL;
        if ( subsystem_state_check(STATE_MANUAL) )
          set_state(STATE_MANUAL);
      }
      else if (state_req == STATE_AUTO){
        engine_sys.state_req = STATE_AUTO;
        steering_sys.state_req = STATE_AUTO;
        if ( subsystem_state_check(STATE_AUTO) )
          set_state(STATE_AUTO);
      }
      break;
      
    case STATE_MANUAL:
      if ( !subsystem_state_check(STATE_MANUAL) )
        set_state(STATE_ERROR);
      engine_sys.set_command(throttle);
      break;
      
    case STATE_AUTO:
      if ( !subsystem_state_check(STATE_AUTO) )
        set_state(STATE_ERROR);
      float steering = path_track.stanley_control(0.5f);
      float vel_cmd = path_track.longitude_control();
      steering_sys.set_shaft(static_cast<double>(steering));
      engine_sys.set_command(vel_cmd);
      break;
  }
  path_track.model();
}

void AutopilotSystem::updateSubsystem(){
  engine_sys.update();
  steering_sys.update();
}

