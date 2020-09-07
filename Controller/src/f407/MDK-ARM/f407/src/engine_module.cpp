#include "engine_module.hpp"

EngineSubsystem::EngineSubsystem(CAN_HandleTypeDef* hcan_t):
driver(hcan_t, CONF_DRIVER_COMMU_SAFE_TIME){
  reset();
}

void EngineSubsystem::reset(){
  set_command(0);
  driver.available = false;
  state_req = STATE_READY;
  set_state(STATE_READY);
}

void EngineSubsystem::prepare(){
  
}

/* Used to receive message from can bus, this function should
 * be called in corresponding can bus rx interrupt. */
void EngineSubsystem::updateMsg(){
  // New series of info obtained
  if (driver.get_message(&info)){
    if (info.motor_state == MOTOR_RESET){
      set_state(STATE_ERROR);
      reset();
    }
  }
}

/* Set different command according to current state */
void EngineSubsystem::set_command(float cmd){
  if( state == STATE_MANUAL ){
      velocity_cmd.fvalue = 0;
      current_cmd.fvalue = cmd;
  }
  else if( state == STATE_AUTO ){
      velocity_cmd.fvalue = cmd;
      current_cmd.fvalue = 0;
  }
  else{
      velocity_cmd.fvalue = 0;
      current_cmd.fvalue = 0;
  }
}

void EngineSubsystem::update(){
  // If lost the communication, set back to manual mode forcely.
  if ((state >= STATE_READY && driver.check_timeout())
   || (state >= STATE_READY && info.motor_state == MOTOR_RESET))
    set_state(STATE_ERROR);
  
  switch(state){
    case STATE_RESET:
    case STATE_SYSTEM_UP:
      set_command(0);
      // If motor state ok, switch to next state
      if (!driver.check_timeout()
        && info.motor_state == MOTOR_STANBY)
        set_state(static_cast<SYSTEM_STATE>(state + 1));
      break;
    case STATE_ERROR:
      set_command(0);
      // From ERROR state return to READY state
      if (info.motor_state == MOTOR_STANBY
      && (!driver.check_timeout()))
        set_state(STATE_READY);
      break;
    case STATE_READY:  
      set_command(0);
      // If state is ready, change state according to `state_req`
      if (state_req == STATE_MANUAL)
        set_state(STATE_MANUAL);
      else if (state_req == STATE_AUTO)
        set_state(STATE_AUTO);
      break;
    case STATE_MANUAL:
      driver.send_message(CONF_DRIVER_MODE_C, &current_cmd);
      break;
    case STATE_AUTO:
      driver.send_message(CONF_DRIVER_MODE_V, &velocity_cmd);
      break;
  }
}
