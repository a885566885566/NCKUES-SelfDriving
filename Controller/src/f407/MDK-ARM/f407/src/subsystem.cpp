#include "subsystem.hpp"

Subsystem::Subsystem(){
  state = STATE_RESET;
  state_req = STATE_RESET;
}

Subsystem::~Subsystem(){
}

bool Subsystem::set_state(SYSTEM_STATE req){
  if (state == req) return false;
  if ( req < state 
     || (state + 1 == req )
     || (state >= STATE_READY && req == STATE_ERROR)){
    reset();
    state = req;
    return true;
  }
  return false;
}
