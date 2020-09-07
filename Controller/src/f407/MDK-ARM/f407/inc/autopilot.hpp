#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "configs.h"
#include "communication.hpp"
#include "subsystem.hpp"

/* Subsystems */
#include "turning_module.hpp"
#include "engine_module.hpp"

#include "path_tracking.hpp"

class AutopilotSystem:public Subsystem{
private:
  NAVIGATOR_INFO info;

  PathTrack path_track;
  NavigatorCommunicator navigator;

  /* Sensors */
  ADC_HandleTypeDef* throttle_hadc;
  float throttle;

  /* State check */
  bool subsystem_state_check(SYSTEM_STATE s);
  bool subsystem_ready_check();

public:
  /* Subsystem */
  EngineSubsystem engine_sys;
  TurningSubsystem steering_sys;

  CAR_STATE car;
  AutopilotSystem(ADC_HandleTypeDef* adc_t, 
    CAN_HandleTypeDef* hcan_driver, 
    CAN_HandleTypeDef* hcan_navigator);
  void reset();
  void update();
  void prepare();
  void throttle_update(uint32_t adc_v);
  void updateMsg();
  void updateSubsystem();
};

#endif 
