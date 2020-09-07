#ifndef TURNING_CONTROL_H
#define TURNING_CONTROL_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "configs.h"
#include "motor_control.hpp"
#include "pid_control.hpp"
#include "utils.hpp"
#include "subsystem.hpp"

#define CLUTCH_OFF false
#define CLUTCH_ON true

/* This a subsystem majors to control front wheel shaft angle.
 * It inherits the state management features from `Subsystem`,
 * which providee a system way to deal with state change.
 * 
 * 1. Use `set_state()` to change state, it will automatically
 *    reset all control parameters if needed and switch specific 
 *    task in `update()`. 
 * 2. `prepare()` run intialization procedure for shaft encoder, 
 *    which will forcely switch state to `SYSTEM_UP`.
 * 3. `update()` should be called in ISR.
 */

class TurningSubsystem:public Subsystem{
  private:
    double shaftAngle;
    // Shaft resolution
    static const uint16_t resolution = CONF_SHAFT_RESOLUTION;
  
    // Encoder pin setting
    GPIO_TypeDef* zeroPort;
    uint16_t zeroMask;
    volatile uint32_t* shaftEncoder;
    // Clutch pin setting
    GPIO_TypeDef* clutchPort;
    uint16_t clutchMask;
  
    Motor motor;
    PI shaftPI;
  
    void set_clutch(bool set);
  public:
    TurningSubsystem();
      
    void reset();
    // This method only valid in AUTO state
    void set_shaft(double angle);
    void prepare(Buzzer* buz);
    void update();
};
#endif

