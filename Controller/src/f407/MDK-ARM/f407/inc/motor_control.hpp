#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "configs.h"
#include "pid_control.hpp"

/* This class provide basic velocity control ability. 
 * The method is to excute `update()` in ISR, then it
 * will update the velocity PI controller and output 
 * to the target pwm pin and direction pin automatically.
 * 
 * Steps to use this class:
 * 1. Call the constructor, which will setup all pins
 *    required and encoder counter that used.
 * 2. Set the velocity command with `set_speed()`.
 * 3. Put `update()` in an ISR, which has cycle time `dt`.
 * 4. If any problems occur, you can run `stop()` to forcely
 *    stop the output, and reset all PI controller parameters.
 *
 * The public data:
 * -Speed: update on every ISR called, unit according to 
 *    the given `resolution`
 */

class Motor{
  private:
    // Pin setting
    volatile uint32_t* pwmSig;
    GPIO_TypeDef* dirPinPort;
    uint16_t dirPinMask;
    // Encoder counter register address
    volatile uint32_t* motorEncoder;
  
    // Computing parameters
    int32_t encPreCount;
    // Global settings
    double resolution;
    uint8_t dt;
  
    // This constant limited the maxima output pwm
    static const uint16_t pwmMax = CONF_PWM_MAXIMA;
  
    // This function should be called with the period 
    // set in dt, otherwise the result will be wrong.
    double read_speed();
  
  public:
    double speed;
    PI velocityPI;
  
    Motor(volatile uint32_t* const cnt, 
          GPIO_TypeDef* const port, 
          const uint16_t pinMask,
          volatile uint32_t* const motorEnc, 
          double res, 
          uint8_t dt_t);
    
    // Set velocity command, unit according to `resolution`
    void set_speed(double cmd);
          
    void reset();
    
    // Drive the motor with pwm command
    void drive(double cmd);
          
    // Set output to 0
    void stop();
          
    // Call this function in regular period
    void update();
};

#endif
