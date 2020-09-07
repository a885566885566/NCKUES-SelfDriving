#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>

class PI{
  private:
    /* Tuning Parameter */
    double kp;
    double ki;
    
    double intergral;
    double intergral_windup;
  
  protected:
    /* Temperory data */
    double cmd;
    double output;
    double error;
    double pre_error;
  
    /* Computing Parameter */
    uint8_t dt;           // update frequency unit in ms
    int16_t upperBound;  // upper limit of output
    int16_t lowerBound;  // lower limit of output
    
  public:
    PI(double kp_t, double ki_t, uint8_t dt_t, 
       int16_t upper, int16_t lower);
    void set_target(double target);
    void reset();
    double update(double current);
};

class PID: public PI{
  private:
    /* Tuning Parameter */
    double kd;
    double decayRatio;
    
    /* Temperory data */
    double derivative;
  
  public:
    PID(double kp_t, double ki_t, double kd_t, uint8_t dt_t, 
        double decay_t, int16_t upper, int16_t lower);
    void reset();
    double update(double current);
};

#endif
