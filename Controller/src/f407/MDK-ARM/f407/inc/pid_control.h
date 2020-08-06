#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>
typedef struct{
  /* Tuning Parameter */
  double kp;
  double ki;
  double kd;
  double decayRatio;
  
  /* Computing Parameter */
  uint32_t dt;
  uint32_t upperBound;
  uint32_t lowerBound;
  
  /* Temperory data */
  double error;
  double pre_error;
  double intergral;
  double derivative;
  double cmd;
  double output;
  
} PID_STRUCT;


double inline boundLimit(double input, const double upper, const double lower);
uint32_t inline boundLimitInt(uint32_t input, const uint32_t upper, const uint32_t lower);
void PIDsetTarget(volatile PID_STRUCT *pid_t, double target);
void PID_init(volatile PID_STRUCT *pid_t, double KP, double KI, double KD, double DT, double upper, double lower, double Decay);
double PID_update(volatile PID_STRUCT *pid_t, double current);
#endif
