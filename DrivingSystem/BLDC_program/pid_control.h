#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "utils.h"

typedef struct{
  /* Tuning Parameter */
  double kp;
  double ki;
  double kd;
  double decayRatio;
  
  /* Computing Parameter */
  double dt;
  int16_t upperBound;
  int16_t lowerBound;
  
  /* Temperory data */
  double error;
  double pre_error;
  double intergral;
  double derivative;
  double cmd;
  double output;
  
} PID_STRUCT;


double inline boundLimit(double input, const double upper, const double lower);
int16_t inline boundLimitInt(int16_t input, const int16_t upper, const int16_t lower);
void PIDsetTarget(volatile PID_STRUCT *pid_t, double target);
void PID_init(volatile PID_STRUCT *pid_t, double KP, double KI, double KD, double DT, double upper, double lower, double Decay);
double PID_update(volatile PID_STRUCT *pid_t, double current);
void PID_reset(volatile PID_STRUCT *pid_t);
#endif
