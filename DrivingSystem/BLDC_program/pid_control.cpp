#include "pid_control.h"

double inline boundLimit(double input, const double upper, const double lower){
  return ( input > upper ) ? upper : (( input < lower ) ? lower : input);
}

int16_t inline boundLimitInt(int16_t input, const int16_t upper, const int16_t lower){
  return ( input > upper ) ? upper : (( input < lower ) ? lower : input);
}

void PIDsetTarget(volatile PID_STRUCT *pid_t, double target){
  pid_t->cmd = target;
}

void PID_init(volatile PID_STRUCT *pid_t, double KP, double KI, double KD, double DT, double upper, double lower, double Decay){
  pid_t->kp = KP;
  pid_t->ki = KI;
  pid_t->kd = KD;
  pid_t->dt = DT;             // In millisecond
  pid_t->decayRatio = Decay;  // 0~1
  pid_t->upperBound = upper;
  pid_t->lowerBound = lower;
  pid_t->intergral  = 0;
  pid_t->derivative = 0;
  pid_t->cmd = 0;
}

double PID_update(volatile PID_STRUCT *pid_t, double current){
  // Calculate Error gain
  static double G_out = 0;
  static double I_out = 0;
  static double D_out = 0;

  pid_t->error = pid_t->cmd - current;
  G_out = pid_t->kp * pid_t->error;
  
  // Calculate Intergral
  pid_t->intergral = pid_t->intergral + pid_t->error*pid_t->dt/1000.0;
  pid_t->intergral = constrain(pid_t->intergral, pid_t->lowerBound, pid_t->upperBound);
  I_out = pid_t->ki * pid_t->intergral;
  
  #ifdef CONF_PID_D_ENABLE
  // Calculate Differential
  pid_t->derivative = 1000 * (pid_t->error - pid_t->pre_error) / pid_t->dt;
  D_out = pid_t->kd * pid_t->derivative;
  pid_t->derivative *= pid_t->decayRatio;
  #endif

  // Calculate output
  pid_t->output = constrain( G_out + I_out + D_out, pid_t->lowerBound, pid_t->upperBound);
  //pid_t->output = boundLimit( G_out + I_out + D_out, pid_t->upperBound, pid_t->lowerBound);//, pid_t->upperBound, pid_t->lowerBound);
  
  return pid_t->output;
}

void PID_reset(volatile PID_STRUCT *pid_t){
  pid_t->error = 0;
  pid_t->pre_error = 0;
  pid_t->intergral = 0;
  pid_t->derivative = 0;
  pid_t->output = 0;
}