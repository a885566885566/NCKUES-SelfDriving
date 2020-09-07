#include "pid_control.hpp"

double inline boundLimit(double input, const double upper, const double lower){
  return ( input > upper ) ? upper : (( input < lower ) ? lower : input);
}

/* PI */
PI::PI(double kp_t, double ki_t, uint8_t dt_t, 
       int16_t upper, int16_t lower){
  reset();
  kp = kp_t;
  ki = ki_t;
  dt = dt_t;
  upperBound = upper;  
  lowerBound = lower;      
}

void PI::reset(){
  error = 0;
  pre_error = 0;
  intergral = 0;
  intergral_windup = 0;
  output = 0;
  cmd = 0;
}

void PI::set_target(double target){
  cmd = target;
}

double PI::update(double current){
  // Calculate Error gain
  error = cmd - current;
  
  // Calculate Intergral
  intergral = intergral + error * dt / 1000.0 - intergral_windup;
  output = boundLimit( kp * error + ki * intergral, upperBound, lowerBound);//, pid_t->upperBound, pid_t->lowerBound);
  // Deal with intergral windup
  intergral_windup = (intergral - output) / (dt / 1000);
  return output;
}

/* PID */
PID::PID(double kp_t, double ki_t, double kd_t, uint8_t dt_t, 
         double decay_t, int16_t upper, int16_t lower)
           :PI(kp_t, ki_t, dt_t, upper, lower){
  kd = kd_t;
  decayRatio = decay_t; 
  reset();
}

void PID::reset(){
  PI::reset();
  derivative = 0;
}

double PID::update(double current){
  double out = PI::update(current);
  derivative = (error - pre_error) / (dt / 1000.0);
  derivative *= decayRatio;
  pre_error = error;
  output = boundLimit( out + kd * derivative, upperBound, lowerBound);
  return output;
}

