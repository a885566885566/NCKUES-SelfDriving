#include "path_tracking.hpp"
#include "configs.h"

PathTrack::PathTrack(){
  traj.a = 0;               traj.b = 0;
  traj.vs = 0;              traj.ve = 0;
  
  car.x = 0;                car.y = 0;
  car.yaw = 0;              car.velocity_rear = 0;
  car.angle_steering = 0;   car.beta = 0;
}

float PathTrack::calc_slope(float x){
  return 3*traj.a*x*x + 2*traj.b*x;
}

float PathTrack::calc_y(float x){
  return traj.a*x*x*x + traj.b*x*x;
}

float PathTrack::calc_offset(float slope, float x, float y){
  return y - slope * x;
}

void PathTrack::solve_intersect(float s1, float b1, float s2, float b2, float* xi, float* yi){
  (*xi) = (b2 - b1) / (s1 - s2);
  (*yi) = s1 * (*xi) + b1;
}

void PathTrack::set_path(float a, float b, float vs, float ve){
  traj.a = a;
  traj.b = b;
  traj.vs = vs;
  traj.ve = ve;
}

void PathTrack::set_origin(float x, float y, float yaw, float v){
  origin.x = x;
  origin.y = y;
  origin.yaw = yaw;
}

void PathTrack::update_local(float x, float y, float yaw, float v){
  car.x = x - origin.x;
  car.y = y - origin.y;
  car.yaw = yaw - origin.yaw;
  car.velocity_rear = v;
}

float PathTrack::stanley_control(float k){
  // Calculate look ahead
  static float look_x, look_y;
  static float s1, b1, y1;
  static float s2, b2;
  static float e, steering, theta;
  s1 = calc_slope(car.x);
  y1 = calc_y(car.x);
  b1 = calc_offset(s1, car.x, car.y);
  
  s2 = tanf(car.yaw);
  if (s2 < 0.0001f){
    look_x = car.x;
    look_y = s1 * car.x + b1;
  }
  else{
    s2 = -1 / s2;
    b2 = calc_offset(s2, car.x, car.y);
    solve_intersect(s1, b1, s2, b2, &look_x, &look_y);
  }
  
  // Stanley control
  e = (car.x-look_x) * (cosf(car.yaw + 1.5707f)) + 
      (car.y-look_y) * (sinf(car.yaw + 1.5707f));
  theta = car.yaw - atanf(calc_slope(look_x));
  steering = -(theta + atanf(k * e / car.velocity_rear));
  steering = (steering > CONF_CAR_MAX_STEER) ? CONF_CAR_MAX_STEER:
        ((steering < -CONF_CAR_MAX_STEER) ? -CONF_CAR_MAX_STEER:steering);
  return steering;
}

float PathTrack::longitude_control(){
  return 0.0f;
}

/*
 * Simulation car motion with kinectic car model.
 *              LR * tan(delta)               v
 * beta = atan(-----------------),  vf = ------------
 *                    LEN                 cos(delta)
 *                     
 * local_p = vf * dt * [cos(beta), sin(beta)].T
 *                 | cos(yaw) -sin(yaw) |
 * p(t+1) = p(t) + |                    |
 *                 | sin(yaw)  cos(yaw) |
 *                      v * dt * tan(delta)
 * yaw(t+1) = yaw(t) + ---------------------
 *                            LEN
 */
void PathTrack::model(){
  static float vf;
  // C.G. velocity angle
  car.beta = atan(CONF_CAR_LR*tan(car.angle_steering)/CONF_CAR_LEN);
  // C.G. velocity 
  vf = car.velocity_rear / cos(car.angle_steering);
  // Update car position
  car.x += vf*CONF_CAR_MODEL_UPDATE_DT*(
      cos(car.yaw) * cos(car.beta) 
    - sin(car.yaw) * sin(car.beta)  );
  
  car.y += vf*CONF_CAR_MODEL_UPDATE_DT*(
      sin(car.yaw) * cos(car.beta) 
    + cos(car.yaw) * sin(car.beta)  );
  
  // Update yaw
  car.yaw +=car.velocity_rear * CONF_CAR_MODEL_UPDATE_DT* tan(car.angle_steering) / CONF_CAR_LEN;
}
