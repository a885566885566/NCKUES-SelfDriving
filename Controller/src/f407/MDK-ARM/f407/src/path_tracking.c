#include "path_tracking.h"
#include "configs.h"

float calc_slope(TRAJECTORY* traj_t, float x){
  return 3*(traj_t->a)*x*x + 2*(traj_t->b)*x;
}

float calc_y(TRAJECTORY* traj_t, float x){
  return (traj_t->a)*x*x*x + (traj_t->b)*x*x;
}

float calc_offset(float slope, float x, float y){
  return y - slope * x;
}

void solve_intersect(float s1, float b1, float s2, float b2, float* xi, float* yi){
  (*xi) = (b2 - b1)/(s1 - s2);
  (*yi) = s1 * (*xi) + b1;
}

float path_stanley(CAR_STATE* car_t, TRAJECTORY* traj_t, float k){
  // Calculate look ahead
  static float look_x, look_y;
  static float s1, b1, y1;
  static float s2, b2;
  static float e, steering, theta;
  s1 = calc_slope(traj_t, car_t->x);
  y1 = calc_y(traj_t, car_t->x);
  b1 = calc_offset(s1, car_t->x, car_t->y);
  
  s2 = tanf(car_t->yaw);
  if (s2 < 0.0001){
    look_x = car_t->x;
    look_y = s1 * car_t->x + b1;
  }
  else{
    s2 = -1 / s2;
    b2 = calc_offset(s2, car_t->x, car_t->y);
    solve_intersect(s1, b1, s2, b2, &look_x, &look_y);
  }
  
  // Stanley control
  e = ((car_t->x)-look_x) * (cosf((car_t->yaw) + 1.5707)) + 
      ((car_t->y)-look_y) * (sinf((car_t->yaw) + 1.5707));
  theta = (car_t->yaw) - atanf(calc_slope(traj_t, look_x));
  steering = -(theta + atanf(k * e / car_t->velocity_rear));
  steering = (steering > CONF_CAR_MAX_STEER)?CONF_CAR_MAX_STEER:
        ((steering < -CONF_CAR_MAX_STEER)?-CONF_CAR_MAX_STEER:steering);
  return steering;
}
