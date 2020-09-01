#ifndef PATH_TRACKING_H
#define PATH_TRACKING_H

#include "autopilot.h"
#include "math.h"

inline float calc_slope(TRAJECTORY* traj_t, float x);

inline float calc_y(TRAJECTORY* traj_t, float x);

inline float calc_offset(float slope, float x, float y);

void solve_intersect(float s1, float b1, float s2, float b2, float* xi, float* yi);

float stanley_control(CAR_STATE* car_t, TRAJECTORY* traj_t, float k);

#endif
