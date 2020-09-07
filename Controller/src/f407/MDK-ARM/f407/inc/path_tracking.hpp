#ifndef PATH_TRACKING_H
#define PATH_TRACKING_H

#include "math.h"

/* Real time physical car states */
typedef struct{
  float x;
  float y;
  float yaw;
  float velocity_rear;
  float angle_steering;  // degree
  float beta;            // angle of center velocity
} CAR_STATE;

/* Reference trajectory */
typedef struct{
  float a;
  float b;
  float vs;
  float ve;
} TRAJECTORY;

class PathTrack{
  private:
    float calc_slope(float x);
    float calc_y(float x);
    float calc_offset(float slope, float x, float y);
    void solve_intersect(float s1, float b1, float s2, float b2, float* xi, float* yi);
    CAR_STATE origin;
  public:
    PathTrack();
    TRAJECTORY traj;
    CAR_STATE car;
    void set_path(float a, float b, float vs, float ve);
    float stanley_control(float k);
    float longitude_control();
  
    void model();
    void set_origin(float x, float y, float yaw, float v);
    void update_local(float x, float y, float yaw, float v);
};
#endif
