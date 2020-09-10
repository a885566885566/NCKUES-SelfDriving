#ifndef SOCKET_PI
#define SOCKET_PI

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sl/Camera.hpp>

#define COMMU_PORT 8787
// IP of pi
#define TARGET_HOST "10.1.1.12"

typedef struct{
    int32_t id;
    int32_t label;
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
} MSG_OBSTACLE;

typedef struct{
    double x;
    double y;
    double z;
    double ox;
    double oy;
    double oz;
    double ow;
} MSG_POSE;

int send_pose_msg(sl::Pose pose);

int send_obs_msg(sl::Objects obs);

int send_msg(void* data, int len);

#endif
