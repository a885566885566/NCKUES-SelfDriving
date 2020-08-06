#ifndef ROUTINE_H
#define ROUTINE_H

#include "definitions.h"
#include "communicate.h"
#include "control.h"

typedef struct{
    COMMAND cmd_reci;
    COMMAND cmd_tran;
    uint32_t last_report_time;
}ROUTINE;

void routine_init(ROUTINE* rou);
void command_process(ROUTINE* const rou, COMMU* const commu);
void motor_info_report(ROUTINE* rou, COMMU* const commu, const MOTOR_STATE* const state);
void debug_msg();
#endif
