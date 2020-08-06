#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <Arduino.h>

#include <SPI.h>
#include <mcp2515.h>
#include "definitions.h"

#define COMMUNICATE_SERAIL
#define COMMUNICATE_CANBUS
#define STOP_CHAR ';'
#define BUFFER_LENGTH 10
#define CAN_DATA_LEN 7      // Not include 'mode' byte

#define next_index(a) ((a) + 1)%BUFFER_LENGTH

typedef struct{
    #ifdef COMMUNICATE_CANBUS
    struct can_frame can_msg_reci;
    struct can_frame can_msg_tran;
    #endif

    #ifdef COMMUNICATE_SERAIL
    uint16_t start_index;
    uint16_t end_index;
    char buffer[BUFFER_LENGTH];
    #endif
} COMMU;

typedef struct{
    char mode;
    uint16_t id;
    union{
        byte array[CAN_DATA_LEN];
        double fvalue;  // 4 bytes
        long ivalue;    // 4 bytes
    } data;
} COMMAND;

void commu_init(COMMU* const commu);
void commu_read(COMMU* const commu, COMMAND* const cmd);
void commu_send(COMMU* const commu, COMMAND* const cmd);
bool commu_available(COMMU* const commu);
void commu_debug(COMMU* const commu);

#endif
