#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <Arduino.h>

#include <SPI.h>
#include <mcp2515.h>
#include "definitions.h"

//#define COMMUNICATE_SERAIL
#define COMMUNICATE_CANBUS
#define STOP_CHAR ';'
#define BUFFER_LENGTH 10
#define CAN_DATA_LEN 4      // Not include 'mode' byte

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

typedef union{
    byte array[CAN_DATA_LEN];
    double fvalue;  // 4 bytes
    long ivalue;    // 4 bytes
} COMMU_DATA;

typedef struct{
    COMMU_DATA velocity;
    COMMU_DATA current;
    COMMU_DATA state;
    uint8_t mode;
} COMMAND;

void commu_init(COMMU* const commu);
void commu_read(COMMU* const commu, COMMAND* const cmd);
void commu_send(COMMU* const commu, uint8_t id, COMMU_DATA* const data);
bool commu_available(COMMU* const commu);
void commu_debug(COMMU* const commu);

#endif
