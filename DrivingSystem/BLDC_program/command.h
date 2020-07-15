#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>

#define COMMUNICATE_SERAIL
#define STOP_CHAR ';'
#define BUFFER_LENGTH 20

typedef struct{
    bool available;
    uint16_t start_index;
    uint16_t end_index;
    char buffer[BUFFER_LENGTH];
    char mode;
    uint16_t data;
} COMMAND;

void command_init(COMMAND* cmd);
bool command_available(COMMAND* cmd);
void command_read(COMMAND* cmd);
uint16_t next_index(uint16_t index);
void command_debug(COMMAND* cmd);

#endif