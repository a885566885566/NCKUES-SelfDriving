#include "command.h"

#include "Arduino.h"

void command_init(COMMAND* cmd){
    #ifdef COMMUNICATE_SERAIL
    Serial.begin(9600);
    #endif
    cmd->start_index = 0;
    cmd->end_index = 0;
}

/* Detect if */
bool command_available(COMMAND* cmd){
    #ifdef COMMUNICATE_SERAIL
    if(Serial.available()){
        char input = Serial.read();
        if (input == '\n' || input == '\r') return false;
        cmd->buffer[cmd->end_index] = input;
        cmd->end_index = next_index(cmd->end_index);
        if(input == STOP_CHAR) return true;
    }
    return false;
    #endif
}

void command_read(COMMAND* cmd){
    #ifdef COMMUNICATE_SERAIL
    cmd->mode = cmd->buffer[cmd->start_index];
    cmd->data = 0;
    for (   cmd->start_index=next_index(cmd->start_index);
            cmd->buffer[cmd->start_index]!=STOP_CHAR; 
            cmd->start_index=next_index(cmd->start_index)){
        cmd->data = 10* cmd->data + (cmd->buffer[cmd->start_index]-'0');
    }
    cmd->start_index=next_index(cmd->start_index);
    #endif
}

uint16_t next_index(uint16_t index){
    return (index+1)%BUFFER_LENGTH;
}

void command_debug(COMMAND* cmd){
    for (int i=0; i<BUFFER_LENGTH; i++){
        Serial.print(cmd->buffer[i]);
        Serial.print(", ");
    }
    Serial.print("\nStart= ");
    Serial.println(cmd->start_index);
    Serial.print("End= ");
    Serial.println(cmd->end_index);
}