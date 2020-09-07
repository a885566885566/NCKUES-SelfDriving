#include "communicate.h"

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>
#include "definitions.h"

MCP2515 mcp2515(CONF_PIN_CS);

void commu_init(COMMU* const commu){
    // Setup Serial 
    #ifdef COMMUNICATE_SERAIL
    Serial.begin(9600);
    commu->start_index = 0;
    commu->end_index = 0;
    #endif
    // Setup CAN BUS
    #ifdef COMMUNICATE_CANBUS
    mcp2515.reset();
    mcp2515.setFilter(MCP2515::RXF0, false, CONF_CENTRAL_CONTROLLER_ID);
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    #endif
}

/* Detect if there is message coming. */
bool commu_available(COMMU* const commu){
    #ifdef COMMUNICATE_SERAIL
    if(Serial.available()){
        char input = Serial.read();
        if (input == '\n' || input == '\r') return false;
        commu->buffer[commu->end_index] = input;
        commu->end_index = next_index(commu->end_index);
        if(input == STOP_CHAR) return true;
    }
    #endif  

    #ifdef COMMUNICATE_CANBUS
    if (mcp2515.readMessage(&(commu->can_msg_reci)) == MCP2515::ERROR_OK){
        return true;
    }
    #endif

    return false;
}

void commu_read(COMMU* const commu, COMMAND* const cmd){
    #ifdef COMMUNICATE_SERAIL
    cmd->mode = commu->buffer[commu->start_index];
    cmd->data.ivalue = 0;
    for (   commu->start_index=next_index(commu->start_index);
            commu->buffer[commu->start_index]!=STOP_CHAR; 
            commu->start_index=next_index(commu->start_index)){
        cmd->data.ivalue = 10* cmd->data.ivalue + (commu->buffer[commu->start_index]-'0');
    }
    commu->start_index=next_index(commu->start_index);
    #endif

    #ifdef COMMUNICATE_CANBUS
    uint8_t filter = commu->can_msg_reci.can_id % CONF_ID_OFFSET;
    uint8_t mask = commu->can_msg_reci.can_id >> CONF_ID_OFFSET;
    if (mask == CONF_MOTOR_DRIVER_ID){
        switch(filter){
            case CONF_MODE_CURRENT:
                cmd->mode = CONF_MODE_CURRENT;
                for (int i=0; i<commu->can_msg_reci.can_dlc; i++)
                    cmd->current.array[i] = commu->can_msg_reci.data[i];
                break;
            case CONF_MODE_VELOCITY:
                cmd->mode = CONF_MODE_VELOCITY;
                for (int i=0; i<commu->can_msg_reci.can_dlc; i++)
                    cmd->velocity.array[i] = commu->can_msg_reci.data[i];
                break;
            case CONF_MODE_STOP:
                cmd->mode = CONF_MODE_STOP;
                for (int i=0; i<commu->can_msg_reci.can_dlc; i++)
                    cmd->state.array[i] = commu->can_msg_reci.data[i];
                break;
            case CONF_TERMINATE_ID:
                break;
        }
    }
    #endif
}

#ifdef COMMUNICATE_CANBUS
void commu_send(COMMU* const commu, uint8_t id, COMMU_DATA* const data){
    #ifdef COMMUNICATE_CANBUS
    commu->can_msg_tran.can_id = (CONF_MOTOR_DRIVER_ID<<CONF_ID_OFFSET) + id;
    commu->can_msg_tran.can_dlc = CAN_DATA_LEN;
    
    for (int i=0; i<CAN_DATA_LEN; i++)
        commu->can_msg_tran.data[i] = data->array[i];
    mcp2515.sendMessage(&(commu->can_msg_tran));
    #endif
}
#endif

void commu_debug(COMMU* const commu, COMMAND *cmd){
    #ifdef COMMUNICATE_SERAIL
    /*
    for (int i=0; i<BUFFER_LENGTH; i++){
        Serial.print(commu->buffer[i]);
        Serial.print(", ");
    }
    Serial.print("\nStart= ");
    Serial.println(commu->start_index);
    Serial.print("End= ");
    Serial.println(commu->end_index);
    */
    Serial.print("Mode=");
    Serial.print(cmd->mode);
    Serial.print(", Data=");
    Serial.println(cmd->data.fvalue);
    #endif
}
