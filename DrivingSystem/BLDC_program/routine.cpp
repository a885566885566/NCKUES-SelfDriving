#include "routine.h"
#include "Arduino.h"
extern CONTROL motor;
extern MOTOR_STATE motor_state;

void routine_init(ROUTINE* rou){
    rou->cmd_tran.mode = CONF_MODE_VELOCITY;
    rou->cmd_tran.id = CONF_MOTOR_DRIVER_ID;
    rou->last_report_time = 0;
}

void command_process(ROUTINE* const rou, COMMU* const commu){
    if (commu_available(commu)){
        commu_read(commu, &(rou->cmd_reci));
        Serial.print("Get ");
        Serial.print(rou->cmd_reci.mode);
        Serial.print(", ");
        Serial.println(rou->cmd_reci.data.fvalue);
        //commu_debug(commu, &cmd);
        
        switch (rou->cmd_reci.mode){
        case CONF_MODE_VELOCITY:
            if (rou->cmd_reci.data.fvalue < CONF_VELOCITY_THRESHOLD)
                control_set_mode(&motor, CONTROL_NONE);
            else{
                control_set_mode(&motor, CONTROL_VELOCITY);
                control_set_speed(rou->cmd_reci.data.fvalue);
            }
            break;
        case CONF_MODE_CURRENT:
            if (rou->cmd_reci.data.fvalue < CONF_CURRENT_THRESHOLD)
                control_set_mode(&motor, CONTROL_NONE);
            else{
                control_set_mode(&motor, CONTROL_CURRENT);
                control_set_current(rou->cmd_reci.data.fvalue);
            }
            break;
        case CONF_MODE_STOP:
            control_set_mode(&motor, CONTROL_NONE);
            break;
        default:
            control_set_mode(&motor, CONTROL_NONE);
            break;
        }
    }
}

void motor_info_report(ROUTINE* rou, COMMU* const commu, const MOTOR_STATE* const state){
    static uint32_t now_time = 0;
    now_time = millis();
    if( now_time - rou->last_report_time > CONF_REPORT_INTERVAL){
        rou->cmd_tran.data.fvalue = state->velocity;
        //rou->cmd_tran.data.fvalue = 3.697;
        commu_send(commu, &(rou->cmd_tran));
        rou->last_report_time = now_time;
        /*
        Serial.print("CAN= ");
        Serial.print(rou->cmd_tran.mode);
        Serial.print(',');
        Serial.print(rou->cmd_tran.id);
        Serial.print(',');
        for(int i=0;i <8; i++){
            Serial.print(rou->cmd_tran.data.array[i]);
            Serial.print("-");
        }
        Serial.println(" ");*/
    }
}

void debug_msg(){
    static uint32_t last_time = 0;
    static uint32_t now_time = 0;
    now_time = millis();
    if ( now_time - last_time > CONF_DEBUG_REPORT_INTERVAL){
        Serial.print(motor_state.pwm_cmd);
        Serial.print(',');
        Serial.print(motor_state.velocity);
        Serial.print(',');
        Serial.println(motor_state.current);
        last_time = now_time;
    }
}
