#include "command.h"
#include "utils.h"
#include "control.h"

COMMAND cmd;
extern CONTROL motor;
BEEPER_CONFIG buzzer;
void setup(){
    pin_init();
    system_init();
    
    command_init(&cmd);
    control_init(&motor);
    ISR_enable();
    utils_beep_init(&buzzer, PIN_BUZZER);
    utils_beep_set(&buzzer, 1, 100, 3);
}

void loop(){
    //utils_beep_update(&buzzer);
    
    motor_key(true, CONF_MOTOR_CW);
    
    for(float i=0; i<10; i+=0.1){
        //PIDsetTarget( &(motor.pid_current), i );
        PIDsetTarget( &(motor.pid_velocity), i );
        delay(20);
    }
    delay(3000);
    for(float i=10; i>0; i-=0.1){
        //PIDsetTarget( &(motor.pid_current), i );
        PIDsetTarget( &(motor.pid_velocity), i );
        delay(20);
    }
    motor_key(false, CONF_MOTOR_CW);
    delay(1000);
    /*
    if (command_available(&cmd)){
        command_read(&cmd);
        Serial.print("Mode=");
        Serial.print(cmd.mode);
        Serial.print(", Data=");
        Serial.println(cmd.data);
        //command_debug(&cmd);
        switch (cmd.mode){
        case CONF_MODE_MOTOR:
            control_set_speed(cmd.data);
            break;
        
        case CONF_MODE_STOP:
            control_set_speed(0);
            break;
            
        case CONF_MODE_READ:
            break;
        default:
            break;
        }
    }*/
    //delay(100);
}