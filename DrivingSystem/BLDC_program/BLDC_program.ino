#include "communicate.h"
#include "utils.h"
#include "control.h"
#include "routine.h"

COMMU commu;
ROUTINE routine;
extern BEEPER_CONFIG buzzer;
extern MOTOR_STATE motor_state;
extern CONTROL motor;
void setup(){
    // This two init function set pins to safe state, 
    // so they should be perform at fast as possible.
    pin_init();
    system_init();
    
    // Init for can bus module
    commu_init(&commu);
    // Init for motor control parameters
    control_init(&motor);
    // Init for routine(Including report infomation)
    routine_init(&routine);

    // Enable control update ISR 
    ISR_enable();

    // Buzzer setting
    utils_beep_init(&buzzer, PIN_BUZZER);
    utils_beep_set(&buzzer, 1, 35, 2);
}

void loop(){
    // Handle command from communication device
    command_process(&routine, &commu);

    // Send velocity infomation back to Center Controller
    motor_info_report(&routine, &commu, &motor_state);

    utils_beep_update(&buzzer);

    // Send debug message 
    debug_msg();
}

void motor_test(){
    motor_key(true, CONF_MOTOR_CW);
    /*
    for(float i=0; i<0.6; i+=0.01){
        PIDsetTarget( &(motor.pid_current), i );
        delay(20);
    }
    delay(3000);
    for(float i=0.6; i>0; i-=0.01){
        PIDsetTarget( &(motor.pid_current), i );
        delay(20);
    }*/

    for(float i=0; i<25; i+=0.1){
        PIDsetTarget( &(motor.pid_velocity), i );
        delay(2);
    }
    delay(5000);
    for(float i=25; i>0; i-=0.1){
        PIDsetTarget( &(motor.pid_velocity), i );
        delay(2);
    }
    motor_key(false, CONF_MOTOR_CW);
    delay(1000);
}
