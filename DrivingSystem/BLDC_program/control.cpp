#include "control.h"
#include "definitions.h"
#include "utils.h"
#include "pid_control.h"
#include "encoder.h"
#include "Arduino.h"

volatile CONTROL motor;

void control_init(volatile CONTROL *ctrl){
    //double KP, double KI, double KD, double DT, double upper, double lower, double Decay
    //PID_init(&(ctrl->pid_velocity), 3, 0, 0, CONF_ENCODER_TS, CONF_CURRENT_LIM, -1*CONF_CURRENT_LIM, 0.9);
    PID_init(&(ctrl->pid_velocity), 0.2, 0.02, 0, CONF_ENCODER_TS, CONF_PWM_LIM, -1*CONF_PWM_LIM, 0.9);
    PID_init(&(ctrl->pid_current),  8, 0.2, 0, CONF_CONTROL_VELOCITY_DT, CONF_PWM_LIM, -1*CONF_PWM_LIM, 0.9);
    ctrl->current_integrator = 0;
    ctrl->current_limitator = 0;
    encoder_init(&(ctrl->enc));
    ctrl->current_counter = 0;
    ctrl->velocity_counter = 0;
}

void control_set_speed(double velocity){
    PIDsetTarget(&(motor.pid_velocity), velocity);
}

void ISR_enable(){
    TCCR2A = 0;
    TCCR2B = 0; 
    TCCR2B |= (1<<WGM22);               // CTC mode; Clear Timer on Compare
    TCCR2B |= (1<<CS20) | (1<<CS21);    // Prescaler == 64
    TIMSK2 |= (1 << OCIE2A);            // enable CTC for TIMER1_COMPA_vect
    TCNT2 = 0;
    OCR2A = 125;    // Generate 2kHz ISR event
}

void ISR_disable(){
    TCCR2A = 0;
    TCCR2B = 0; 
}

ISR(TIMER2_COMPA_vect){
    encoder_sampling(&(motor.enc));
    
    if (motor.current_counter > CONF_CONTROL_CURRENT_COUNT){
        double now_current = read_current();
        double now_speed = encoder_speed_mt(&(motor.enc));
        motor.current_integrator += PID_update( &(motor.pid_velocity), now_speed );
        motor.current_integrator = constrain(motor.current_integrator, 0, 250);
        //motor.current_limitator = constrain(CONF_CURRENT_LIM_PARA_A * now_current + CONF_CURRENT_LIM_PARA_B, 0, 1);
        //motor.current_integrator *= motor.current_limitator;
        write_motor(motor.current_integrator);
        Serial.print(motor.pid_velocity.cmd);
        Serial.print(',');
        Serial.print(motor.current_limitator);
        Serial.print(',');
        Serial.print(motor.current_integrator/10);
        Serial.print(',');
        Serial.println(now_speed);
        //Serial.println(encoder_speed_mt(&(motor.enc)));
        /* Current control 
        double now_current = read_current();
        motor.current_integrator += PID_update( &(motor.pid_current), now_current );
        motor.current_integrator = constrain(motor.current_integrator, 0, 250);
        motor.current_limitator = constrain(CONF_CURRENT_LIM_PARA_A * now_current + CONF_CURRENT_LIM_PARA_B, 0, 1);
        motor.current_integrator *= motor.current_limitator;
        write_motor(motor.current_integrator);
        Serial.print(motor.pid_current.cmd);
        Serial.print(',');
        Serial.print(motor.current_limitator);
        Serial.print(',');
        Serial.print(motor.current_integrator/100);
        Serial.print(',');
        Serial.println(now_current);*/
        motor.current_counter = 0;
    }
    /*
    else if (motor.velocity_counter > CONF_CONTROL_VELOCITY_COUNT){
        double CMD_velocity = PID_update( &(motor.pid_velocity), encoder_speed_mt(&(motor.enc)) );
        PIDsetTarget( &(motor.pid_current), CMD_velocity );
        motor.velocity_counter = 0;
    }*/
    motor.current_counter++;
    //motor.velocity_counter++;
}