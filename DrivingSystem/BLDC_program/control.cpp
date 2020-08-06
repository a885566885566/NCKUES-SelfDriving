#include "control.h"
#include "definitions.h"
#include "utils.h"
#include "pid_control.h"
#include "encoder.h"
#include "Arduino.h"

volatile CONTROL motor;
volatile BEEPER_CONFIG buzzer;
volatile MOTOR_STATE motor_state;

void control_init(volatile CONTROL *ctrl){
    //double KP, double KI, double KD, double DT, double upper, double lower, double Decay
    //PID_init(&(ctrl->pid_velocity), 3, 0, 0, CONF_ENCODER_TS, CONF_CURRENT_LIM, -1*CONF_CURRENT_LIM, 0.9);
    //PID_init(&(ctrl->pid_velocity), 5, 10, 0, CONF_CONTROL_CURRENT_DT, CONF_PWM_LIM, -1*CONF_PWM_LIM, 0.9);
    //PID_init(&(ctrl->pid_velocity), 0.01, 0.001, 0, CONF_CONTROL_CURRENT_DT, CONF_PWM_LIM, -1*CONF_PWM_LIM, 0.9);
    ctrl->mode = CONTROL_CURRENT;
    PID_init(&(ctrl->pid_velocity), 0.01, 0.015, 0, CONF_CONTROL_CURRENT_DT, CONF_PWM_LIM, -1*CONF_PWM_LIM, 0.9);
    PID_init(&(ctrl->pid_current),  80, 2, 0, CONF_CONTROL_CURRENT_DT, CONF_PWM_LIM, -1*CONF_PWM_LIM, 0.9);
    ctrl->current_integrator = 0;
    ctrl->current_limitator = 0;
    encoder_init(&(ctrl->enc));
    ctrl->current_counter = 0;
    ctrl->velocity_counter = 0;
}

void control_set_mode(volatile CONTROL *ctrl, CONTROL_MODE mode){
    if (ctrl->mode == mode) return;
    switch (mode){
    case CONTROL_VELOCITY:
        control_set_current(0);
        break;
    case CONTROL_CURRENT:
        control_set_speed(0);
        break;
    default:
        control_set_current(0);
        control_set_speed(0);
        break;
    }
    ctrl->mode = mode;
}

void control_set_speed(double velocity){
    PIDsetTarget(&(motor.pid_velocity), velocity);
}

void control_set_current(double current){
    PIDsetTarget(&(motor.pid_current), current);
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
    motor_state.current = read_current();
    motor_state.velocity = encoder_speed_mt(&(motor.enc));

    if (motor.mode == CONTROL_NONE){
        motor.current_counter = 0;
        motor.velocity_counter = 0;
    }
    // Current control
    else if (motor.mode == CONTROL_CURRENT 
        && motor.current_counter > CONF_CONTROL_CURRENT_COUNT){
        motor.current_integrator = PID_update( &(motor.pid_current), motor_state.current );
        motor.current_integrator = constrain(motor.current_integrator, 0, 250);
        /*motor.current_limitator = constrain(CONF_CURRENT_LIM_PARA_A * now_current + CONF_CURRENT_LIM_PARA_B, 0, 1);
        motor.current_integrator *= motor.current_limitator;*/
        write_motor(motor.current_integrator);
        motor.current_counter = 0;
    }
    // Velocity control
    else if (motor.mode == CONTROL_VELOCITY 
        && motor.velocity_counter > CONF_CONTROL_VELOCITY_COUNT){
        motor.current_integrator += PID_update( &(motor.pid_velocity), motor_state.velocity );
        motor.current_integrator = constrain(motor.current_integrator, 0, 250);
        //motor.current_limitator = constrain(CONF_CURRENT_LIM_PARA_A * now_current + CONF_CURRENT_LIM_PARA_B, 0, 1);
        //motor.current_integrator *= motor.current_limitator;
        write_motor(motor.current_integrator);
        motor.velocity_counter = 0;
    }
    motor_state.pwm_cmd = motor.current_integrator;
    //double CMD_velocity = PID_update( &(motor.pid_velocity), encoder_speed_mt(&(motor.enc)) );
    //PIDsetTarget( &(motor.pid_current), CMD_velocity );
    motor.current_counter++;
    motor.velocity_counter++;
}
