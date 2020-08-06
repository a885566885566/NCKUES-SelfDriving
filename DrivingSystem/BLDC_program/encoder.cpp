#include "encoder.h"
#include "Arduino.h"
#include "definitions.h"
#include "utils.h"

void encoder_init(volatile ENCODER* enc){
    enc->position = 0;
    enc->speed = 0;
    enc->step_counter = 0;
}

/* Encoder update frequency = 2kHz */

void encoder_sampling(volatile ENCODER* enc){
    static bool pre_state = false;
    static bool now_state = false;
    now_state = digitalRead(PIN_TG);
    if ( now_state != pre_state ){
    //if ( pre_state == false && now_state == true ){
        enc->position += 1;
        /*
        if ( digitalRead(PIN_REV) == CONF_MOTOT_DIR )
            enc->position += 1;
        else 
            enc->position -= 1;
        */
        pre_state = now_state;
        if ( enc->step_counter > CONF_ENCODER_TS_COUNT ){
            // Already a Ts (100ms)
            enc->speed = CONF_ENCODER_RESOLUTION * enc->position / (CONF_ENCODER_TP * enc->step_counter);
            enc->step_counter = 0;
            enc->position = 0;
        }
    }
    if (enc->step_counter < 8000)
        enc->step_counter++;
}

double encoder_speed_mt(volatile ENCODER* enc){
    static double filter = 0.5;
    // speed = [rotation / s]
    if ( enc->step_counter > CONF_ENCODER_TS_COUNT ){
        enc->speed = CONF_ENCODER_RESOLUTION * enc->position / (CONF_ENCODER_TP * enc->step_counter);
    //if ( fabs(enc->speed)<10*filter )
        filter = filter * 0.9 + enc->speed * 0.1;
    }
    return filter;
}