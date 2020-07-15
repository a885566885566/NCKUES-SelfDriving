#ifndef ENCODER_H
#define ENCODER_H
#include "utils.h"

typedef struct{
    int32_t position;
    uint16_t step_counter;
    double speed;
} ENCODER;

void encoder_init(volatile ENCODER* enc);

/* Encoder sampling updater, calculate absolute position
 * , excute this function with timer ISR */
void encoder_sampling(volatile ENCODER* enc);

/* Approximate speed with M/T method */
double encoder_speed_mt(volatile ENCODER* enc);
#endif