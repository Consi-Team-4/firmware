#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoderSetup();

// Only call from within a task
void encoderRead(float *position, float *speed, uint32_t *raw_speed);

#endif