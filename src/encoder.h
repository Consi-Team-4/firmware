#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoderSetup();

// Only call from within a task
void encoderRead(float *position, float *speed);

void encoderReadDebug(uint32_t *position, uint32_t *speed_2_20);

#endif