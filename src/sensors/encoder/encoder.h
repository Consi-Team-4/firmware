#ifndef ENCODER_H
#define ENCODER_H


void encoderSetup();

// Only call from within a task
void encoderRead(float *position, float *speed);

#endif