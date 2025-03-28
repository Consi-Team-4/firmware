#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"

typedef enum {
    SERVO_FR = 0,
    SERVO_FL = 1,
    SERVO_BR = 2,
    SERVO_BL = 3,
    ESC = 4,
    SERVO_STEER = 5,
    SERVO_COUNT
} ServoID;

void servoSetup();

// Value should be between -1000 and 1000 inclusive
void servoWrite(ServoID servo, int value);

#endif