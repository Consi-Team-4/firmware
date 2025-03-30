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

typedef struct servoLimits_s {
    int minPosition;
    int maxPosition;
} servoLimits_t;

const servoLimits_t servoLimits[SERVO_COUNT] = {
    [SERVO_FR]      = { -600,   150 },
    [SERVO_FL]      = { -600,   150 },
    [SERVO_BR]      = { -800,   -50 },
    [SERVO_BL]      = { -800,   -50 },
    [ESC]           = { -1000,  1000},
    [SERVO_STEER]   = { -300,   300 },
};

void servoSetup();

// Value should be between -1000 and 1000 inclusive
void servoWrite(ServoID servo, int value);

#endif