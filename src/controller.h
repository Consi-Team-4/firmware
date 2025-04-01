#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include <servo.h>
#include <lidar.h>

typedef struct suspensionData_s
{
    ServoID servo;
    float neutralPosition;
    float integral;
    float output;
} suspensionData_t;

static suspensionData_t suspensionData[4] = {
    [SERVO_FR] = {SERVO_FR, -300, 0, 0},
    [SERVO_FL] = {SERVO_FL, -300, 0, 0},
    [SERVO_BR] = {SERVO_BR, -500, 0, 0},
    [SERVO_BL] = {SERVO_BL, -500, 0, 0},
};

void controllerSetup();

void controllerInfo(float *setpoint, float *integral, float *output);

void escSetK(float KP, float KI);
void escSetSetpoint(float setpoint);
void escEnable(bool enable);

void suspensionSetK(float KP, float KI, float KD, float highpassTau);
void suspensionEnable(bool enable);

#endif