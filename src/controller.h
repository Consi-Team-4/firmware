#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>


void controllerSetup();

void controllerInfo(float *setpoint, float *integral, float *output);

void escSetK(float KP, float KI);
void escSetSetpoint(float setpoint);
void escEnable(bool enable);


#endif