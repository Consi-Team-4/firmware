#ifndef IMU_H
#define IMU_H

#include "FreeRTOS.h"
#include "task.h"

static TaskHandle_t imuTask;
static volatile uint64_t imuIrqMicros;
void imuSetup();

#endif
