#ifndef KALMAN_H
#define KALMAN_H

#include "FreeRTOS.h"
#include "task.h"

// C definitions for interaction with the rest of the code


// Returns the handle to the kalman filter task
// This task can get notified by the IMU code whenever a new measurement is ready
TaskHandle_t kalmanSetup();



#endif