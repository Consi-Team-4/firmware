#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"


typedef struct imuData_s {
    uint64_t micros;
    // Gyros are rad/s (keeping everything in radians so trig functions are convenient)
    float Gx;
    float Gy;
    float Gz;
    // Accelerations are m/s^2
    float Ax;
    float Ay;
    float Az;
} imuData_t;

void imuSetup(TaskHandle_t taskToNotify);

// Only call from within a task
void imuGetData(imuData_t *buf);

#endif
