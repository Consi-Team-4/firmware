#ifndef KALMAN_H
#define KALMAN_H

#include "FreeRTOS.h"
#include "task.h"

// Returns the handle to the kalman filter task
// This task gets notified by the IMU code whenever a new measurement is ready
TaskHandle_t kalmanSetup(void);

typedef struct kalmanState_s {
    uint64_t micros;
    float z;
    float vz;
    float x;
    float vx;
    float beta;
    float vbeta;
    float gamma;
    float vgamma;
} kalmanState_t;

// Gets a copy of the current kalman state
void kalmanGetState(kalmanState_t *buf);

#endif // KALMAN_H