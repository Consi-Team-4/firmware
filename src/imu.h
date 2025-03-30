#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct imuRaw_s {
    uint64_t micros;
    // Gyros are rad/s (keeping everything in radians so trig functions are convenient)
    float Gx;
    float Gy;
    float Gz;
    // Accelerations are m/s^2
    float Ax;
    float Ay;
    float Az;
} imuRaw_t;

typedef struct imuFiltered_s {
    uint64_t micros;
    // Gyros are rad/s (keeping everything in radians so trig functions are convenient)
    float roll;
    float pitch;
    float Vroll;
    float Vpitch;
    float Vz;
} imuFiltered_t;

void imuSetup();

// Only call from within a task
void imuGetRaw(imuRaw_t *buf);
void imuGetFiltered(imuFiltered_t *buf);

void imuSetK(float AngleTau, float LinearTau, float x);

#endif
