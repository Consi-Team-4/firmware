#ifndef IMU_H
#define IMU_H

#include <stdint.h>


typedef struct imuData_s {
    uint64_t micros;
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;
    float ThetaZ;
} imuData_t;

void imuSetup();
void imuGetData(imuData_t *buf);

#endif
