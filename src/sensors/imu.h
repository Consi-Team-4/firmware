#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Dummy IMU data structure
typedef struct {
    uint64_t micros;
    float Pitch;
    float Roll;
    // You can add additional fields if needed
} imuData_t;

// Function prototype for retrieving IMU data
void imuGetData(imuData_t *data);

#ifdef __cplusplus
}
#endif

#endif // IMU_H