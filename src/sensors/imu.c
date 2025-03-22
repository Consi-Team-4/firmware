#include "imu.h"

// Dummy implementation: returns constant values.
void imuGetData(imuData_t *data) {
    data->micros = 123456789; // Dummy timestamp in microseconds
    data->Pitch = 1.23f;      // Dummy pitch in degrees
    data->Roll  = 2.34f;      // Dummy roll in degrees
}