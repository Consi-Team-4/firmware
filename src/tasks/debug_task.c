#include "debug_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "imu.h"                   // For imuGetData()
#include "speed_controller.h"      // For getCurrentSpeed()
#include "suspension_controller.h" // For getServoPosition()
#include "lidar.h"                 // For getLidarDistance()

#define FRONT_LEFT_SERVO  0
#define FRONT_RIGHT_SERVO 1
#define REAR_LEFT_SERVO   2
#define REAR_RIGHT_SERVO  3

void debugTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        imuData_t imu;
        imuGetData(&imu);
        
        float currentSpeed = getCurrentSpeed();
        float flServo = getServoPosition(FRONT_LEFT_SERVO);
        float frServo = getServoPosition(FRONT_RIGHT_SERVO);
        float rlServo = getServoPosition(REAR_LEFT_SERVO);
        float rrServo = getServoPosition(REAR_RIGHT_SERVO);
        
        int lidarDistance = getLidarDistance();
        
        printf("Telemetry: Time: %llu us, Speed: %.2f, Pitch: %.2f, Roll: %.2f, LIDAR: %d cm\n",
            imu.micros, currentSpeed, imu.Pitch, imu.Roll, lidarDistance);
        printf("Servos: FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f\n",
            flServo, frServo, rlServo, rrServo);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}