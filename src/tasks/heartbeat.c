#include "heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

// Dummy data values
float dummyIMU_Ax = 0.0f;
float dummyIMU_Ay = 0.0f;
float dummyIMU_Az = 9.81f;

float dummyEncoderPos = 42.0f;
float dummyLidarDist = 123.4f;
float dummyServoPos = 15.0f;

// Heartbeat Task Implementation
void statusHeartbeatTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t xDelay = pdMS_TO_TICKS(1000);  // 1 second
    int count = 0;

    while (1) {
        printf(
            "[Heartbeat %d] IMU: Ax=%.2f Ay=%.2f Az=%.2f | Encoder=%.2f | Servo=%.2f | Lidar=%.2f\n",
            count++,
            dummyIMU_Ax,
            dummyIMU_Ay,
            dummyIMU_Az,
            dummyEncoderPos,
            dummyServoPos,
            dummyLidarDist
        );

        vTaskDelay(xDelay);
    }
}