#include "heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"  // Include your encoder interface

#include <stdio.h>

// Dummy data values (keep others if still using dummy values for now)
float dummyIMU_Ax = 0.0f;
float dummyIMU_Ay = 0.0f;
float dummyIMU_Az = 9.81f;
float dummyLidarDist = 123.4f;
float dummyServoPos = 15.0f;

// Heartbeat Task Implementation
void statusHeartbeatTask(void *pvParameters) {
    (void)pvParameters;

    const TickType_t xDelay = pdMS_TO_TICKS(1000);  // 1 second
    int count = 0;

    float encoderPos, encoderSpeed;
    uint32_t encoderRaw;

    while (1) {
        encoderRead(&encoderPos, &encoderSpeed, &encoderRaw);

        printf(
            "[Heartbeat %d] IMU: Ax=%.2f Ay=%.2f Az=%.2f | Encoder Pos=%.2f Speed=%.2f Raw=%lu | Servo=%.2f | Lidar=%.2f\n",
            count++,
            dummyIMU_Ax,
            dummyIMU_Ay,
            dummyIMU_Az,
            encoderPos,
            encoderSpeed,
            encoderRaw,
            dummyServoPos,
            dummyLidarDist
        );

        vTaskDelay(xDelay);
    }
} 

void hearbeat_init() {
    xTaskCreate(
        statusHeartbeatTask,
        "StatusHeartbeat",
        1024,      // Stack size in words (not bytes)
        NULL,      // Task parameter
        1,         // Priority
        NULL       // Task handle (optional)
    );
}