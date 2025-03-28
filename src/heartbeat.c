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


static StaticTask_t heartbeatTaskBuffer;
static StackType_t heartbeatStackBuffer[1000];
TaskHandle_t heartbeatTask;

void heartbeatTaskFunc(void *);

void heartbeatSetup() {
    heartbeatTask = xTaskCreateStatic(heartbeatTaskFunc, "heartbeat", sizeof(heartbeatStackBuffer)/sizeof(StackType_t), NULL, 3, heartbeatStackBuffer, &heartbeatTaskBuffer);
}

// Heartbeat Task Implementation
void heartbeatTaskFunc(void *) {
    int count = 0;

    float encoderPos, encoderSpeed;
    uint32_t encoderSteps, encoderSpeed_2_20;

    while (1) {
        encoderRead(&encoderPos, &encoderSpeed);
        encoderReadDebug(&encoderSteps, &encoderSpeed_2_20);

        printf("Odometer: % 7.3f, Speed: % 7.3f\n", encoderPos, encoderSpeed);

        // printf(
        //     "[Heartbeat %d] IMU: Ax=%.2f Ay=%.2f Az=%.2f | Encoder Pos=%.2f Speed=%.2f | Servo=%.2f | Lidar=%.2f\n",
        //     count++,
        //     dummyIMU_Ax,
        //     dummyIMU_Ay,
        //     dummyIMU_Az,
        //     encoderPos,
        //     encoderSpeed,
        //     dummyServoPos,
        //     dummyLidarDist
        // );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 