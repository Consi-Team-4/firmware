#include "heartbeat.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "encoder.h"
#include "controller.h"
#include "imu.h"

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
    heartbeatTask = xTaskCreateStatic(heartbeatTaskFunc, "heartbeat", sizeof(heartbeatStackBuffer)/sizeof(StackType_t), NULL, 1, heartbeatStackBuffer, &heartbeatTaskBuffer);
}

// Heartbeat Task Implementation
void heartbeatTaskFunc(void *) {
    int count = 0;

    float encoderPos, encoderSpeed;
    uint32_t encoderSteps, encoderSpeed_2_20;
    float setpoint, integral, output;

    imuRaw_t imuRaw;
    imuFiltered_t imuFiltered;

    while (1) {
        encoderRead(&encoderPos, &encoderSpeed);
        encoderReadDebug(&encoderSteps, &encoderSpeed_2_20);
        controllerInfo(&setpoint, &integral, &output);
        imuGetRaw(&imuRaw);
        imuGetFiltered(&imuFiltered);
        

        //printf("Setpoint: % 7.3f Speed: % 7.3f Integral: % 8.3f Output: % 8.3f\n", setpoint, encoderSpeed, integral, output);
        //printf("Gx: % 7.3f Gy: % 7.3f Gz: % 7.3f Ax: % 7.3f Ay: % 7.3f Az: % 7.3f\n", imuRaw.Gx, imuRaw.Gy, imuRaw.Gz, imuRaw.Ax, imuRaw.Ay, imuRaw.Az);
        printf("pitch: % 7.3f Vpitch: % 7.3f roll: % 7.3f Vroll: % 7.3f Vz: % 7.3f Az: % 7.3f\n", imuFiltered.pitch, imuFiltered.Vpitch, imuFiltered.roll, imuFiltered.Vroll, imuFiltered.Vz, imuRaw.Az);
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

        vTaskDelay(pdMS_TO_TICKS(100));
    }
} 