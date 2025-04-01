#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "lidar.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "servo.h"
#include "encoder.h"
#include "imu.h"
#include "lidar.h"

static uint64_t lastMicros;

// ESC feedback control
static float escKP = 500; // Tuned
static float escKI = 5000;

static float escIntegral;
static float escSetpoint;
static float escOutput;

static bool escFeedbackEnable = false;

static float timeConstantToDecayFactor(float timeConstant)
{ // Time constant in seconds
    return expf(-SERVO_PERIOD_MS / (1000 * timeConstant));
}

// P and D are tuned. I and highpass are tunedish
static float suspensionKP = 10000;
static float suspensionKI = 10000;
static float suspensionKD = 800;
float suspensionHighpass; // setting in setup since function call

static bool suspensionFeedbackEnable = true;


const float halfLength = 0.16; //Distance from IMU to center of axle
const float halfWidth = 0.06; // Distance from center of axel to wheel


static bool lidarFeedbackEnable = true;

static LidarQueue_t *qR; // Right side
static uint indexFR;
static uint indexBR;

static LidarQueue_t *qL; // Left side
static uint indexFL;
static uint indexBL;

float lidarKP = 10000;

const float lidarOffset = 0.040;


static StaticTimer_t feedbackTimerBuffer;
TimerHandle_t feedbackTimer;


void feedback(TimerHandle_t xTimer);
void escFeedback(float dt, float encoderSpeed);
void suspensionFeedback(suspensionData_t *data, float dt, float z, float vz, float lidarz);

// Functions to call from elsewhere
void controllerSetup() {
    suspensionHighpass = timeConstantToDecayFactor(5);

    qR = lidarGetQueueR();
    qL = lidarGetQueueL();

    feedbackTimer = xTimerCreateStatic("feedback", pdMS_TO_TICKS(SERVO_PERIOD_MS), pdTRUE, NULL, feedback, &feedbackTimerBuffer);
    xTimerStart(feedbackTimer, portMAX_DELAY);
}

void controllerInfo(float *setpoint, float *integral, float *output) {
    *setpoint = escSetpoint;
    *integral = escIntegral;
    *output = escOutput;
}

void escSetK(float KP, float KI) {
    escIntegral = 0;
    escKP = KP;
    escKI = KI;
}

void escSetSetpoint(float setpoint) {
    escSetpoint = setpoint;
}

void escEnable(bool enable) {
    escFeedbackEnable = enable;
}


void suspensionSetK(float KP, float KI, float KD, float highpassTau) {
    for (int i = 0; i < 4; i++) {
        suspensionData[i].integral = 0;
    }
    suspensionKP = KP;
    suspensionKI = KI;
    suspensionKD = KD;
    suspensionHighpass = timeConstantToDecayFactor(highpassTau);
}

void suspensionEnable(bool enable) {
    suspensionFeedbackEnable = enable;
}


void suspensionLidarSetK(float KP) {
    lidarKP = KP;
}

void suspensionLidarEnable(bool enable) {
    lidarFeedbackEnable = enable;
}




void feedback(TimerHandle_t xTimer) {
    uint64_t nowMicros = to_us_since_boot(get_absolute_time());
    float dt = (nowMicros - lastMicros) / 1000000.0; // Convert to milliseconds
    lastMicros = nowMicros;

    float encoderPosition, encoderSpeed;
        encoderRead(&encoderPosition, &encoderSpeed);

    if (escFeedbackEnable) {
        escFeedback(dt, encoderSpeed);
    }

    if (suspensionFeedbackEnable) {
        imuFiltered_t imuFiltered;
        imuGetFiltered(&imuFiltered);

        float zP = halfLength * sinf(imuFiltered.pitch);
        float zR = halfWidth * sinf(imuFiltered.roll);

        float vzP = halfLength * imuFiltered.Vpitch; // sin(x) = x
        float vzR = halfWidth * imuFiltered.Vroll;   // sin(x) = x

        float lidarzFR = 0;
        float lidarzFL = 0;
        float lidarzBR = 0;
        float lidarzBL = 0;
        if (lidarFeedbackEnable) {
            float frontX = encoderPosition - lidarOffset;
            float backX = frontX - 2*halfLength;
            uint count;

            count = 0;
            while (qR->data[indexFR].x < frontX && count < LIDAR_QUEUE_LEN) {
                indexFR++;
                count++;
                if (indexFR >= LIDAR_QUEUE_LEN) { indexFR = 0; }
            }
            if (count < LIDAR_QUEUE_LEN) {
                lidarzFR = qR->data[indexFR].z;
            } else {
                printf("FR couldn't find data!\n");
            }

            count = 0;
            while (qL->data[indexFL].x < frontX && count < LIDAR_QUEUE_LEN) {
                indexFL++;
                count++;
                if (indexFL >= LIDAR_QUEUE_LEN) { indexFL = 0; }
            }
            if (count < LIDAR_QUEUE_LEN) {
                lidarzFL = qL->data[indexFL].z;
            } else {
                printf("FL couldn't find data!\n");
            }
            
            count = 0;
            while (qR->data[indexBR].x < backX && count < LIDAR_QUEUE_LEN) {
                indexBR++;
                count++;
                if (indexBR >= LIDAR_QUEUE_LEN) { indexBR = 0; }
            }
            if (count < LIDAR_QUEUE_LEN) {
                lidarzBR = qR->data[indexBR].z;
            } else {
                printf("BR couldn't find data!\n");
            }
            
            count = 0;
            while (qL->data[indexBL].x < backX && count < LIDAR_QUEUE_LEN) {
                indexBL++;
                count++;
                if (indexBL >= LIDAR_QUEUE_LEN) { indexBL = 0; }
            }
            if (count < LIDAR_QUEUE_LEN) {
                lidarzBL = qL->data[indexBL].z;
            } else {
                printf("BL couldn't find data!\n");
            }
        }

        suspensionFeedback(suspensionData + SERVO_FR, dt, zP - zR, imuFiltered.Vz + vzP - vzR, lidarzFR);
        suspensionFeedback(suspensionData + SERVO_FL, dt, zP + zR, imuFiltered.Vz + vzP + vzR, lidarzFL);
        suspensionFeedback(suspensionData + SERVO_BR, dt, -zP - zR, imuFiltered.Vz - vzP - vzR, lidarzBR);
        suspensionFeedback(suspensionData + SERVO_BL, dt, -zP + zR, imuFiltered.Vz - vzP + vzR, lidarzBL);
    }
}

void escFeedback(float dt, float encoderSpeed) {
    // Feedback only intended for postive setpoint because encoder speed is always positive
    float error = escSetpoint - encoderSpeed;
    escIntegral += escKI * error * dt;

// Prevent integral windup
// Limiting such that 1000 >= escKP*(-maxError) + escIntegral
#define maxError 2
    const float maxIntegral = 1000 + escKP * maxError;
    const float minIntegral = escKP * (-maxError);
    if (escIntegral > maxIntegral) { escIntegral = maxIntegral; }
    else if (escIntegral < minIntegral) { escIntegral = minIntegral; }

    escOutput = escKP * error + escIntegral;

    if (escOutput > 1000)
    { escOutput = 1000; }
    else if (escOutput < 0)
    { escOutput = 0; }
    servoWrite(ESC, (int)escOutput);
}

void suspensionFeedback(suspensionData_t *data, float dt, float z, float vz, float lidarz) {
    servoLimits_t limits = servoLimits()[data->servo];
    // Using the veloctiy directly as the derivative input
    // No need to deal with setpoint since setpoint is always 0

    data->integral = suspensionHighpass * data->integral + suspensionKI * (-z) * dt;

// Prevent integral windup
// Limiting such that (maxPosition-neutralPosition) >= suspensionKP*(-maxP) + data->integral + suspensionKD*(-maxD)
#define maxP 0.1
#define maxD 1
    const float maxIntegral = limits.maxPosition - data->neutralPosition + suspensionKP * maxP + suspensionKD * maxD;
    const float minIntegral = limits.minPosition - data->neutralPosition + suspensionKP * (-maxP) + suspensionKD * (-maxD);
    if (data->integral > maxIntegral) { data->integral = maxIntegral; }
    if (data->integral < minIntegral) { data->integral = minIntegral; }

    data->output = data->neutralPosition - lidarKP*lidarz + suspensionKP * (-z) + data->integral + suspensionKD * (-vz);

    if (data->output > limits.maxPosition){ data->output = limits.maxPosition; }
    else if (data->output < limits.minPosition) { data->output = limits.minPosition; }
    servoWrite(data->servo, (int)data->output);
}
