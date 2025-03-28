#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "pico/stdlib.h"
#include <stdio.h>

#include "servo.h"
#include "encoder.h"

// Tuned values
static float escKP = 500;
static float escKI = 5000;
static float escIntegral;
static float escSetpoint;
static float escOutput;
static uint64_t lastMicros;
static bool escFeedbackEnable = false;

static StaticTimer_t feedbackTimerBuffer;
TimerHandle_t feedbackTimer;


void feedback(TimerHandle_t xTimer);
void escFeedback();



// Functions to call from elsewhere
void controllerSetup() {
    feedbackTimer = xTimerCreateStatic("feedback", pdMS_TO_TICKS(5), pdTRUE, NULL, feedback, &feedbackTimerBuffer);
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
    escIntegral = 0;
}





void feedback(TimerHandle_t xTimer) {
    if (escFeedbackEnable) {
        escFeedback();
    }
}

void escFeedback() {
    // Feedback only intended for postive setpoint because encoder speed is always positive

    float encoderPosition, encoderSpeed;
    encoderRead(&encoderPosition, &encoderSpeed);

    uint64_t nowMicros = to_us_since_boot(get_absolute_time());
    float dt = (nowMicros - lastMicros) / 1000000.0; // Convert to milliseconds
    lastMicros = nowMicros;

    // Feedback only intended for postive setpoint because encoder speed is always positive
    float error = escSetpoint - encoderSpeed;
    escIntegral += escKI*error*dt;

    // Prevent integral windup
    // Limiting such that 1000 <= escKP*-5m/s + escIntegral
    if (escIntegral > (1000 + escKP*5)) { escIntegral = (1000 + escKP*5); }
    else if (escIntegral < -escKP*5) { escIntegral = -escKP*5; }

    escOutput = escKP*error + escIntegral;
    if (escOutput > 1000) { escOutput = 1000; }
    else if (escOutput < 0) { escOutput = 0; }
    servoWrite(ESC, (int)escOutput);
}

