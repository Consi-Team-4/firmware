#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

// === Local Modules ===
#include "utils/kalman.h"
#include "sensors/imu.h"
#include "controllers/encoder.h"
#include "controllers/servo.h"
#include "sensors/lidar.h"

// === Print Kalman Filter State Function ===
void printKalmanState(TimerHandle_t xTimer) {
    kalmanState_t s;
    kalmanGetState(&s);

    float degBeta = s.beta * (180.0 / M_PI);
    float degVBeta = s.vbeta * (180.0 / M_PI);
    float degGamma = s.gamma * (180.0 / M_PI);
    float degVGamma = s.vgamma * (180.0 / M_PI);

    printf(
        "Z:% 7.3f\tvZ:% 7.3f\tX:% 7.3f\tvX:% 7.3f\t"
        "Beta:% 7.3f\tvBeta:% 7.3f\tGamma:% 7.3f\tvGamma:% 7.3f\t\n",
        s.z, s.vz, s.x, s.vx, degBeta, degVBeta, degGamma, degVGamma
    );
}

int main()
{
    stdio_init_all();

    // Serial startup delay for debugging
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms + 1000) {
        printf("Waiting for serial...\n");
        sleep_ms(50);
    }

    printf("Start ==========================================================================\n");

    // === Setup tasks for Kalman filter, IMU, Encoder ===
    TaskHandle_t kalmanTask = kalmanSetup();
    imuSetup(kalmanTask);
    encoderSetup();

    // Optional modules (uncomment when needed)
    //servoSetup();
    //lidarSetup();

    // === Timer to print Kalman filter state ===
    static StaticTimer_t timerBuffer;
    TimerHandle_t printTimer = xTimerCreateStatic(
        "kalmanRead",
        pdMS_TO_TICKS(10),
        pdTRUE,
        NULL,
        printKalmanState,
        &timerBuffer
    );
    xTimerStart(printTimer, portMAX_DELAY);

    // === Start FreeRTOS Scheduler ===
    vTaskStartScheduler();

    // === Should never reach here ===
    while (1) {
        // Optional: error handling if scheduler exits
    }

    return 0;
}