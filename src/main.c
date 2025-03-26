#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#include <stdio.h>
#include <math.h>
#include "time.h"

#include "pico/stdlib.h"

#include "kalman.h"
#include "imu/imu.h"
#include "encoder/encoder.h"
#include "servo.h"
#include "lidar.h"

void printKalmanState(TimerHandle_t xTimer)
{
    kalmanState_t s;
    kalmanGetState(&s);
    float degBeta = M_PI / 180.0 * s.beta;
    float degVBeta = M_PI / 180.0 * s.vbeta;
    float degGamma = M_PI / 180.0 * s.gamma;
    float degVGamma = M_PI / 180.0 * s.vgamma;
    printf("Z:% 7.3f\tvZ:% 7.3f\tX:% 7.3f\tvX:% 7.3f\tBeta:% 7.3f\tvBeta:% 7.3f\tGamma:% 7.3f\tvGamma:% 7.3f\t", s.z, s.vz, s.x, s.vx, degBeta, degVBeta, degGamma, degVGamma);
}

int main()
{
    stdio_init_all();

    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms + 1000)
    {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");

    // TaskHandle_t kalmanTask = kalmanSetup();
    // imuSetup(kalmanTask);
    // encoderSetup();
    servoSetup();
    lidarSetup();

    static StaticTimer_t timerBuffer;
    TimerHandle_t printTimer = xTimerCreateStatic("kalmanRead", 10, pdTRUE, NULL, printKalmanState, &timerBuffer);
    xTimerStart(printTimer, portMAX_DELAY);

    vTaskStartScheduler();
}
