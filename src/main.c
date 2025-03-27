#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

#include "imu/imu.h"
#include "encoder/encoder.h"
#include "servo.h"

#include "lidar.h"
#include "constants.h"
#include "bluetooth.h" 


// void printKalmanState(TimerHandle_t xTimer) {
//     imuData_t imu;
//     imuGetData(&imu);
//     // float degGx = 180.0 / M_PI * imu.Gx;
//     // float degGy = 180.0 / M_PI * imu.Gy;
//     // float degGz = 180.0 / M_PI * imu.Gz;
//     // printf("%10lluus\t% 7.4fm/s^2x\t% 7.4fm/s^2y\t% 7.4fm/s^2z\t% 7.2fdeg/sx\t% 7.2fdeg/sy\t% 7.2fdeg/sz\n", imu.micros, imu.Ax, imu.Ay, imu.Az, degGx, degGy, degGz);

//     kalmanState_t s;
//     kalmanGetState(&s);
//     float degBeta = 180.0 / M_PI * s.beta;
//     float degVBeta = 180.0 / M_PI * s.vbeta;
//     float degGamma = 180.0 / M_PI * s.gamma;
//     float degVGamma = 180.0 / M_PI * s.vgamma;
//     printf("Z:% 7.3f\tvZ:% 7.3f\taZ:% 7.3f\tX:% 7.3f\tvX:% 7.3f\taX:% 7.3f\tBeta:% 7.3f\tvBeta:% 7.3f\tGamma:% 7.3f\tvGamma:% 7.3f\n", s.z, s.vz, -(imu.Az-GRAVITY), s.x, s.vx, -imu.Ax, degBeta, degVBeta, degGamma, degVGamma);
//     //printP();
// }

const uint start_button = 18;

int main()
{
    stdio_init_all();
    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms+3000) {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");


    // TaskHandle_t kalmanTask = kalmanSetup();
    // imuSetup(kalmanTask);
    // encoderSetup();
    // // servoSetup();
    // // lidarSetup();

    // printf("Setup complete!\n");

    

    
    imuSetup();
    servoSetup();
    encoderSetup();

    gpio_init(start_button);
    gpio_set_dir(start_button, false);
    gpio_pull_up(start_button);
    sleep_ms(10);

    while(gpio_get(start_button)) {
        sleep_ms(50);
        printf("Waiting for start button.\n");
        sleep_ms(50);
        printf("Waiting for start button...\n");
    }

    vTaskStartScheduler();
}