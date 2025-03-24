#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "servo.h"
#include "pid.h"
#include "encoder.h"
#include "drive_esc.h"  // Make sure this matches your include path


// The i2c_dma_* functions block the calling task, but allow other tasks to continue running.
// Eg. while waiting on the i2c transfer from the IMU, we can also communicate with the lidar or do calculations

// Also, keep in mind that we have 2 cores. So something can be running simultaniously with an ISR.
// Pretty sure that as long as we do all of our setup code on one core, we'll be fine.

/* 
int main()
{    
    //printf("testing 3");
    stdio_init_all();
    //printf("testing4");
    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while ( to_ms_since_boot(get_absolute_time()) < start_ms+3000) {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");

    //imuSetup();
    //servoSetup();
    //pidSetup();
    escSetup();
    vTaskStartScheduler();
    
}
*/

int main() {
    stdio_init_all();  // Initialize USB serial

    // Wait for the serial connection to be ready (optional)
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms + 3000) {
        printf("Waiting for serial...\n");
        sleep_ms(500);
    }

    printf("Starting ESC control system...\n");
    encoderSetup();
    encoderRead(&position, &speed);
    printf("pos: %.2f m, speed: %.2f m/s \n", position, speed);
    escSetup();  // Initialize ESC control + serial task

    vTaskStartScheduler();  // Start FreeRTOS scheduler (never returns)
}