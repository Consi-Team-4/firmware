#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"


// The i2c_dma_* functions block the calling task, but allow other tasks to continue running.
// Eg. while waiting on the i2c transfer from the IMU, we can also communicate with the lidar or do calculations

// Also, keep in mind that we have 2 cores. So something can be running simultaniously with an ISR.
// Pretty sure that as long as we do all of our setup code on one core, we'll be fine.


int main()
{    
    stdio_init_all();

    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while ( to_ms_since_boot(get_absolute_time()) < start_ms+1000) {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");

    imuSetup();

    vTaskStartScheduler();
}
