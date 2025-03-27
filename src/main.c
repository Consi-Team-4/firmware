#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

// === Local Modules ===
#include "esc.h"
#include "heartbeat.h"
#include "encoder.h"
#include "servo.h"
#include "log.h"
#include "lidar.h"


// === Main ===
int main() {
    // Initialize USB Serial
    stdio_init_all();

    // Delay to give serial time to connect
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms + 1000) {
        printf("Waiting for serial...\n");
        sleep_ms(50);
    }

    printf("Start ==========================================================================\n");


    log_init();
    log_printf(LOG_INFO, "System startup complete.");

    lidarSetup();
    //encoderSetup();
    //servoSetup(); //starts cli for setting servo positions

 
    //escSetup(); starts cli for setting esc speeds
    //hearbeat_init();

    //Start FreeRTOS Scheduler
    vTaskStartScheduler();

    // Should never hit here
    while (1) { }

    return 0;
}