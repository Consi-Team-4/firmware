#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

// === Local Modules ===
#include "controllers/esc.h"
#include "tasks/heartbeat.h"

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

    // Start Heartbeat Task (prints dummy data)
    xTaskCreate(
        statusHeartbeatTask,
        "StatusHeartbeat",
        1024,      // Stack size in words (not bytes)
        NULL,      // Task parameter
        1,         // Priority
        NULL       // Task handle (optional)
    );

    escSetup(); 

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    // Should never hit here
    while (1) { }

    return 0;
}