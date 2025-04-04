#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

// === Local Modules ===
#include "heartbeat.h"
#include "imu.h"
#include "encoder.h"
#include "servo.h"
#include "log.h"
#include "lidar.h"
#include "console.h"
#include "controller.h"
#include "bluetooth.h"

const uint startButtonPin = 18; // Pin D6

// === Main ===
int main()
{
    // Initialize USB Serial
    stdio_init_all();

    // Delay to give serial time to connect for debugging setup code
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms + 2000) {
        printf("Waiting for serial...\n");
        sleep_ms(50);
    }

    printf("Start ==========================================================================\n");

    printf("Entering encoder setup!");
    encoderSetup();
    printf("Entering servo setup!");
    servoSetup();
    printf("Entering imu setup!");
    imuSetup();
    printf("Entering lidar setup!");
    lidarSetup();
    printf("Entering controller setup!");
    controllerSetup();
    printf("Entering bluetooth setup!");
    bluetoothSetup();
    printf("Entering console setup!");
    consoleSetup();
    printf("Entering heartbeat setup!");
    heartbeatSetup();

    // Wait for red start button pin to be pressed
    // This gives us time to turn on the esc and get it calibrated.
    gpio_init(startButtonPin);
    gpio_set_dir(startButtonPin, false);
    gpio_pull_up(startButtonPin);
    sleep_ms(10);
    while (gpio_get(startButtonPin))
    {
        sleep_ms(50);
        printf("Waiting for start button.\n");
        sleep_ms(50);
        printf("Waiting for start button...\n");
    }

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    // Should never hit here
    while (1) {}

    return 0;
}