#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "servo.h"
#include "bluetooth.h" 

int main()
{
    stdio_init_all();
    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) < start_ms+1000) {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");
    
    // Initialize all subsystems
    imuSetup();
    servoSetup();
    bluetoothSetup();
    
    // Start the scheduler
    vTaskStartScheduler();
}