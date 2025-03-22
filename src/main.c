#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Include only the tasks you want to test:
#include "wifi_task.h"
#include "debug_task.h"
#include "config.h"  // Contains Wi-Fi credentials and UDP port

int main(void) {
    stdio_init_all();
    // Give time for USB/serial to initialize:
    sleep_ms(1000);

    // Initialize Wi-Fi hardware
    wifiSetup();

    // Create only the Wi-Fi and Debug tasks:
    xTaskCreate(wifiTask, "WiFi Task", 1024, NULL, 3, NULL);
    xTaskCreate(debugTask, "Debug Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler:
    vTaskStartScheduler();

    while (1) {
    }

    return 0;
}