#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"


// The i2c_dma_* functions block the calling task, but allow other tasks to continue running.
// Eg. while waiting on the i2c transfer from the IMU, we can also communicate with the lidar or do calculations

// Also, keep in mind that we have 2 cores. So something can be running simultaniously with an ISR.
// Pretty sure that as long as we do all of our setup code on one core, we'll be fine.



void ledTaskFunc(void *)
{   
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(100);
        gpio_put(LED_PIN, 0);
        vTaskDelay(100);
    }
}

int main()
{    
    stdio_init_all();

    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while ( to_ms_since_boot(get_absolute_time()) < start_ms+3000) {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");

    imuSetup();

    StaticTask_t taskBuffer;
    StackType_t stackBuffer[1000];
    TaskHandle_t ledTask = xTaskCreateStatic(ledTaskFunc, "ledTask", sizeof(stackBuffer)/sizeof(StackType_t), NULL, 10, stackBuffer, &taskBuffer);

    // TaskHandle_t ledTask;
    // xTaskCreate(ledTaskFunc, "ledTask", 1000, NULL, 10, &ledTask);
    vTaskStartScheduler();
}
