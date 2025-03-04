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
        printf("Blink\n");
    }
}

int main()
{    
    stdio_init_all();

    //imuSetup();

    StaticTask_t taskBuffer;
    StackType_t stackBuffer[1000];
    TaskHandle_t ledTask = xTaskCreateStatic(ledTaskFunc, "ledTask", sizeof(stackBuffer)/sizeof(StackType_t), NULL, 10, stackBuffer, &taskBuffer);

    // TaskHandle_t ledTask;
    // xTaskCreate(ledTaskFunc, "ledTask", 1000, NULL, 10, &ledTask);
    vTaskStartScheduler();
}
