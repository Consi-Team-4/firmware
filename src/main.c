#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"


void led_task()
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


    StaticTask_t xTaskBuffer;
    StackType_t xStackBuffer[32*1000];
    xTaskCreateStatic(led_task, "LED_Task", 256, NULL, 1, xStackBuffer, &xTaskBuffer);
    vTaskStartScheduler();

    while(1){};
}