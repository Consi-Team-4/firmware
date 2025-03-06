#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"


// The i2c_dma_* functions block the calling task, but allow other tasks to continue running.
// Eg. while waiting on the i2c transfer from the IMU, we can also communicate with the lidar or do calculations

// Also, keep in mind that we have 2 cores. So something can be running simultaniously with an ISR.
// Pretty sure that as long as we do all of our setup code on one core, we'll be fine.



// void ledTaskFunc(void *)
// {   
//     const uint LED_PIN = PICO_DEFAULT_LED_PIN;
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     while (true) {
//         gpio_put(LED_PIN, 1);
//         vTaskDelay(100);
//         gpio_put(LED_PIN, 0);
//         vTaskDelay(100);


//         // Test notify task
//         imuIrqMicros = to_us_since_boot(get_absolute_time());
        
//         // Notify imu task so it can handle the I2C
//         xTaskNotifyGive(imuTask);
//     }
// }

void callback(void) {
    //BaseType_t higherPriorityTaskWoken = 0;

    printf("Interrupt triggered!\n");

    // This function will be called on any GPIO interrupt
    // So we start by ignoring it if it's not the one we care about
    if (gpio_get_irq_event_mask(26) & GPIO_IRQ_EDGE_FALL) { // Trigger on INT1 rising edge
        gpio_acknowledge_irq(26, GPIO_IRQ_EDGE_FALL); // Acknowledge the request since we're responding to it

        // Print a message
        printf("Hello there!\n");
    }
    return;
}

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

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);


    gpio_init(26);
    gpio_set_dir(26, GPIO_IN); // Set pin as input
    gpio_pull_up(26);
    
    irq_add_shared_handler(IO_IRQ_BANK0, callback, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); // Tested, know that the handler is being set up correctly
    irq_set_enabled(IO_IRQ_BANK0, true);

    gpio_set_irq_enabled(26, GPIO_IRQ_EDGE_FALL, true);
    printf("Set up interrupt!\n");



    while (true) {
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
    }
}
