#include "lidar.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"


static volatile uint8_t lidar0Buffer[9];
static volatile uint8_t lidar0Index = 0;


static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;


static void uart0RxISR(void);
static void lidarTaskFunc(void *);

void lidarSetup() {

    // Set up uart0 for lidar 0
    uart_init(uart0, 115200);
    gpio_set_function(16, UART_FUNCSEL_NUM(uart0, 16)); // GPIO16/D4 is Uart TX
    gpio_set_function(17, UART_FUNCSEL_NUM(uart0, 17)); // GPIO17/D5 is Uart RX
    // Set uart settings
    uart_set_hw_flow(uart0, false, false); // No CTS or RTS
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE); // 8 data bits, 1 stop bit, no parity bit
    uart_set_fifo_enabled(uart0, false); // No FIFO - want interrupt to trigger as soon as we get a character

    // Set up interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart0RxISR);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(uart0, true, false); // Trigger interrupt when it receives data



    
    lidar0Index = 0; // Reset


    lidarTask = xTaskCreateStatic(lidarTaskFunc, "lidarTask", sizeof(lidarStackBuffer)/sizeof(StackType_t), NULL, 4, lidarStackBuffer, &lidarTaskBuffer);

}


static void uart0RxISR(void){
    BaseType_t higherPriorityTaskWoken = 0;

    while (uart_is_readable(uart0) && lidar0Index < 9) {
        uint8_t rx_char = uart_getc(uart0);
        lidar0Buffer[lidar0Index] = rx_char;
        
        if (lidar0Index < 2 && rx_char != 0x59) { // Invalid frame header
            lidar0Index = 0; // Reset
            continue;

        } else if (lidar0Index == 8) { // Checksum byte
            uint8_t checksum = 0;
            for (uint8_t i=0; i<8; i++) { checksum += lidar0Buffer[i]; }
            if (rx_char != checksum) { // Bad checksum
                lidar0Index = 0; // Reset
                continue;
            } else {
                irq_set_enabled(UART0_IRQ, false); // Turn off ISR so data doesn't get overwritten
                vTaskNotifyGiveFromISR(lidarTask, &higherPriorityTaskWoken); // Wake main task
            }
        }
        lidar0Index++;
    }

    portYIELD_FROM_ISR( &higherPriorityTaskWoken );
}

static void lidarTaskFunc(void *) {
    // Wait for notification from ISR
    while (true) {
        if (ulTaskNotifyTake(true, portMAX_DELAY)) {
            // For now, just printing values
            uint16_t distance = ((uint16_t*)lidar0Buffer)[1];
            uint16_t strength = ((uint16_t*)lidar0Buffer)[2];
            float temperature = ((uint16_t*)lidar0Buffer)[3] / 8.0 - 256;
            printf("% 5ucm\t% 5u\t% 7.3fC\n", distance, strength, temperature);
            
            // Reset
            lidar0Index = 0;
            while(uart_is_readable(uart0)) { uart_getc(uart0); } // Empty queue to clear old data
            irq_set_enabled(UART0_IRQ, true);
        }
    }
}