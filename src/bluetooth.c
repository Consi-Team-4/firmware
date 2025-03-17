#include "bluetooth.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART Configuration for DSD TECH HC-05
#define UART_ID_1 uart1   // HC-05 connection
#define UART1_TX_PIN 8    // Connect to HC-05 RXD
#define UART1_RX_PIN 9    // Connect to HC-05 TXD
#define BAUD_RATE 9600    // Default HC-05 baud rate

// Bluetooth tasks
static void uart0_to_uart1_task(void *pvParameters) {
    while (1) {
        if (uart_is_readable(uart0)) {
            char ch = uart_getc(uart0);
            uart_putc_raw(UART_ID_1, ch);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void uart1_to_uart0_task(void *pvParameters) {
    while (1) {
        if (uart_is_readable(UART_ID_1)) {
            char ch = uart_getc(UART_ID_1);
            uart_putc_raw(uart0, ch);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Setup function for HC-05 Bluetooth
void bluetoothSetup(void) {
    // Initialize UART1 for HC-05
    uart_init(UART_ID_1, BAUD_RATE);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    
    // Configure to match HC-05 defaults (8N1)
    uart_set_format(UART_ID_1, 8, 1, UART_PARITY_NONE);
    
    printf("HC-05 Bluetooth module initialized\n");
    
    // Create tasks for Bluetooth communication
    xTaskCreate(uart0_to_uart1_task, "USB to HC05", 256, NULL, 1, NULL);
    xTaskCreate(uart1_to_uart0_task, "HC05 to USB", 256, NULL, 1, NULL);
}