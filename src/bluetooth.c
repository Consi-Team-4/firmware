#include "bluetooth.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART Configuration for DSD TECH HC-05
#define UART_ID_1 uart1   // HC-05 connection
#define UART1_TX_PIN 20    // D8 Connect to HC-05 RXD
#define UART1_RX_PIN 21    // D9 Connect to HC-05 TXD
#define BAUD_RATE 9600    // Default HC-05 baud rate

// Bluetooth tasks
static void task(void *pvParameters) {
    while (1) {
        int retval = stdio_getchar_timeout_us(0);
        if (retval != PICO_ERROR_TIMEOUT) {
            uart_putc_raw(UART_ID_1, (uint8_t)retval);
        }
        if (uart_is_readable(UART_ID_1)) {
            char ch = uart_getc(UART_ID_1);
            stdio_putchar_raw(ch);
        }
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
    xTaskCreate(task, "USB to HC05", 1000, NULL, 1, NULL);
}