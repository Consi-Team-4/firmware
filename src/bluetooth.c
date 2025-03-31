#include "bluetooth.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// Use UART0 instead of UART1
#define UART_ID_0 uart0   // HC-05 connection
#define UART0_TX_PIN 0    // GPIO 0 → Connect to HC-05 RXD
#define UART0_RX_PIN 1    // GPIO 1 → Connect to HC-05 TXD
#define BAUD_RATE 115200  // Match your HC-05 config

// Bluetooth task
static void task(void *pvParameters) {
    while (1) {
        int retval = stdio_getchar_timeout_us(0);
        if (retval != PICO_ERROR_TIMEOUT) {
            uart_putc_raw(UART_ID_0, (uint8_t)retval);
        }
        if (uart_is_readable(UART_ID_0)) {
            char ch = uart_getc(UART_ID_0);
            stdio_putchar_raw(ch);
        }
    }
}

// Setup function for HC-05 on uart0
void bluetoothSetup(void) {
    // Initialize UART0 for HC-05
    uart_init(UART_ID_0, BAUD_RATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    
    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    uart_set_format(UART_ID_0, 8, 1, UART_PARITY_NONE);
    
    printf("HC-05 Bluetooth module initialized on uart0\n");
    
    // Create FreeRTOS task
    xTaskCreate(task, "USB to HC05", 1000, NULL, 1, NULL);
}