#include "bluetooth.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "uart_rx.pio.h"
#include "console.h"

// #define UART0_TX_PIN 0    // GPIO 0 → Connect to HC-05 RXD
#define RX_PIN 1    // GPIO 1 → Connect to HC-05 TXD
#define BAUD_RATE 115200  // Match your HC-05 config

// Stealing from example here
// https://github.com/raspberrypi/pico-examples/tree/master/pio/uart_rx
// Will probably take the part of the console.c that evaluates the commands and put it into a separate function
// That way, commands here can get handled the same way
// (Probably also worth adding a command to handle both throttle and steering in one command so that we don't need to send as many messages)

static const PIO pio = pio1;
static const uint sm = 0;

static char input[64];
static volatile uint i;


static StaticTask_t bluetoothTaskBuffer;
static StackType_t bluetoothStackBuffer[1000];
TaskHandle_t bluetoothTask;

static void bluetoothTaskFunc(void *);


// pio fifo (even when merged) is only 8 deep
// if we didn't use interrupts, then even with the minimum vTaskDelay(1), we would only be able to take in 8 characters per ms at best
// That's not a lot
// So interrupts

// Setup function for HC-05 on uart0
void bluetoothSetup(void) {
    // Initialize pio uart
    int offset = pio_add_program(pio, &uart_rx_program);
    uart_rx_program_init(pio, sm, offset, RX_PIN, BAUD_RATE);
    
    printf("HC-05 Bluetooth module initialized on pio uart\n");

    // Enable interrupt
    
    // Create FreeRTOS task
    bluetoothTask = xTaskCreateStatic(bluetoothTaskFunc, "bluetooth", sizeof(bluetoothStackBuffer)/sizeof(StackType_t), NULL, 2, bluetoothStackBuffer, &bluetoothTaskBuffer);
}



static void blueToothIrqCallback(void) {
    while(!pio_sm_is_rx_fifo_empty(pio, sm)) {
        char c = uart_rx_program_getc(pio, sm);
        
        if (c == '\n' || c == '\r') {
            input[i < sizeof(input) ? i : sizeof(input)-1] = '\0';  // Null-terminate
            // Notify task func and disable interrupt
            break;
        }
        
        if (i < sizeof(input) - 1) {
            input[i] = (char)c;
            i++;
        }
    }
}


static void bluetoothTaskFunc(void *) {
    while (true) {
        ulTaskNotifyTake(true, portMAX_DELAY);
        
        consoleRunCommand(input);

        // Reset
        i = 0;
        // Re enable interrupt
    }
}