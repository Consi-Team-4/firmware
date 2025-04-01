#include "bluetooth.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "uart_rx.pio.h"

// #define UART0_TX_PIN 0    // GPIO 0 → Connect to HC-05 RXD
#define RX_PIN 1    // GPIO 1 → Connect to HC-05 TXD
#define BAUD_RATE 115200  // Match your HC-05 config

// Stealing from example here
// https://github.com/raspberrypi/pico-examples/tree/master/pio/uart_rx
// Will probably take the part of the console.c that evaluates the commands and put it into a separate function
// That way, commands here can get handled the same way
// (Probably also worth adding a command to handle both throttle and steering in one command so that we don't need to send as many messages)

const PIO pio = pio1;
const uint sm = 0;

static char inputBuffer[64];
static uint8_t write_index;
static uint8_t cobs_counter;


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
    
    // Create FreeRTOS task
    bluetoothTask = xTaskCreateStatic(bluetoothTaskFunc, "bluetooth", sizeof(bluetoothStackBuffer)/sizeof(StackType_t), NULL, 2, bluetoothStackBuffer, &bluetoothTaskBuffer);
}



static void blueToothIrqCallback(void) {
    while(!pio_sm_is_rx_fifo_empty(pio, sm)) {
        char c = uart_rx_program_getc(pio, sm);
        
      
        
    }
}


static void bluetoothTaskFunc(void *) {
    char input[64];

    while (true) {
        char *p_write = input;

        while (true) {
            while(pio_sm_is_rx_fifo_empty(pio, sm)) {
                vTaskDelay(pdMS_TO_TICKS(1))
            }
            char c = uart_rx_program_getc(pio, sm);
            /
            
        
        
            if (c == '\n' || c == '\r') {
                input[i < sizeof(input) ? i : sizeof(input)-1] = '\0';  // Null-terminate
                putchar('\n');
                break;
            }

            if (i < sizeof(input) - 1) {
                input[i] = (char)c;
                i++;
                putchar(c);  // Echo back the input
            }
        }
        }
    }
}