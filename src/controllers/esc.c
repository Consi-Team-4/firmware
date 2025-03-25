#include "FreeRTOS.h"
#include <stdio.h>
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "esc.h"
#include <stdlib.h>  // for atoi()

#define ESC_PIN 25

static StaticTask_t escTaskBuffer;
static StackType_t escStackBuffer[1000];
TaskHandle_t escTask;

static StaticTask_t serialTaskBuffer;
static StackType_t serialStackBuffer[1000];
TaskHandle_t serialTask;

uint slice_num;  // PWM slice number

//void escTaskFunc(void *);

void escSetup() {
    gpio_set_function(ESC_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(ESC_PIN);

    pwm_set_clkdiv(slice_num, 64.0);          // Clock divider
    pwm_set_wrap(slice_num, 39062);           // Wrap for 50Hz PWM (20ms period)
    pwm_set_enabled(slice_num, true);

    // Neutral signal (1.5ms pulse) to arm the ESC
    uint neutral_level = (uint)(1.5 / 20.0 * 39062);  // 2929
    pwm_set_gpio_level(ESC_PIN, neutral_level);

    printf("ESC armed with neutral signal (1.5ms pulse)\n");
    sleep_ms(3000);

    // Start the ESC PWM update task (optional)
    /*
    escTask = xTaskCreateStatic(
        escTaskFunc,
        "escTask",
        sizeof(escStackBuffer)/sizeof(StackType_t),
        NULL,
        3,
        escStackBuffer,
        &escTaskBuffer
    );
*/
    // Start the serial command task for live control
    serialTask = xTaskCreateStatic(
        serialCommandTaskFunc,
        "serialTask",
        sizeof(serialStackBuffer)/sizeof(StackType_t),
        NULL,
        2,
        serialStackBuffer,
        &serialTaskBuffer
    );
}

/*
void escTaskFunc(void *) {
    while (true) {
        pwm_set_gpio_level(ESC_PIN, 1700);
        sleep_ms(3000);
        vTaskDelay(4);
    }
    
}
*/

void serialCommandTaskFunc(void *params) {
    char input[32];

    while (true) {
        int i = 0;

        printf("Enter PWM value (1000-2000): ");

        // Read one character at a time
        while (true) {
            int c = getchar_timeout_us(0);  // Non-blocking read
            if (c == PICO_ERROR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay
                continue;
            }

            if (c == '\n' || c == '\r') {
                input[i] = '\0';  // Null-terminate
                break;
            }

            if (i < sizeof(input) - 1) {
                input[i++] = (char)c;
                putchar(c);  // Echo back the input
            }
        }

        // Convert input to PWM microseconds (1000-2000)
        int pwm_us = atoi(input);
        printf("\nReceived PWM value: %d\n", pwm_us);

        if (pwm_us < 0 || pwm_us > 3000) {
            printf("Error: PWM value out of range! (0-3000)\n");
            continue;
        }

        // Calculate PWM level based on wrap
        #define PWM_PERIOD_US 20000  // 20ms period for 50Hz

        uint pwm_level = (uint)((pwm_us / (float)PWM_PERIOD_US) * 39062);
        pwm_set_gpio_level(ESC_PIN, pwm_level);
        printf("Updated PWM to %d us (%d level)\n", pwm_us, pwm_level);
    }
}
