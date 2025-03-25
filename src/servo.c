#include "servo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>


#include "imu.h"
#include "encoder.h"

#define PWM_PERIOD 5

const uint steering = 15;
const uint esc = 25;
const uint servos[4] = {26, 27, 28, 29};

static float pulse = 1500;


static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

static StaticTimer_t timerBuffer;
TimerHandle_t timer;

void feedback(TimerHandle_t xTimer);
void servoTaskFunc(void *);

void servoSetup() {
    //26 and 27 share a slice, 28 and 29 share a slice

    gpio_set_function(steering, GPIO_FUNC_PWM);
    pwm_set_gpio_level(steering, 1500);
    gpio_set_function(esc, GPIO_FUNC_PWM);
    pwm_set_gpio_level(esc, 1500);
    int i;
    for (i = 0; i<4; i++) {
        gpio_set_function(servos[i], GPIO_FUNC_PWM);
        pwm_set_gpio_level(servos[i], 1500); // Servo to the middle
    }

    uint slice_num;
    slice_num = pwm_gpio_to_slice_num(steering);
    pwm_set_clkdiv(slice_num, 125.0); // Set clock to 1MHz
    pwm_set_wrap(slice_num, PWM_PERIOD*1000); // 10ms period - Servos normally use 20ms period, but faster frequency means lower latency
    pwm_set_enabled(slice_num, true); // Start the pwm!

    slice_num = pwm_gpio_to_slice_num(esc);
    pwm_set_clkdiv(slice_num, 125.0); // Set clock to 1MHz
    pwm_set_wrap(slice_num, PWM_PERIOD*1000); // 10ms period - Servos normally use 20ms period, but faster frequency means lower latency
    pwm_set_enabled(slice_num, true); // Start the pwm!s

    slice_num = pwm_gpio_to_slice_num(servos[0]);
    pwm_set_clkdiv(slice_num, 125.0); // Set clock to 1MHz
    pwm_set_wrap(slice_num, PWM_PERIOD*1000); // 10ms period - Servos normally use 20ms period, but faster frequency means lower latency
    pwm_set_enabled(slice_num, true); // Start the pwm!

    slice_num = pwm_gpio_to_slice_num(servos[2]);
    pwm_set_clkdiv(slice_num, 125.0); // Set clock to 1MHz
    pwm_set_wrap(slice_num, PWM_PERIOD*1000); // 10ms period - Servos normally use 20ms period, but faster frequency means lower latency
    pwm_set_enabled(slice_num, true); // Start the pwm!

    servoTask = xTaskCreateStatic(servoTaskFunc, "servoTask", sizeof(servoStackBuffer)/sizeof(StackType_t), NULL, 3, servoStackBuffer, &servoTaskBuffer);
    timer = xTimerCreateStatic("feedback", pdMS_TO_TICKS(5), pdTRUE, NULL, feedback, &timerBuffer);
    xTimerStart(timer, portMAX_DELAY); 
}

void feedback(TimerHandle_t xTimer) {
    float position;
    float speed;
    uint32_t raw_speed;
    encoderRead(&position, &speed, &raw_speed);

    static uint8_t count;
    if (count == 0) {
        printf("Speed: %f\tRawSpeed: %u", speed, raw_speed);
    }
    if (count==100){count=0;}
    else {count++;}

    pwm_set_gpio_level(esc, pulse);
}

void servoTaskFunc(void *) {
    char input[32];

    while (true) {
        int i = 0;

        printf("Enter command, value: ");

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
        char command = input[0];
        int value = atoi(input+1);
        printf("\nReceived value: %d\n", value);

        if (command == 'E') {
            pulse = value;
        }
    }
}