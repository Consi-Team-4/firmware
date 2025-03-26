#include "servo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "encoder.h"
#include "imu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// === Servo GPIOs ===
#define SERVO_PIN_1 26
#define SERVO_PIN_2 27
#define SERVO_PIN_3 28
#define SERVO_PIN_4 29

#define WHEEL_DISTANCE 300
#define THETA_LIDAR (30 * M_PI / 180)
#define THETA_IMU   (0 * M_PI / 180)
#define LIDAR_HEIGHT 284

// === FreeRTOS task handles ===
static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

static StaticTask_t serialServoTaskBuffer;
static StackType_t serialServoStackBuffer[1000];
TaskHandle_t serialServoTask;

// === Prototypes ===
void servoTaskFunc(void *);
void serialServoCommandTaskFunc(void *);
void delay(int milliseconds);

// === Setup ===
void servoSetup() {
    int servo_pins[] = {SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3, SERVO_PIN_4};

    for (int i = 0; i < 4; i++) {
        gpio_set_function(servo_pins[i], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(servo_pins[i]);
        pwm_set_clkdiv(slice_num, 125.0);
        pwm_set_wrap(slice_num, 5000); // 5ms period
        pwm_set_gpio_level(servo_pins[i], 1500);
        pwm_set_enabled(slice_num, true);
        sleep_ms(100);
    }

    servoTask = xTaskCreateStatic(
        servoTaskFunc, "servoTask",
        sizeof(servoStackBuffer) / sizeof(StackType_t), NULL, 3,
        servoStackBuffer, &servoTaskBuffer
    );

    serialServoTask = xTaskCreateStatic(
        serialServoCommandTaskFunc, "serialServoTask",
        sizeof(serialServoStackBuffer) / sizeof(StackType_t), NULL, 2,
        serialServoStackBuffer, &serialServoTaskBuffer
    );
}

void delay(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// === Individual servo PWM translation ===
void setServo1(int change) {
    int setting = (change - 0) * (1500 - 400) / (3000 - 0);
    pwm_set_gpio_level(SERVO_PIN_1, setting);
}

void setServo2(int change) {
    change = 3000 - change;
    int setting = (change - 0) * (2500 - 100) / (3000 - 0);
    pwm_set_gpio_level(SERVO_PIN_2, setting);
}

void setServo3(int change) {
    change = 3000 - change;
    int setting = (change - 0) * (2000 - 500) / (3000 - 0);
    pwm_set_gpio_level(SERVO_PIN_3, 3000 - setting);
}

void setServo4(int change) {
    int setting = (change - 0) * (2000 - 500) / (3000 - 0);
    pwm_set_gpio_level(SERVO_PIN_4, setting);
}

// === LIDAR-driven coordinated motion (if used) ===
void adjustServoPosition(double delta, int lidarReading) {
    if (delta > 3000) delta = 3000;
    if (delta < 0) delta = 0;

    float currentSpeed = 1000; // Placeholder speed
    int tiltDistance = LIDAR_HEIGHT * sin(THETA_IMU);
    int lidarDistance = lidarReading * cos(THETA_LIDAR - THETA_IMU);

    vTaskDelay((lidarDistance - tiltDistance) / (currentSpeed * 1000));

    setServo1(delta);
    setServo2(delta);

    vTaskDelay(WHEEL_DISTANCE / (currentSpeed * 1000));

    setServo3(delta);
    setServo4(delta);
}

// === Idle servo task (you can modify or remove this) ===
void servoTaskFunc(void *params) {
    (void)params;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Idle
    }
}

// === Serial control task: S1 1500, S4 2300, etc ===
void serialServoCommandTaskFunc(void *params) {
    char input[32];

    while (true) {
        int i = 0;
        printf("Enter servo command (e.g. S1 1500): ");

        while (true) {
            int c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (c == '\n' || c == '\r') {
                input[i] = '\0';
                break;
            }

            if (i < sizeof(input) - 1) {
                input[i++] = (char)c;
                putchar(c);
            }
        }

        int servoNum = 0, pwm_us = 0;

        // Handle SALL command
        if (sscanf(input, "SALL %d", &pwm_us) == 1) {
            if (pwm_us < 0 || pwm_us > 3000) {
                printf("\nError: PWM out of range (0–3000)\n");
                continue;
            }
        
            setServo1(pwm_us);
            setServo2(pwm_us);
            setServo3(pwm_us);
            setServo4(pwm_us);
        
            printf("\nSet ALL servos to %d us\n", pwm_us);
            continue;
        }
        
        // Handle S1–S4 command
        if (sscanf(input, "S%d %d", &servoNum, &pwm_us) == 2) {
            if (pwm_us < 0 || pwm_us > 3000) {
                printf("\nError: PWM out of range (0–3000)\n");
                continue;
            }
        
            switch (servoNum) {
                case 1: setServo1(pwm_us); break;
                case 2: setServo2(pwm_us); break;
                case 3: setServo3(pwm_us); break;
                case 4: setServo4(pwm_us); break;
                default:
                    printf("\nInvalid servo ID: %d (use 1–4)\n", servoNum);
                    continue;
            }
        
            printf("\nSet Servo %d to %d us\n", servoNum, pwm_us);
        } else {
            printf("\nInvalid format. Use:\n");
            printf("  S<servo#> <PWM_us> (e.g., S2 1800)\n");
            printf("  SALL <PWM_us> (e.g., SALL 2000)\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}