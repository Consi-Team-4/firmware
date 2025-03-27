#include "servo.h"
#include <stdio.h>
#include "encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include <time.h>
#include "pico/stdlib.h"
#include "timers.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <math.h>
#include "log.h"
#include <stdlib.h> // For atoi()

// === Servo GPIO Pins ===
#define SERVO_PIN_FR 26  // Front Right
#define SERVO_PIN_FL 27  // Front Left
#define SERVO_PIN_BR 28  // Back Right
#define SERVO_PIN_BL 29  // Back Left

// === PWM Config ===
#define PWM_FREQ_HZ 50
#define PWM_WRAP 20000
#define PWM_CLKDIV 125.0

// === Servo IDs ===
typedef enum {
    SERVO_FR = 0,
    SERVO_FL = 1,
    SERVO_BR = 2,
    SERVO_BL = 3,
    SERVO_COUNT
} ServoID;

// === Mappings ===
static const uint8_t SERVO_GPIO[SERVO_COUNT] = {
    [SERVO_FR] = SERVO_PIN_FR,
    [SERVO_FL] = SERVO_PIN_FL,
    [SERVO_BR] = SERVO_PIN_BR,
    [SERVO_BL] = SERVO_PIN_BL,
};

static const bool SERVO_INVERT[SERVO_COUNT] = {
    [SERVO_FR] = true,  // Inverted
    [SERVO_FL] = true,  // Inverted
    [SERVO_BR] = false,
    [SERVO_BL] = false,
};

static const char *SERVO_NAME[SERVO_COUNT] = {
    [SERVO_FR] = "Front Right",
    [SERVO_FL] = "Front Left",
    [SERVO_BR] = "Back Right",
    [SERVO_BL] = "Back Left"
};

// === RTOS Task Handles ===
static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

static StaticTask_t serialTaskBuffer;
static StackType_t serialStackBuffer[1000];
TaskHandle_t serialTaskServo;

// Function prototypes
void servoTaskFunc(void *);
void serialCommandTaskFuncServo(void *);

// === Servo Setup ===
void servoSetup() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint gpio = SERVO_GPIO[i];
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(gpio);
        pwm_set_clkdiv(slice, PWM_CLKDIV);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_gpio_level(gpio, 1500); // Center position
        pwm_set_enabled(slice, true);
    }

    // Start the servo control task
    servoTask = xTaskCreateStatic(
        servoTaskFunc,
        "servoTask",
        sizeof(servoStackBuffer) / sizeof(StackType_t),
        NULL,
        3,
        servoStackBuffer,
        &servoTaskBuffer
    );

    // Start the serial command task
    serialTaskServo = xTaskCreateStatic(
        serialCommandTaskFuncServo,
        "serialTaskServo",
        sizeof(serialStackBuffer) / sizeof(StackType_t),
        NULL,
        2,
        serialStackBuffer,
        &serialTaskBuffer
    );
}

// === Servo Control Functions ===
void servoSetRaw(uint gpio, uint us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    pwm_set_gpio_level(gpio, us);
}

void servoSetByID(ServoID id, uint us) {
    if (id >= SERVO_COUNT) return;

    uint original_us = us;

    if (SERVO_INVERT[id]) {
        us = 3000 - us;  // Invert direction
    }

    servoSetRaw(SERVO_GPIO[id], us);

    log_printf(
        LOG_DEBUG,
        "Set %s (GPIO %d): Requested %dus → Applied %dus%s",
        SERVO_NAME[id],
        SERVO_GPIO[id],
        original_us,
        us,
        SERVO_INVERT[id] ? " [INVERTED]" : ""
    );
}

void servoSetAll(uint us) {
    for (ServoID id = 0; id < SERVO_COUNT; id++) {
        servoSetByID(id, us);
    }
}

// === Serial Command Task ===
void serialCommandTaskFuncServo(void *params) {
    char input[32];

    while (true) {
        int i = 0;
        printf("Enter PWM value (1000-2000) for all servos: ");

        // Read input
        while (true) {
            int c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (c == '\n' || c == '\r') {
                input[i] = '\0';  // Null-terminate
                break;
            }

            if (i < sizeof(input) - 1) {
                input[i++] = (char)c;
                putchar(c);  // Echo input
            }
        }

        // Convert input to PWM microseconds (1000-2000)
        int pwm_us = atoi(input);
        printf("\nReceived PWM value: %d\n", pwm_us);

        // if (pwm_us < 1000 || pwm_us > 2000) {
        //     printf("Error: PWM value out of range! (1000-2000)\n");
        //     continue;
        // }

        // Update all servos
        servoSetAll(pwm_us);

        printf("Updated all servos to %d us\n", pwm_us);
    }
}

// === Main Servo Task (Continuous Operation) ===
void servoTaskFunc(void *) {
    while (true) {
        // Keep the task running, allowing external commands to update servos
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
