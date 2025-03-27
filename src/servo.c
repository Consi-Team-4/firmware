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


// === RTOS Task ===
static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

void servoTaskFunc(void *);

void servoSetup() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint gpio = SERVO_GPIO[i];
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(gpio);
        pwm_set_clkdiv(slice, PWM_CLKDIV);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_gpio_level(gpio, 1500); // Center
        pwm_set_enabled(slice, true);
    }

    servoTask = xTaskCreateStatic(
        servoTaskFunc,
        "servoTask",
        sizeof(servoStackBuffer) / sizeof(StackType_t),
        NULL,
        3,
        servoStackBuffer,
        &servoTaskBuffer
    );
}

void servoSetRaw(uint gpio, uint us) {
    if (us < 500) us = 500;
    if (us > 2500) us = 2500;
    pwm_set_gpio_level(gpio, us);
    //log_printf(LOG_DEBUG, "Servo on GPIO %d set to %dus", gpio, us);
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

// Test routine: For each servo, move it through a full range of motion
// 1. Center (1500us)
// 2. Sweep to minimum (500us)
// 3. Sweep to maximum (2500us)
// 4. Return to center
// Includes logging and delays for observation
void servoTest(void) {
    const uint testPositions[] = {1500, 500, 2500, 1500};
    const int delayMs = 500;

    log_printf(LOG_INFO, "Starting servo test sequence...");


    for (ServoID id = 0; id < SERVO_COUNT; id++) {
        log_printf(LOG_INFO, "Testing %s (GPIO %d)", SERVO_NAME[id], SERVO_GPIO[id]);

        for (int i = 0; i < sizeof(testPositions) / sizeof(testPositions[0]); i++) {
            servoSetByID(id, testPositions[i]);
            vTaskDelay(pdMS_TO_TICKS(delayMs));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    log_printf(LOG_INFO, "Servo test loop complete.");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
}

void servoTaskFunc(void *) {

    //servoSetAll(500);
    while (true) {
        //servoTest();
        
    }
}