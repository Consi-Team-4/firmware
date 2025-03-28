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
#define PWM_CLKDIV 125.0 // 1MHz
#define PWM_WRAP 5000 // 5ms
#define PWM_CENTER 1500

// === Mappings ===
static const uint8_t SERVO_GPIO[SERVO_COUNT] = {
    [SERVO_FR]      = 26, // A0
    [SERVO_FL]      = 27, // A1
    [SERVO_BR]      = 28, // A2
    [SERVO_BL]      = 29, // A3
    [ESC]           = 25, // D2
    [SERVO_STEER]   = 15, // D3
};

static const bool SERVO_INVERT[SERVO_COUNT] = {
    [SERVO_FR]      = true,
    [SERVO_FL]      = false,
    [SERVO_BR]      = false,
    [SERVO_BL]      = true,
    [ESC]           = false,
    [SERVO_STEER]   = false,
};


void servoSetup() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        const uint8_t gpio = SERVO_GPIO[i];

        pwm_set_gpio_level(gpio, PWM_CENTER); // Center
        gpio_set_function(gpio, GPIO_FUNC_PWM);

        // Set up slice
        // Note: Since the two front servos and the two rear servos share a slice, this gets called twice. I don't think that's an issue?
        uint slice = pwm_gpio_to_slice_num(gpio);
        pwm_set_clkdiv(slice, PWM_CLKDIV);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_enabled(slice, true);
    }
}

void servoWrite(ServoID servo, int value) {
    if (servo < 0 || servo >= SERVO_COUNT) { return; }

    int clippedValue = value;
    if (clippedValue > 1000) { clippedValue = 1000; }
    else if (clippedValue < -1000) { clippedValue = -1000; }
    if (SERVO_INVERT[servo]) { clippedValue = -1 * clippedValue; } // Make it so all servos go up with positive numbers

    uint us = PWM_CENTER + clippedValue;
    pwm_set_gpio_level(SERVO_GPIO[servo], us);
}
