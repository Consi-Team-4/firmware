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
    [SERVO_FR]      = false, // Suspension servos rotating downards (and therefore lifting the car) is positive. More convenient for feedback control.
    [SERVO_FL]      = true,
    [SERVO_BR]      = true,
    [SERVO_BL]      = false,
    [ESC]           = false,
    [SERVO_STEER]   = false,
};

static servoLimits_t servoLimitsData[SERVO_COUNT] = {
    [SERVO_FR]      = { -600,   150 },
    [SERVO_FL]      = { -600,   150 },
    [SERVO_BR]      = { -800,   -50 },
    [SERVO_BL]      = { -800,   -50 },
    [ESC]           = { -1000,  1000},
    [SERVO_STEER]   = { -300,   300 },
};

servoLimits_t* servoLimits() {
    return servoLimitsData;
}


void servoSetup() {
    for (ServoID i = 0; i < SERVO_COUNT; i++) {
        const uint8_t gpio = SERVO_GPIO[i];

        pwm_set_gpio_level(gpio, (servoLimits()[i].minPosition + servoLimits()[i].maxPosition)/2); // Center
        gpio_set_function(gpio, GPIO_FUNC_PWM);

        // Set up slice
        // Note: Since the two front servos and the two rear servos share a slice, this gets called twice. I don't think that's an issue?
        uint slice = pwm_gpio_to_slice_num(gpio);
        pwm_set_clkdiv(slice, PWM_CLKDIV);
        pwm_set_wrap(slice, 1000*SERVO_PERIOD_MS);
        pwm_set_enabled(slice, true);
    }
}

void servoWrite(ServoID servo, int value) {
    if (servo < 0 || servo >= SERVO_COUNT) { return; }

    int clippedValue = value;
    if (clippedValue > servoLimits()[servo].maxPosition) { clippedValue = servoLimits()[servo].maxPosition; }
    else if (clippedValue < servoLimits()[servo].minPosition) { clippedValue = servoLimits()[servo].minPosition; }
    if (SERVO_INVERT[servo]) { clippedValue = -1 * clippedValue; } // Make it so all suspension servos go up with negative numbers

    uint us = PWM_CENTER + clippedValue;
    pwm_set_gpio_level(SERVO_GPIO[servo], us);
}
