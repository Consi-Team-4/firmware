#include "encoder.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "pico/stdlib.h"

#include "simple_encoder_substep.h"


# define METERS_PER_SUBSTEP 0.0000191614930635 // 52188 substeps in 1.5m
# define PHASE_0_SIZE 154


static substep_state_t state;
static const PIO pio = pio0;
static const uint sm = 0;
static const uint ENCODER_PIN = 5; // Pin D10

static uint64_t prevReadIrqMicros;

static StaticSemaphore_t encoderMutexBuffer;
static SemaphoreHandle_t encoderMutex;

static float cachedPosition;
static float cachedSpeed;

void encoderSetup() {
    pio_add_program(pio, &simple_encoder_substep_program);
    substep_init_state(pio, sm, ENCODER_PIN, &state);
    state.idle_stop_samples = 1000;
    substep_set_calibration_data(&state, PHASE_0_SIZE);
    encoderMutex = xSemaphoreCreateMutexStatic(&encoderMutexBuffer);
}

void encoderRead(float *position, float *speed) {
    // Will cause problems if read more frequently than once every 13 cpu cycles ~= 0.1us
    // This ensures that we don't run into issues (need 2 us because worst case rounds down, doesn't matter since it's still incredibly small)
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    if (to_us_since_boot(get_absolute_time()) > prevReadIrqMicros + 2) { // Safe to read
        substep_update(&state);
        // This is later than the actual time it was read, but the encoder saves that as a 32 bit number, which will roll over every ~71s
        // I could combine this and the number from the encoder to avoid the delay, but I'm lazy
        prevReadIrqMicros = to_us_since_boot(get_absolute_time());
        
        cachedPosition = METERS_PER_SUBSTEP * state.position;
        cachedSpeed = METERS_PER_SUBSTEP * 0.953674316406f * state.speed_2_20; // Magic number is seconds per 2^20 microseconds

        *position = cachedPosition;
        *speed = cachedSpeed;
    } else { // Use cached values
        *position = cachedPosition;
        *speed = cachedSpeed;
    }
    xSemaphoreGive(encoderMutex);
    // Check if we're trying to read again too soon. If so, use cached results
}

void encoderReadDebug(uint32_t *position, uint32_t *speed_2_20) {
    // Will cause problems if read more frequently than once every 13 cpu cycles ~= 0.1us
    // This ensures that we don't run into issues (need 2 us because worst case rounds down, doesn't matter since it's still incredibly small)
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    if (to_us_since_boot(get_absolute_time()) > prevReadIrqMicros + 2) { // Safe to read
        substep_update(&state);
        // This is later than the actual time it was read, but the encoder saves that as a 32 bit number, which will roll over every ~71s
        // I could combine this and the number from the encoder to avoid the delay, but I'm lazy
        prevReadIrqMicros = to_us_since_boot(get_absolute_time());
        
        cachedPosition = METERS_PER_SUBSTEP * state.position;
        cachedSpeed = METERS_PER_SUBSTEP * 0.953674316406f * state.speed_2_20; // Magic number is seconds per 2^20 microseconds

        *position = state.position;
        *speed_2_20 = state.speed_2_20;
    } else { // Use cached values
        *position = state.position;
        *speed_2_20 = state.speed_2_20;
    }
    xSemaphoreGive(encoderMutex);
    // Check if we're trying to read again too soon. If so, use cached results
}