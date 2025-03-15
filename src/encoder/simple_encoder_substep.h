#ifndef SUBSTEP_ENCODER_H
#define SUBSTEP_ENCODER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

typedef struct substep_state_t {
    uint calibration_data[2];
    uint clocks_per_us;
    uint idle_stop_samples;
    PIO pio;
    uint sm;
    uint prev_trans_pos, prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;
    int speed;
    uint position;
    uint raw_step;
} substep_state_t;

static void substep_init_state(PIO pio, int sm, int pin_a, substep_state_t *state);
static void substep_update(substep_state_t *state);
static void substep_calibrate_phases(PIO pio, uint sm);
static void substep_set_calibration_data(substep_state_t *state, int step0);
static void read_pio_data(substep_state_t *state, uint *step, uint *step_us, uint *transition_us);
static uint get_step_start_transition_pos(substep_state_t *state, uint step);
static int substep_calc_speed(int delta_substep, int delta_us);

#endif // SUBSTEP_ENCODER_H
