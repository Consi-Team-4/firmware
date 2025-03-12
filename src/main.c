#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include "imu/imu.h"
#include "servo.h"
#include "lidar.h"

#include "encoder/simple_encoder_substep.h"
#include "simple_encoder_substep.pio.h" // Created by assembling the .pio file. Happens during the make process




static substep_state_t state;
const PIO pio = pio0;
const uint sm = 0;
const uint ENCODER_PIN = 28; // Pin A2

void printEncoderSpeed(TimerHandle_t xTimer) {
    substep_update(&state);
    // print out the result
    printf("pos: %-10d  speed: %-10d  raw_steps: %-10d\n", state.position, state.speed/256, state.raw_step);
}



int main()
{    
    stdio_init_all();

    // Wait a few seconds before doing anything so that the serial monitor has time to load.
    // Otherwise I can't see what happens during the setup to debug :(
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while ( to_ms_since_boot(get_absolute_time()) < start_ms+1000) {
        printf("Waiting...\n");
        sleep_ms(50);
    }
    printf("Start ==========================================================================\n");

    //imuSetup();
    //servoSetup();


    lidarSetup();

    // // Set up encoder
    // pio_add_program(pio, &simple_encoder_substep_program); // defined in simple_encoder_substep.pio.h
    // substep_init_state(pio, sm, ENCODER_PIN, &state); // Pin A2
    // state.idle_stop_samples = 10;
    

    // static StaticTimer_t timerBuffer;
    // TimerHandle_t encoderTimer = xTimerCreateStatic("encoderRead", 10, pdTRUE, NULL, printEncoderSpeed, &timerBuffer);
    // xTimerStart(encoderTimer, portMAX_DELAY);

    vTaskStartScheduler();
}
