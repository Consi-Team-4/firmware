#include <stdio.h>
#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "task.h"
//#include "encoder/simple_encoder_substep.h"
//#include "simple_encoder_substep.pio.h"
//#include "simple_encoder_substep.c"
#include "servo.h"
#include "pid.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PI 3.14159265358979323846
#define ESC_PIN 15
//#define PIN_A 25
const uint PIN_A = 25;  // Define your GPIO pin
// PID Parameters
double Kc = 1.2;   // Proportional gain
double Ki = 0;   // Integral time constant
double Kd = 0;   // Derivative time constant

// Heating element control variables
double setpoint = 200;  // Desired rpms
double rpms = 25.0;  // Current rpms
double output = 0.0;  // Control signal to the heating element

// PID variables
double error = 0.0;  // Error term
double integral = 0.0;  // Integral term
double derivative = 0.0;  // Derivative term
double prev_error = 0.0;  // Previous error term

// PID parameters tuning
double dt = 1e6;  // Time step for PID control
double min_output = 1500;  // Minimum output value
double max_output = 2000;  // Maximum output value



static StaticTask_t pidTaskBuffer;
static StackType_t pidStackBuffer[1000];
TaskHandle_t pidTask;

void pidTaskFunc(void *); // Task function

// motor rpm calculation function
double get_motor_rpms(double input) {
   double motor_rpms = input * 60;
   return motor_rpms;
}
 
// speed calculation function
double calculate_speed(double frequency, double tire_diameter, double gear_ratio) {
    double tire_circumference = PI * tire_diameter;
    double speed = (frequency * tire_circumference) / (gear_ratio);
    return speed;
}

double desired_frequency(double speed, double tire_diameter, double gear_ratio) {
    double tire_circumference = PI * tire_diameter;
    double desired_frequency = (speed * gear_ratio) / tire_circumference;
    return desired_frequency;
}

void arm_esc() {
    gpio_set_function(ESC_PIN, GPIO_FUNC_PWM);  // Set pin as PWM
    const uint slice_num = pwm_gpio_to_slice_num(ESC_PIN); // Get PWM slice 
    //pwm_set_clkdiv(slice_num, 64.0); // Set clock divider for ~50Hz
    pwm_set_clkdiv(slice_num, 125.0); // Set clock divider for ~50Hz
    //pwm_set_wrap(slice_num, 25000);
    pwm_set_wrap(slice_num, 5*1000);  // Set PWM wrap value
    
    

    // Send neutral signal (1500µs) to arm the ESC
    pwm_set_enabled(slice_num, true); // Enable PWM
    pwm_set_gpio_level(ESC_PIN, 1500);
    
    sleep_ms(3000);
    
    sleep_ms(3000);
    pwm_set_gpio_level(ESC_PIN, 2000);
    
    pwm_set_enabled(slice_num, true); 
    sleep_ms(3000);
    printf("esc armed! \n");
    //pwm_set_gpio_level(ESC_PIN, 1900);
    //printf("1900");
    //sleep_ms(3000);
    //pwm_set_gpio_level(ESC_PIN, 1800);
    //printf("1800");
    //sleep_ms(3000);
    //pwm_set_gpio_level(ESC_PIN, 1700);
    //printf("1700");
    //sleep_ms(3000);
    //pwm_set_gpio_level(ESC_PIN, 1600);
    //printf("1600");
    //sleep_ms(3000);
    //pwm_set_gpio_level(ESC_PIN, 1500);
    //printf("1500");
}

void test_speed() {
    printf("start speed test");
    gpio_set_function(ESC_PIN, GPIO_FUNC_PWM);  // Set pin as PWM
    const uint slice_num = pwm_gpio_to_slice_num(ESC_PIN); // Get PWM slice
    //pwm_set_clkdiv(slice_num, 64.0); // Set clock divider for ~50Hz
    pwm_set_clkdiv(slice_num, 125.0); // Set clock divider for ~50Hz
    //pwm_set_wrap(slice_num, 25000);
    pwm_set_wrap(slice_num, 20*1000);  // Set PWM wrap value
    sleep_ms(3000);
    pwm_set_gpio_level(ESC_PIN, 1600);
    pwm_set_enabled(slice_num, true); // Enable PWM
}

void set_esc_speed(int microseconds) {
    pwm_set_gpio_level(ESC_PIN, microseconds * 25);  // Convert to PWM
}
void setup_gpio() {
    
    gpio_init(PIN_A);       
    gpio_set_dir(PIN_A, GPIO_IN);  // Set pin as input
    gpio_pull_up(PIN_A);           // Enable internal pull-up
}

// PID controller function
double pid_controller(double desired_frequency, double current_frequency) {
    // Calculate the error term
    error = desired_frequency - current_frequency;

    // Calculate the integral term using the trapezoidal rule
    integral += (error + prev_error) * dt / (2.0 * Ki);
    integral = integral < min_output ? min_output : integral;
    integral = integral > max_output ? max_output : integral;

    // Calculate the derivative term using the backward difference
    derivative = (error - prev_error) / dt;

    // Calculate the output signal
    output = Kc * (error + integral + Kd * derivative);
    output = output < min_output ? min_output : output;
    output = output > max_output ? max_output : output;

    // Update the previous error term
    prev_error = error;

    return output;
}


void pidSetup() {
    arm_esc(); // Arm ESC
    
    const uint slice_num = pwm_gpio_to_slice_num(ESC_PIN); // Get PWM slice
    pwm_set_enabled(slice_num, true); 
    pwm_set_gpio_level(ESC_PIN, 1600);
    sleep_ms(3000);
    
    //test_speed();
    printf("speed tested \n");
    //vTaskDelay(pdMS_TO_TICKS(3000));
    printf("testing2 \n");
    
    
    printf("testing2 \n");
    // Create PID task
    pidTask = xTaskCreateStatic(
        pidTaskFunc, "pidTask", sizeof(pidStackBuffer) / sizeof(StackType_t), 
        NULL, 3, pidStackBuffer, &pidTaskBuffer);
    printf("testing3 \n");
    if (pidTask == NULL) {
        printf("Error: PID Task creation failed!\n");
    } else {
        printf("PID Task created successfully!\n");
    }
    
}

void pidTaskFunc(void *pvParameters) {
    /** 
    substep_state_t state;
    
    printf("testing4 \n");

    //setup_gpio();

    stdio_init_all();
    PIO pio = pio0;
    const uint sm = 0;

    pio_add_program(pio, &simple_encoder_substep_program);
    substep_init_state(pio, sm, PIN_A, &state);
    uint last_position = 0, last_raw_step = 0;
    int last_speed = 0;
    */
    while (true) {
        //const uint PIN_A = 25;
        
        const uint slice_num = pwm_gpio_to_slice_num(ESC_PIN); // Get PWM slice
        pwm_set_enabled(slice_num, true); 
        pwm_set_gpio_level(ESC_PIN, 1600);
        
        /**
        
        substep_update(&state);
        //printf("testing5 \n");
        if (last_position != state.position || last_speed != state.speed || last_raw_step != state.raw_step) {
            printf("pos: %-10d  speed: %-10d  raw_steps: %-10d\n", state.position, state.speed, state.raw_step);
            last_position = state.position;
            last_speed = state.speed;
            last_raw_step = state.raw_step;
            //printf("testing6 \n");
        }
            
        sleep_ms(10);
        // PID calculations
        double frequency_c = state.speed;
        */
        double frequency_d = setpoint; // Use setpoint instead of function call for simplicity
        double pwm_output = pid_controller(frequency_d, frequency_c);

        //set_esc_speed(pwm_output);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
    