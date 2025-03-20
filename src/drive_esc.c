#include <stdio.h>
#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "servo.h"

#define ESC_PIN 15

static StaticTask_t escTaskBuffer;
static StackType_t escStackBuffer[1000];
TaskHandle_t escTask;

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
    printf("neutral arming signal \n");
    sleep_ms(3000);
    //sleep_ms(3000);
    pwm_set_gpio_level(ESC_PIN, 2000);
    printf("full throttle \n");
    pwm_set_enabled(slice_num, true); 
    sleep_ms(3000);
    printf("esc armed!");

    escTask = xTaskCreateStatic(escTaskFunc, "escTask", sizeof(escStackBuffer)/sizeof(StackType_t), NULL, 3, escStackBuffer, &escTaskBuffer);
}

void escTaskFunc(void *) {

    
    
    while (true) {
        // Swap out with timer later?

        

        pwm_set_gpio_level(ESC_PIN, (int)(1500 + 1000*(imuData.ThetaZ/135.0)));
        vTaskDelay(4);
    }
}

