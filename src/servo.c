#include "servo.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "imu.h"



#define BUTTON_PIN 26
#define SERVO_PIN 27


static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

void servoTaskFunc(void *);

void servoSetup() {
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);


    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    const uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice_num, 125.0); // Set clock to 1MHz
    pwm_set_wrap(slice_num, 5*1000); // 5ms period - Servos normally use 20ms period, but faster frequency means lower latency


    pwm_set_gpio_level(SERVO_PIN, 1500); // Servo to the middle
    pwm_set_enabled(slice_num, true); // Start the pwm!


    servoTask = xTaskCreateStatic(servoTaskFunc, "servoTask", sizeof(servoStackBuffer)/sizeof(StackType_t), NULL, 3, servoStackBuffer, &servoTaskBuffer);
}

void servoTaskFunc(void *) {
    while (true) {
        // Swap out with timer later?

        imuData_t imuData;
        imuGetData(&imuData);

        pwm_set_gpio_level(SERVO_PIN, (int)(1500 + 1000*(imuData.ThetaZ/135.0)));
        vTaskDelay(4);
    }
}