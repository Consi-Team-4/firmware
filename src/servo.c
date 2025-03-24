#include "servo.h"
#include <stdio.h>
#include "encoder.h"

#include "FreeRTOS.h"
#include "task.h"

#include <time.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "imu.h"

#include "math.h"

#define SERVO_PIN_1 26
#define SERVO_PIN_2 27
#define SERVO_PIN_3 28
#define SERVO_PIN_4 29

#define WHEEL_DISTANCE 300 // millimeters
#define THETA_LIDAR 30     // degrees, angle of lidar
#define THETA_IMU 0        // degrees, angle retrieved from IMU
#define LIDAR_HEIGHT 284   // millimeters

static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

void servoTaskFunc(void *);

void servoSetup()
{
    int servo_pins[] = {SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3, SERVO_PIN_4};

    for (int i = 0; i < 4; i++) // set up all four servos
    {
        gpio_set_function(servo_pins[i], GPIO_FUNC_PWM);
        const uint slice_num = pwm_gpio_to_slice_num(servo_pins[i]);
        pwm_set_clkdiv(slice_num, 125.0);  // Set clock to 1MHz
        pwm_set_wrap(slice_num, 5 * 1000); // 5ms period - Servos normally use 20ms period, but faster frequency means lower latency

        pwm_set_gpio_level(servo_pins[i], 1500); // Servo to the middle
        pwm_set_enabled(slice_num, true);        // Start the pwm!
    }

    servoTask = xTaskCreateStatic(servoTaskFunc, "servoTask", sizeof(servoStackBuffer) / sizeof(StackType_t), NULL, 3, servoStackBuffer, &servoTaskBuffer);
}

/**
 * Needed (versus using sleep()) because windows and mac have different versions of this function :(
 */
void delay(int milliseconds)
{
    clock_t start_time = clock();
    while ((clock() - start_time) < milliseconds * (CLOCKS_PER_SEC / 1000))
    {
        // do nothing
    }
}

void setFront(int change)
{

    pwm_set_gpio_level(SERVO_PIN_1, (change));
    pwm_set_gpio_level(SERVO_PIN_2, (change));
}

void setBack(int change)
{
    pwm_set_gpio_level(SERVO_PIN_3, (change));
    pwm_set_gpio_level(SERVO_PIN_4, (change));
}

/**
 * Adjusts the servo by some delta
 */
void adjustServoPosition(double delta, int lidarReading)
{
    printf("adjusting servos");

    // get current speed of the car from encoder
    // float currentSpeed = getSpeed();
    float currentSpeed = 10; // for now

    int tiltDistance = LIDAR_HEIGHT * sin(THETA_IMU);
    int lidarDistance = lidarReading * cos(THETA_LIDAR - THETA_IMU);

    // get amount of time that we should delay before adjusting front wheels
    delay((lidarDistance - tiltDistance) / (currentSpeed * 1000));

    // set front
    setFront(delta);

    delay(WHEEL_DISTANCE / (currentSpeed * 1000)); // should delay based on current speed and length of car

    // set back
    setBack(delta);
}

void servoTaskFunc(void *)
{
    while (true)
    {
        // Swap out with timer later?

        imuData_t imuData;
        imuGetData(&imuData);

        // pwm_set_gpio_level(SERVO_PIN_1, (int)(60));
        // vTaskDelay(4);
    }
}