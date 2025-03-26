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

#include "imu.h"

#include <math.h>

#define SERVO_PIN_1 26
#define SERVO_PIN_2 27
#define SERVO_PIN_3 28
#define SERVO_PIN_4 29

#define WHEEL_DISTANCE 300          // millimeters
#define THETA_LIDAR 30 * M_PI / 180 // degrees, angle of lidar
#define THETA_IMU 0 * M_PI / 180    // degrees, angle retrieved from IMU
#define LIDAR_HEIGHT 284            // millimeters

static StaticTask_t servoTaskBuffer;
static StackType_t servoStackBuffer[1000];
TaskHandle_t servoTask;

void servoTaskFunc(void *);
void delay(int milliseconds);

void servoSetup()
{
    int servo_pins[] = {SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3, SERVO_PIN_4};

    for (int i = 0; i < 4; i++) // set up all four servos
    {
        gpio_set_function(servo_pins[i], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(servo_pins[i]);
        pwm_set_clkdiv(slice_num, 125.0);   // Set clock to 1MHz
        pwm_set_wrap(slice_num, 10 * 1000); // 10ms period - Servos normally use 20ms period, but faster frequency means lower latency

        pwm_set_gpio_level(servo_pins[i], (1500)); // Servo to the middle
        pwm_set_enabled(slice_num, true);
        sleep_ms(1000);
    }
    servoTask = xTaskCreateStatic(servoTaskFunc, "servoTask", sizeof(servoStackBuffer) / sizeof(StackType_t), NULL, 3, servoStackBuffer, &servoTaskBuffer);
}

/**
 * Needed (versus using sleep()) because windows and mac have different versions of this function :(
 */
void delay(int milliseconds)
{
    vTaskDelay(milliseconds);
}

void setServo1(int change) // front right
{
    int setting = (change - 0) * (1500 - 400) / (3000 - 0);
    // edit change based on servo
    pwm_set_gpio_level(SERVO_PIN_1, (setting));
}

void setServo2(int change) // front left
{
    change = 3000 - change;
    int setting = (change - 0) * (2500 - 100) / (3000 - 0);
    // edit change based on servo
    pwm_set_gpio_level(SERVO_PIN_2, (setting));
}

void setServo3(int change) // back right
{
    change = 3000 - change;
    int setting = (change - 0) * (2000 - 500) / (3000 - 0);
    // edit change based on servo
    pwm_set_gpio_level(SERVO_PIN_3, (3000 - setting));
}

void setServo4(int change) // back left
{
    int setting = (change - 0) * (2000 - 500) / (3000 - 0);
    // edit change based on servo
    pwm_set_gpio_level(SERVO_PIN_4, (setting));
}

/**
 * Adjusts the servo by some delta
 */
void adjustServoPosition(double delta, int lidarReading)
{
    if (delta > 3000)
    {
        delta = 3000;
    }
    if (delta < 0)
    {
        delta = 0;
    }
    printf("adjusting servos");

    // get current speed of the car from encoder
    // float currentSpeed = getSpeed();
    float currentSpeed = 1000; // for now

    int tiltDistance = LIDAR_HEIGHT * sin(THETA_IMU);
    int lidarDistance = lidarReading * cos(THETA_LIDAR - THETA_IMU);

    // get amount of time that we should delay before adjusting front wheels
    vTaskDelay((lidarDistance - tiltDistance) / (currentSpeed * 1000));

    // set front
    setServo1(delta);
    setServo2(delta);

    vTaskDelay(WHEEL_DISTANCE / (currentSpeed * 1000)); // should delay based on current speed and length of car

    // set back
    setServo3(delta);
    setServo4(delta);
}

void servoTaskFunc(void *)
{
    while (true)
    {

        // setServo3(0);
        // setServo4(0);
        setServo1(2500);
        printf("1\n");
        printf("%d", pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(50));
        setServo2(2500);
        printf("2\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        setServo3(2500);
        printf("3\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        setServo4(2500);
        printf("4\n");
        vTaskDelay(pdMS_TO_TICKS(50));

        // setServo1()
        // pwm_set_gpio_level(SERVO_PIN_1, (1000));
        // setServo2(400);

        // pwm_set_gpio_level(SERVO_PIN_2, (2500));

        // setServo3(500);
        // setServo4(500);

        // pwm_set_gpio_level(SERVO_PIN_3, (500));

        // pwm_set_gpio_level(SERVO_PIN_4, (2500));

        // pwm_set_gpio_level(SERVO_PIN_3, (120));
        // pwm_set_gpio_level(SERVO_PIN_4, (120));

        // delay(500);
        // pwm_set_gpio_level(SERVO_PIN_2, (0));

        // delay(500);
        // imuData_t imuData;
        // imuGetData(&imuData);

        // pwm_set_gpio_level(SERVO_PIN_1, (int)(60));
        // vTaskDelay(4);
    }
}