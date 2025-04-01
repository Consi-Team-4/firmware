#include "lidar.h"
#include "FreeRTOS.h"
#include "controller.h"
#include "task.h"
#include "log.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "servo.h"
#include "encoder.h"
#include "imu.h"
#include <math.h>

#define LIDAR0_UART uart0
#define LIDAR0_TX_PIN 16
#define LIDAR0_RX_PIN 17
#define LIDAR1_UART uart1
#define LIDAR1_TX_PIN 20
#define LIDAR1_RX_PIN 21

#define LIDAR_BAUDRATE 115200
#define RELEVANT_LIDAR_DATA 128
#define TOLERANCE 50

#define WHEEL_DISTANCE 300                       // millimeters
#define LIDAR_THETA 30 * M_PI / 180.0 // base angle of lidar, used for normalization
#define LIDAR_HEIGHT 284                         // millimeters

static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;

static volatile uint8_t lidar0Buffer[9];
static volatile uint8_t lidar0Index = 0;
static volatile bool lidar0NewData = false;

static volatile uint8_t lidar1Buffer[9];
static volatile uint8_t lidar1Index = 0;
static volatile bool lidar1NewData = true;

const float halfL = 0.16; // Distance from IMU to center of axle
const float halfW = 0.06; // Distance from center of axel to wheel

// Forward declarations
static void lidarTaskFunc(void *);
static void uart0RxISR(void);
static void uart1RxISR(void);

#include <stdio.h>
#include <stdlib.h>

LidarQueue_t q_0;
LidarQueue_t q_1;

static uint64_t lastMicros; // for keeping track of how long it's been since we last called for an update to the suspension

LidarQueue_t *lidarGetQueue0() { return &q_0; }
LidarQueue_t *lidarGetQueue1() { return &q_1; }

/**
 * Initializes the queue.
 */
void initializeLidarQueue_t(LidarQueue_t *q)
{
    q->counter = 0;
    q->numPoints = 0;
}

/**
 * Checks if the queue is full. This is determined by whether the counter value has already reached the max size of the queue.
 */
bool isFull(LidarQueue_t *q) { return (q->counter == MAX_QUEUE_SIZE); }

/**
 * Retrieve the LidarData_t item that was most recently added to the queue
 */
LidarData_t peek(LidarQueue_t *q)
{
    // the counter should be the index that we're NEXT going to put a LidarData_t struct, so we should grab counter-1 index
    return q->data[q->counter - 1];
}

/**
 * Adds an element of LidarData_t at the given distance and position to the queue.
 * If the queue is full, resets the counter to 0 and starts overwriting the data currently in the queue.
 * This shouldn't lead to any issues with overwritten data, as long as the queue size is sufficiently large for the speed of the vehicle.
 */
void enqueue(LidarQueue_t *q, float z, float x)
{
    if (isFull(q))
    {
        q->counter = 0;
    }
    // put data into queue
    q->data[q->counter].z = z;
    q->data[q->counter].x = x;
    // increment counter
    q->counter++;
    if (q->numPoints < MAX_QUEUE_SIZE)
    {
        q->numPoints++;
    }
}

/**
 * Prints out the current state of the queue.
 * This implementation prints out just the LiDAR readings, but we can change what field(s) of the LidarData_t struct we want to print.
 */
// void printLidarQueue_t(LidarQueue_t *q)
// {
//     printf("Current LidarQueue_t: ");
//     int count = q->counter;
//     for (int i = count; i < MAX_QUEUE_SIZE; i++)
//     {
//         printf("%d ", q->data[i].lidar_reading); // everything after/including the counter
//     }
//     for (int i = 0; i < count; i++)
//     {
//         printf("%d ", q->data[i].lidar_reading); // everything before the counter
//     }
//     printf("\n");
// }

/**
 * Calculates variance of the given queue based on the amount of data points that we want to look at at a time (RELEVANT_LIDAR_DATA)
 */
// double calculate_queue_variance(LidarQueue_t *q)
// {
//     int sum = 0, mean, variance = 0;
//     int first_point = q->counter - 1 - RELEVANT_LIDAR_DATA;
//     int i = q->counter - 1;
//     int num_points = 0;

//     if (q->numPoints < RELEVANT_LIDAR_DATA)
//     {
//         return -1; // I don't think we need to handle the case of the first few ms where we don't have enough data.
//     }

//     while (i >= 0 && num_points < RELEVANT_LIDAR_DATA) // start at counter - 1, go back to 0. will stop at # of points we want to look at, or 0
//     {
//         if (q->data[i].lidar_reading != 0)
//         {
//             num_points++;
//             sum += q->data[i].lidar_reading;
//             i--;
//         }
//     }
//     if (num_points < RELEVANT_LIDAR_DATA)
//     {
//         i = MAX_QUEUE_SIZE - 1;
//         while (num_points < RELEVANT_LIDAR_DATA)
//         {
//             if (q->data[i].lidar_reading != 0)
//             {
//                 num_points++;
//                 sum += q->data[i].lidar_reading;
//                 i--;
//             }
//         }
//     }
//     mean = sum / RELEVANT_LIDAR_DATA;

//     // reset
//     i = q->counter - 1;
//     num_points = 0;

//     // follow the same pattern of iterating through the queue, but this time accumulate the variance instead of the mean.

//     while (i >= 0 && num_points < RELEVANT_LIDAR_DATA) // start at counter - 1, go back to 0. will stop at # of points we want to look at, or 0
//     {
//         if (q->data[i].lidar_reading != 0)
//         {
//             num_points++;
//             variance += pow(q->data[i].lidar_reading - mean, 2);
//             i--;
//         }
//     }
//     if (num_points < RELEVANT_LIDAR_DATA)
//     {
//         i = MAX_QUEUE_SIZE - 1;
//         while (num_points < RELEVANT_LIDAR_DATA)
//         {
//             if (q->data[i].lidar_reading != 0)
//             {
//                 num_points++;
//                 variance += pow(q->data[i].lidar_reading - mean, 2);
//                 i--;
//             }
//         }
//     }
//     variance /= RELEVANT_LIDAR_DATA;
//     return variance;
// }

void lidarInstSetup(uart_inst_t *uartInst, uint txPin, uint rxPin, void(*isrFunc)(void) )
{
    // initialize queue for lidar data to be fed into

    initializeLidarQueue_t(&q_0);

    uart_init(uartInst, LIDAR_BAUDRATE);
    gpio_set_function(txPin, GPIO_FUNC_UART);
    gpio_set_function(rxPin, GPIO_FUNC_UART);
    uart_set_hw_flow(uartInst, false, false);
    uart_set_format(uartInst, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uartInst, false);
    irq_set_exclusive_handler(UART_IRQ_NUM(uartInst), isrFunc);
    irq_set_enabled(UART_IRQ_NUM(uartInst), true);
    uart_set_irq_enables(uartInst, true, false); // RX interrupt only
}

void lidarSetup()
{
    // set up both of the lidars and the queues to hold their data
    lidarInstSetup(LIDAR0_UART, LIDAR0_TX_PIN, LIDAR0_RX_PIN, &uart0RxISR);
    lidarInstSetup(LIDAR1_UART, LIDAR1_TX_PIN, LIDAR1_RX_PIN, &uart1RxISR);

    lidarTask = xTaskCreateStatic(
        lidarTaskFunc, "lidarTask",
        sizeof(lidarStackBuffer) / sizeof(StackType_t),
        NULL, 3,
        lidarStackBuffer, &lidarTaskBuffer);

    log_printf(LOG_INFO, "TFmini-S LIDARs initialized.\n");
}

static void uart0RxISR(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(LIDAR0_UART))
    {
        uint8_t byte = uart_getc(LIDAR0_UART);

        // Sync to 0x59 0x59 frame headers
        if (lidar0Index == 0 && byte != 0x59)
            continue;
        if (lidar0Index == 1 && byte != 0x59)
        {
            lidar0Index = 0;
            continue;
        }

        lidar0Buffer[lidar0Index++] = byte;

        if (lidar0Index >= 9)
        {
            // Validate checksum
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++)
                checksum += lidar0Buffer[i];

            if (checksum == lidar0Buffer[8])
            {
                irq_set_enabled(UART0_IRQ, false);
                lidar0NewData = true;
                vTaskNotifyGiveFromISR(lidarTask, &higherPriorityTaskWoken);
            }
            else
            {
                log_printf(LOG_WARN, "LIDAR0 checksum mismatch. Dropping frame.");
            }

            lidar0Index = 0;
        }
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static void uart1RxISR(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(LIDAR1_UART))
    {
        uint8_t byte = uart_getc(LIDAR1_UART);

        // Sync to 0x59 0x59 frame headers
        if (lidar1Index == 0 && byte != 0x59)
            continue;
        if (lidar1Index == 1 && byte != 0x59)
        {
            lidar1Index = 0;
            continue;
        }

        lidar1Buffer[lidar1Index++] = byte;

        if (lidar1Index >= 9)
        {
            // Validate checksum
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++)
                checksum += lidar1Buffer[i];

            if (checksum == lidar1Buffer[8])
            {
                irq_set_enabled(UART1_IRQ, false);
                lidar1NewData = true;
                vTaskNotifyGiveFromISR(lidarTask, &higherPriorityTaskWoken);
            }
            else
            {
                log_printf(LOG_WARN, "LIDAR1 checksum mismatch. Dropping frame.");
            }

            lidar1Index = 0;
        }
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/**
 * Determines a setting to set the servos to based on a given LiDAR reading.
 */
// int mapLidarToServo(int lidarReading)
// {
//     // get current speed of the car from encoder
//     // float currentSpeed = getSpeed();
//     // float encoderPosition, currentSpeed;
//     // encoderRead(&encoderPosition, &currentSpeed);
//     // if (currentSpeed > 0.2) {

//     // }

//     imuFiltered_t imuFiltered;
//     imuGetFiltered(&imuFiltered);
//     double theta_imu = imuFiltered.pitch;

//     int normalizedDistance = lidarReading / cos(THETA_LIDAR_NORMALIZATION + theta_imu);

//     int setting = -(((normalizedDistance - 250) * (1000 + 1000) / (800 - 250)) + -1000);
//     // printf("setting: %d, normalized distance: %d\n", setting, normalizedDistance);
//     return setting;
// }

static void lidarTaskFunc(void *)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        uint64_t nowMicros = to_us_since_boot(get_absolute_time());

        // get position from encoder - this is total distance the car has covered
        float encoderPosition, encoderSpeed;
        encoderRead(&encoderPosition, &encoderSpeed);

        imuFiltered_t imuFiltered;
        imuGetFiltered(&imuFiltered);
        float cosP = cosf(imuFiltered.pitch - LIDAR_THETA);
        float sinP = sinf(imuFiltered.pitch - LIDAR_THETA);

        if (lidar0NewData) {
            uint16_t dist_mm0 = (lidar0Buffer[3] << 8) | lidar0Buffer[2];
            uint16_t strength0 = (lidar0Buffer[5] << 8) | lidar0Buffer[4];
            float temp_c0 = ((lidar0Buffer[7] << 8) | lidar0Buffer[6]) / 8.0f - 256;

                if (dist_mm0 == 65535 || strength0 < 100)
            {
                log_printf(LOG_WARN, "LIDAR0 returned invalid or weak signal (Dist=%u, Strength=%u)", dist_mm0, strength0);
            }
            else
            {
                // log_printf(LOG_INFO, "LIDAR0: %3d mm | Strength: %5u | Temp: %.2f°C", dist_mm0, strength0, temp_c0);
            }

            enqueue(&q_0, 0.001 * (LIDAR_HEIGHT + dist_mm0*sinP), encoderPosition + (0.001 * dist_mm0*cosP));


            irq_set_enabled(UART0_IRQ, true);
            lidar0NewData = false;
        }

        if (lidar1NewData) {
            uint16_t dist_mm1 = (lidar1Buffer[3] << 8) | lidar1Buffer[2];
            uint16_t strength1 = (lidar1Buffer[5] << 8) | lidar1Buffer[4];
            float temp_c1 = ((lidar1Buffer[7] << 8) | lidar1Buffer[6]) / 8.0f - 256;

            if (dist_mm1 == 65535 || strength1 < 100)
            {
                log_printf(LOG_WARN, "LIDAR1 returned invalid or weak signal (Dist=%u, Strength=%u)", dist_mm1, strength1);
            }
            else
            {
                // log_printf(LOG_INFO, "LIDAR1: %3d mm | Strength: %5u | Temp: %.2f°C", dist_mm1, strength1, temp_c1);
            }

            enqueue(&q_1, 0.001 * (LIDAR_HEIGHT + dist_mm1*sinP), encoderPosition + (0.001 * dist_mm1*cosP));


            irq_set_enabled(UART1_IRQ, true);
            lidar1NewData = false;
        }

        

        

        // // add this value to the queue of lidar data
        
        

        // // calculate variance of this data
        // int variance0 = calculate_queue_variance(&q_0);
        // int variance1 = calculate_queue_variance(&q_1);

        // // log_printf(LOG_INFO, "VARIANCE0: %3d cm, VARIANCE1: %3d cm\n", variance0, variance1);

        // if (q_0.numPoints == MAX_QUEUE_SIZE)
        // {                              // if we have enough data to warrant making changes
        //     if (variance0 > TOLERANCE) // if the right wheels need to make adjustments
        //     {
        //         // get the current dt since a suspension update was last called
        //         uint64_t nowMicros = to_us_since_boot(get_absolute_time());
        //         float dt = (nowMicros - lastMicros) / 1000000.0; // Convert to milliseconds
        //         lastMicros = nowMicros;

        //         // get imu data
        //         imuFiltered_t imuFiltered;
        //         imuGetFiltered(&imuFiltered);

        //         float zP = halfL * sinf(imuFiltered.pitch);
        //         float zR = halfW * sinf(imuFiltered.roll);

        //         float vzP = halfL * imuFiltered.Vpitch; // sin(x) = x
        //         float vzR = halfW * imuFiltered.Vroll;  // sin(x) = x

        //         LidarData_t lidar = peek(&q_0);
        //         // map values to amt for servos to move
        //         int servo_setting_num;
        //         if (q_0.counter == 0)
        //         {
        //             servo_setting_num = mapLidarToServo(q_0.data[MAX_QUEUE_SIZE - 1].lidar_reading);
        //         }
        //         else
        //         {
        //             servo_setting_num = mapLidarToServo(q_0.data[q_0.counter - 1].lidar_reading);
        //         }

        //         // create suspension data to send to control loop
        //         // one for front wheel and one for back wheel (of this side)
        //         suspensionData_t suspensionDataFront = {(ServoID)0, -300, 0, servo_setting_num};
        //         suspensionData_t suspensionDataBack = {(ServoID)2, -500, 0, servo_setting_num};

        //         // call feedback function to send this lidar data over
        //         // suspensionFeedback(&suspensionDataFront, dt, zP - zR, imuFiltered.Vz + vzP - vzR, lidar);
        //         // suspensionFeedback(&suspensionDataBack, dt, -zP - zR, imuFiltered.Vz - vzP - vzR, lidar);
        //     }
        //     else if (variance1 > TOLERANCE) // if the left wheels need to make adjustments
        //     {
        //         // get the current dt since a suspension update was last called
        //         uint64_t nowMicros = to_us_since_boot(get_absolute_time());
        //         float dt = (nowMicros - lastMicros) / 1000000.0; // Convert to milliseconds
        //         lastMicros = nowMicros;

        //         // get imu data
        //         imuFiltered_t imuFiltered;
        //         imuGetFiltered(&imuFiltered);

        //         float zP = halfL * sinf(imuFiltered.pitch);
        //         float zR = halfW * sinf(imuFiltered.roll);

        //         float vzP = halfL * imuFiltered.Vpitch; // sin(x) = x
        //         float vzR = halfW * imuFiltered.Vroll;  // sin(x) = x

        //         // create suspension data to send to control loop
        //         // one for front wheel and one for back wheel (of this side)
        //         LidarData_t lidar = peek(&q_1);
        //         // map values to amt for servos to move
        //         int servo_setting_num;
        //         if (q_1.counter == 0)
        //         {
        //             servo_setting_num = mapLidarToServo(q_1.data[MAX_QUEUE_SIZE - 1].lidar_reading);
        //         }
        //         else
        //         {
        //             servo_setting_num = mapLidarToServo(q_1.data[q_1.counter - 1].lidar_reading);
        //         }

        //         suspensionData_t suspensionDataFront = {(ServoID)1, -300, 0, servo_setting_num};
        //         suspensionData_t suspensionDataBack = {(ServoID)3, -500, 0, servo_setting_num};

        //         // call feedback function to send this lidar data over
        //         // suspensionFeedback(&suspensionDataFront, dt, zP + zR, imuFiltered.Vz + vzP + vzR, lidar);
        //         // suspensionFeedback(&suspensionDataBack, dt, -zP + zR, imuFiltered.Vz - vzP + vzR, lidar);
        //     }
        // }
    }
}