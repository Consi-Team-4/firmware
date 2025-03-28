#include "lidar.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "servo.h"
#include <math.h>

#define LIDAR0_UART uart0
#define LIDAR0_TX_PIN 16
#define LIDAR0_RX_PIN 17
#define LIDAR1_UART uart1
#define LIDAR1_TX_PIN 20
#define LIDAR1_RX_PIN 21

#define LIDAR_BAUDRATE 115200
#define MAX_QUEUE_SIZE 10

static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;

static volatile uint8_t lidar0Buffer[9];
static volatile uint8_t lidar0Index = 0;

static volatile uint8_t lidar1Buffer[9];
static volatile uint8_t lidar1Index = 0;

// Forward declarations
static void lidarTaskFunc(void *);
static void uart0RxISR(void);
static void uart1RxISR(void);

#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    uint16_t data[MAX_QUEUE_SIZE];
    // int front;
    // int rear;
    // int size;
    int counter;
} Queue;

Queue q_0;
Queue q_1;

// Function to initialize the queue
void initializeQueue(Queue *q)
{
    // q->front = 0;
    // q->rear = 0;
    q->counter = 0;
}

// Function to check if the queue is empty
// bool isEmpty(Queue *q) { return (q->front == q->rear); }

// Function to check if the queue is full
bool isFull(Queue *q) { return (q->counter == MAX_QUEUE_SIZE); }

// Function to add an element to the queue (Enqueue
// operation)
void enqueue(Queue *q, int value)
{
    if (isFull(q))
    {
        q->counter = 0;
    }
    q->data[q->counter] = value;
    q->counter++;
}

// Function to print the current queue
void printQueue(Queue *q)
{
    printf("Current Queue: ");
    int count = q->counter;
    for (int i = count; i < MAX_QUEUE_SIZE; i++)
    {
        printf("%d ", q->data[i]); // everything after/including the counter
    }
    for (int i = 0; i < count; i++)
    {
        printf("%d ", q->data[i]); // everything before the counter
    }
    printf("\n");
}
/**
 * Prints out all of the items in the queue
 */

double calculate_queue_variance(Queue *q)
{
    // int variance = 0;
    // int mean = 0;
    // int sum = 0;

    // // Calculate the mean of the last `n` elements
    // for (int i = 0; i < MAX_QUEUE_SIZE; i++)
    // {
    //     sum += q->data[i];
    // }
    // mean = sum / MAX_QUEUE_SIZE;

    // // Calculate the variance
    // for (int i = 0; i < MAX_QUEUE_SIZE; i++)
    // {
    //     if (q->data[i] != 0)
    //     { // ignore any 0 values when queue is first initialized
    //         variance += pow(q->data[i] - mean, 2);
    //     }
    // }
    // variance /= MAX_QUEUE_SIZE; // Divide by `n`, not `size`

    // return variance;
    int max = 0;
    int min = 1000;
    for (int i = 0; i < MAX_QUEUE_SIZE; i++)
    {
        // printf("data: %d\n", q->data[i]);
        if (q->data[i] < min)
        {
            min = q->data[i];
            printf("new min: %d\n", min);
        }
        if (q->data[i] > max)
        {
            max = q->data[i];
        }
    }
    int maxmin = max - min;
    printf("max: %d, min: %d, max-min: %d\n", max, min, maxmin);

    return maxmin;
}

void lidar0Setup()
{
    // initialize queue for lidar data to be fed into

    initializeQueue(&q_0);

    uart_init(LIDAR0_UART, LIDAR_BAUDRATE);
    gpio_set_function(LIDAR0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIDAR0_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(LIDAR0_UART, false, false);
    uart_set_format(LIDAR0_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LIDAR0_UART, false);

    irq_set_exclusive_handler(UART0_IRQ, uart0RxISR);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(LIDAR0_UART, true, false); // RX interrupt only
}

void lidar1Setup()
{
    // initialize queue for lidar data to be fed into
    initializeQueue(&q_1);

    uart_init(LIDAR1_UART, LIDAR_BAUDRATE);
    gpio_set_function(LIDAR1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIDAR1_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(LIDAR1_UART, false, false);
    uart_set_format(LIDAR1_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LIDAR1_UART, false);

    irq_set_exclusive_handler(UART1_IRQ, uart1RxISR);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(LIDAR1_UART, true, false); // RX interrupt only
}

void lidarSetup()
{
    // set up both of the lidars and the queues to hold their data
    lidar0Setup();
    lidar1Setup();

    lidarTask = xTaskCreateStatic(
        lidarTaskFunc, "lidarTask",
        sizeof(lidarStackBuffer) / sizeof(StackType_t),
        NULL, 4,
        lidarStackBuffer, &lidarTaskBuffer);

    log_printf(LOG_INFO, "TFmini-S LIDAR initialized on UART0 (GPIO %d/%d)", LIDAR0_TX_PIN, LIDAR0_RX_PIN);
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

static void lidarTaskFunc(void *)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t dist_mm0 = (lidar0Buffer[3] << 8) | lidar0Buffer[2];
        uint16_t strength0 = (lidar0Buffer[5] << 8) | lidar0Buffer[4];
        float temp_c0 = ((lidar0Buffer[7] << 8) | lidar0Buffer[6]) / 8.0f - 256;

        uint16_t dist_mm1 = (lidar1Buffer[3] << 8) | lidar1Buffer[2];
        uint16_t strength1 = (lidar1Buffer[5] << 8) | lidar1Buffer[4];
        float temp_c1 = ((lidar1Buffer[7] << 8) | lidar1Buffer[6]) / 8.0f - 256;

        if (dist_mm0 == 65535 || strength0 < 100)
        {
            log_printf(LOG_WARN, "LIDAR0 returned invalid or weak signal (Dist=%u, Strength=%u)", dist_mm0, strength0);
        }
        else
        {
            log_printf(LOG_INFO, "LIDAR0: %3d mm | Strength: %5u | Temp: %.2f°C", dist_mm0, strength0, temp_c0);
        }

        if (dist_mm1 == 65535 || strength1 < 100)
        {
            log_printf(LOG_WARN, "LIDAR1 returned invalid or weak signal (Dist=%u, Strength=%u)", dist_mm1, strength1);
        }
        else
        {
            log_printf(LOG_INFO, "LIDAR1: %3d mm | Strength: %5u | Temp: %.2f°C", dist_mm1, strength1, temp_c1);
        }

        // add this value to the queue of lidar data
        enqueue(&q_0, dist_mm0);
        enqueue(&q_1, dist_mm1);

        // calculate variance of this data
        // double variance0 = sqrt(calculate_queue_variance(&q_0, 10));
        // DifferenceMetrics Diff0 = calculate_difference(&q_0, 10);
        int variance0 = calculate_queue_variance(&q_0);
        int variance1 = calculate_queue_variance(&q_1);

        // double variance1 = sqrt(calculate_queue_variance(&q_1, 10));
        printQueue(&q_0);
        printf("\n");
        printQueue(&q_1);
        log_printf(LOG_INFO, "VARIANCE0: %3d cm, VARIANCE1: %3d cm", variance0, variance1);

        if (dist_mm0 < 10)
        {

            // servoSetAll(500);
        }
        else if (dist_mm0 < 20)
        {
            // servoSetAll(1000);
        }
    }
}