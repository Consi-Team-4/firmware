#include "lidar.h"
#include "math.h"
#include "servo.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

static volatile uint8_t lidar0Buffer[9];
static volatile uint8_t lidar0Index = 0;

static int maxQueueSize = 50;

static double tolerance = 5;

static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;

static void uart0RxISR(void);
static void lidarTaskFunc(void *);
typedef struct
{
    int data[50];
    int front;
    int rear;
    int size;
    int max;
} Queue;

Queue q_1;

/**
 * Initialize queue of fixed length
 */
void initQueue(Queue *q)
{
    q->front = 0;
    q->rear = -1;
    q->size = 0;
    q->max = maxQueueSize;
}

/**
 * Whether the queue is full
 */
int isFull(Queue *q)
{
    return (q->size == maxQueueSize); // q->max
}

/**
 * Return size of queue
 */
int queueSize(Queue *q)
{
    return (q->size);
}

/**
 * Whether the queue is empty
 */
int isEmpty(Queue *q)
{
    return (q->size == 0);
}

/**
 * Removes the first item from the queue.
 */
int dequeue(Queue *q)
{
    if (isEmpty(q))
    {
        printf("Queue is empty\n");
        return -1;
    }

    int item = q->data[q->front];
    q->front = (q->front + 1) % 50; // q->max;
    q->size--;
    return item;
}

/**
 * Adds an integer item to the queue.
 * If the queue is full, this will pop off the last item from the queue
 * before adding the next item, keeping the queue at fixed length.
 */
void enqueue(Queue *q, int item)
{
    if (isFull(q))
    {
        // If full, pop the front element
        dequeue(q);
    }

    q->rear = (q->rear + 1) % 50; // q->max
    q->data[q->rear] = item;
    q->size++;
    // Serial.println(item);
    // Serial.println(q->size);
}

/**
 * Prints out all of the items in the queue
 */
void display(Queue *q)
{
    if (isEmpty(q))
    {
        printf("Queue is empty\n");
        return;
    }
    printf("Queue elements: ");
    for (int i = q->front; i <= q->rear; i++)
    {
        printf("%d ", q->data[i]);
    }
    printf("\n");
}

double calculate_queue_variance(Queue *q, int size)
{
    double sum = 0, mean, variance = 0;

    // Calculate the mean
    for (int i = 0; i < size; i++)
    {
        sum += q->data[i];
    }
    mean = sum / size;

    // Calculate the variance
    for (int i = 0; i < size; i++)
    {
        variance += pow(q->data[i] - mean, 2);
    }
    variance /= size;

    return variance;
}

void lidarSetup()
{

    // Set up queue for lidar distance data
    initQueue(&q_1);
    // Set up uart0 for lidar 0
    uart_init(uart0, 115200);
    gpio_set_function(16, UART_FUNCSEL_NUM(uart0, 16)); // GPIO16/D4 is Uart TX
    gpio_set_function(17, UART_FUNCSEL_NUM(uart0, 17)); // GPIO17/D5 is Uart RX
    // Set uart settings
    uart_set_hw_flow(uart0, false, false);          // No CTS or RTS
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE); // 8 data bits, 1 stop bit, no parity bit
    uart_set_fifo_enabled(uart0, false);            // No FIFO - want interrupt to trigger as soon as we get a character

    // Set up interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart0RxISR);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(uart0, true, false); // Trigger interrupt when it receives data

    lidar0Index = 0; // Reset

    lidarTask = xTaskCreateStatic(lidarTaskFunc, "lidarTask", sizeof(lidarStackBuffer) / sizeof(StackType_t), NULL, 4, lidarStackBuffer, &lidarTaskBuffer);
}

static void uart0RxISR(void)
{
    BaseType_t higherPriorityTaskWoken = 0;

    while (uart_is_readable(uart0) && lidar0Index < 9)
    {
        uint8_t rx_char = uart_getc(uart0);
        lidar0Buffer[lidar0Index] = rx_char;

        if (lidar0Index < 2 && rx_char != 0x59)
        {                    // Invalid frame header
            lidar0Index = 0; // Reset
            continue;
        }
        else if (lidar0Index == 8)
        { // Checksum byte
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 8; i++)
            {
                checksum += lidar0Buffer[i];
            }
            if (rx_char != checksum)
            {                    // Bad checksum
                lidar0Index = 0; // Reset
                continue;
            }
            else
            {
                irq_set_enabled(UART0_IRQ, false);                           // Turn off ISR so data doesn't get overwritten
                vTaskNotifyGiveFromISR(lidarTask, &higherPriorityTaskWoken); // Wake main task
            }
        }
        lidar0Index++;
    }

    portYIELD_FROM_ISR(&higherPriorityTaskWoken);
}

static void lidarTaskFunc(void *)
{
    // Wait for notification from ISR
    while (true)
    {
        if (ulTaskNotifyTake(true, portMAX_DELAY))
        {
            // For now, just printing values
            uint16_t distance = ((uint16_t *)lidar0Buffer)[1];
            uint16_t strength = ((uint16_t *)lidar0Buffer)[2];
            float temperature = ((uint16_t *)lidar0Buffer)[3] / 8.0 - 256;

            // Put value for distance into queue
            enqueue(&q_1, distance);

            uint16_t variance = calculate_queue_variance(&q_1, queueSize(&q_1));
            printf("% 5ucm\t% 5u\t% 7.3fC\t% 5u\n", distance, strength, temperature, variance);

            if (variance > tolerance)
            {
                double delta; // value to hold amount that the servo should move
                // calculate servo distance, set delta to this amt
                adjustServoPosition(delta); // call servo function, pass in servo setting
            }

            // Reset
            lidar0Index = 0;
            while (uart_is_readable(uart0))
            {
                uart_getc(uart0);
            } // Empty queue to clear old data
            irq_set_enabled(UART0_IRQ, true);
        }
    }
}