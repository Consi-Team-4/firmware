#include "lidar.h"
#include "math.h"
#include "servo.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "time.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

static volatile uint8_t lidar0Buffer[9];
static volatile uint8_t lidar0Index = 0;

static volatile uint8_t lidar1Buffer[9];
static volatile uint8_t lidar1Index = 0;

static int maxQueueSize = 1000;

static double tolerance = 5;

static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;

static void uart0RxISR(void);
static void uart1RxISR(void);
static void lidarTaskFunc(void *);
typedef struct
{
    int data[1000];
    int front;
    int rear;
    int size;
    int max;
} Queue;

Queue q_0;
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
    q->front = (q->front + 1) % maxQueueSize; // q->max;
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

    q->rear = (q->rear + 1) % maxQueueSize; // q->max
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

double calculate_queue_variance(Queue *q, int size, int numItems)
{
    double sum = 0, mean, variance = 0;

    // Calculate the mean
    for (int i = 0; i < size; i++)
    {
        sum += q->data[i];
    }
    mean = sum / size;

    // if the queue is smaller than numItems
    int n;
    if (numItems > size)
    {
        n = size;
    }
    else
    {
        n = numItems;
    }

    // Calculate the variance
    for (int i = size - 1; i < size - n; i--)
    {
        variance += pow(q->data[i] - mean, 2);
    }
    variance /= size;

    return variance;
}

void lidarSetup()
{

    printf("lidar setup start\n");
    // Set up queue for lidar distance data
    initQueue(&q_0);
    sleep_ms(100);
    initQueue(&q_1);
    sleep_ms(100);
    // Set up uart0 for lidar 0
    uart_init(uart0, 115200);
    gpio_set_function(16, UART_FUNCSEL_NUM(uart0, 16)); // GPIO16/D4 is Uart TX
    gpio_set_function(17, UART_FUNCSEL_NUM(uart0, 17)); // GPIO17/D5 is Uart RX

    sleep_ms(100);

    // Set uart settings
    uart_set_hw_flow(uart0, false, false);          // No CTS or RTS
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE); // 8 data bits, 1 stop bit, no parity bit
    uart_set_fifo_enabled(uart0, false);            // No FIFO - want interrupt to trigger as soon as we get a character

    sleep_ms(100);

    // Set up interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart0RxISR);
    irq_set_enabled(UART0_IRQ, true);

    sleep_ms(100);

    uart_set_irq_enables(uart0, true, false); // Trigger interrupt when it receives data

    lidar0Index = 0; // Reset

    sleep_ms(100);


    //  **** second lidar ****
    uart_init(uart1, 115200);
    gpio_set_function(18, UART_FUNCSEL_NUM(uart1, 18)); // GPIO18/D6 is Uart TX
    gpio_set_function(19, UART_FUNCSEL_NUM(uart1, 19)); // GPIO19/D7 is Uart RX

    sleep_ms(100);

    // Set uart settings
    uart_set_hw_flow(uart1, false, false);          // No CTS or RTS
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE); // 8 data bits, 1 stop bit, no parity bit
    uart_set_fifo_enabled(uart1, false);            // No FIFO - want interrupt to trigger as soon as we get a character

    sleep_ms(100);

    // Set up interrupt
    irq_set_exclusive_handler(UART1_IRQ, uart1RxISR);
    irq_set_enabled(UART1_IRQ, true);

    sleep_ms(100);

    uart_set_irq_enables(uart1, true, false); // Trigger interrupt when it receives data

    lidar1Index = 0; // Reset

    sleep_ms(100);

    lidarTask = xTaskCreateStatic(lidarTaskFunc, "lidarTask", sizeof(lidarStackBuffer) / sizeof(StackType_t), NULL, 4, lidarStackBuffer, &lidarTaskBuffer);
    printf("lidar setup complete\n");

    sleep_ms(1000);
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

static void uart1RxISR(void)
{
    BaseType_t higherPriorityTaskWoken = 0;

    while (uart_is_readable(uart1) && lidar1Index < 9)
    {
        uint8_t rx_char = uart_getc(uart1);
        lidar1Buffer[lidar1Index] = rx_char;

        if (lidar1Index < 2 && rx_char != 0x59)
        {                    // Invalid frame header
            lidar1Index = 0; // Reset
            continue;
        }
        else if (lidar1Index == 8)
        { // Checksum byte
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 8; i++)
            {
                checksum += lidar1Buffer[i];
            }
            if (rx_char != checksum)
            {                    // Bad checksum
                lidar1Index = 0; // Reset
                continue;
            }
            else
            {
                irq_set_enabled(UART1_IRQ, false);                           // Turn off ISR so data doesn't get overwritten
                vTaskNotifyGiveFromISR(lidarTask, &higherPriorityTaskWoken); // Wake main task
            }
        }
        lidar1Index++;
    }

    portYIELD_FROM_ISR(&higherPriorityTaskWoken);
}

int mapDistanceToAngle(int distance)
{
    return (distance - 0) * (2800 - 200) / (1000 - 0);
}

static void lidarTaskFunc(void *)
{
    printf("got to task\n");
    // Wait for notification from ISR
    while (true)
    {
        // if (ulTaskNotifyTake(true, portMAX_DELAY))
        // {
            printf("got inside the if\n");
            // For now, just printing values
            uint16_t distance0 = ((uint16_t *)lidar0Buffer)[1];
            uint16_t strength0 = ((uint16_t *)lidar0Buffer)[2];
            float temperature0 = ((uint16_t *)lidar0Buffer)[3] / 8.0 - 256;

            uint16_t distance1 = ((uint16_t *)lidar1Buffer)[1];
            uint16_t strength1 = ((uint16_t *)lidar1Buffer)[2];
            float temperature1 = ((uint16_t *)lidar1Buffer)[3] / 8.0 - 256;
            // Put value for distance into queue
            enqueue(&q_0, distance0);
            enqueue(&q_1, distance1);

            uint16_t variance0 = calculate_queue_variance(&q_0, queueSize(&q_0), 10);
            uint16_t variance1 = calculate_queue_variance(&q_1, queueSize(&q_1), 10);

            printf("% 5ucm\t% 5u\t% 7.3fC\t% 5u\n", distance0, strength0, temperature0, variance0);
            printf("% 5ucm\t% 5u\t% 7.3fC\t% 5u\n", distance1, strength1, temperature1, variance1);

            if (variance0 > tolerance || variance1 > tolerance)
            {
                double delta = mapDistanceToAngle(distance0); // value to hold amount that the servo should move
                                                              // calculate servo distance, set delta to this amt
                // adjustServoPosition(delta, distance0);        // call servo function, pass in servo setting
            }

            if (variance0 > tolerance || variance1 > tolerance)
            {
                double delta = mapDistanceToAngle(distance1); // value to hold amount that the servo should move
                                                              // calculate servo distance, set delta to this amt
                // adjustServoPosition(delta, distance1);        // call servo function, pass in servo setting
            }

            // Reset
            lidar0Index = 0;
            while (uart_is_readable(uart0))
            {
                uart_getc(uart0);
            } // Empty queue to clear old data
            irq_set_enabled(UART0_IRQ, true);

            lidar1Index = 0;
            while (uart_is_readable(uart1))
            {
                uart_getc(uart1);
            } // Empty queue to clear old data
            irq_set_enabled(UART1_IRQ, true);
        // }
    }
}