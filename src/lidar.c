#include "lidar.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "log.h"
#include "encoder.h"
#include "imu.h"


#define LIDAR0_UART uart0
#define LIDAR0_TX_PIN 16
#define LIDAR0_RX_PIN 17

#define LIDAR1_UART uart1
#define LIDAR1_TX_PIN 20
#define LIDAR1_RX_PIN 21

#define LIDAR_BAUDRATE 115200

#define LIDAR_PERIOD_MS 10

#define LIDAR_THETA 30 * M_PI / 180.0 // base angle of lidar, used for normalization
#define LIDAR_HEIGHT 284                         // millimeters

static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;

static volatile uint8_t lidar0Buffer[9];
static volatile uint8_t lidar0Index = 0;
static volatile bool lidar0NewData = false;

float lidar0Filtered = 0;

static volatile uint8_t lidar1Buffer[9];
static volatile uint8_t lidar1Index = 0;
static volatile bool lidar1NewData = true;

float lidar1Filtered = 0;

static float timeConstantToDecayFactor(float timeConstant) { // Time constant in seconds
    return expf(-LIDAR_PERIOD_MS/(1000 * timeConstant));
}

float lidarHighpass;


LidarQueue_t qR;
LidarQueue_t qL;

static void enqueue(LidarQueue_t *q, float z, float x) {
    // put data into queue
    q->data[q->writeIndex].z = z;
    q->data[q->writeIndex].x = x;
    // increment counter
    q->writeIndex++;
    if (q->writeIndex >= LIDAR_QUEUE_LEN) { q->writeIndex = 0; }
}


// Forward declarations
static void lidarTaskFunc(void *);
static void uart0RxISR(void);
static void uart1RxISR(void);


LidarQueue_t *lidarGetQueueR() { return &qR; }
LidarQueue_t *lidarGetQueueL() { return &qL; }

void lidarSetK(float highpassTau) {
    lidarHighpass = timeConstantToDecayFactor(highpassTau);
}

void lidarGetMostRecent(LidarData_t *dataR, LidarData_t *dataL) {
    *dataR = qR.data[qR.writeIndex > 0 ? qR.writeIndex - 1 : LIDAR_QUEUE_LEN];
    *dataL = qL.data[qL.writeIndex > 0 ? qL.writeIndex - 1 : LIDAR_QUEUE_LEN];
}

void lidarInstSetup(uart_inst_t *uartInst, uint txPin, uint rxPin, void(*isrFunc)(void) )
{
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
    lidarHighpass = timeConstantToDecayFactor(1.0);

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


static void lidarTaskFunc(void *)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        // get position from encoder - this is total distance the car has covered
        float encoderPosition, encoderSpeed;
        encoderRead(&encoderPosition, &encoderSpeed);

        imuFiltered_t imuFiltered;
        imuGetFiltered(&imuFiltered);
        float cosP = cosf(imuFiltered.pitch - LIDAR_THETA);
        float sinP = sinf(imuFiltered.pitch - LIDAR_THETA);

        if (lidar0NewData) {
            uint16_t dist_mm = (lidar0Buffer[3] << 8) | lidar0Buffer[2];
            uint16_t strength = (lidar0Buffer[5] << 8) | lidar0Buffer[4];
            float temp_c = ((lidar0Buffer[7] << 8) | lidar0Buffer[6]) / 8.0f - 256;

            if (dist_mm == 65535 || strength < 100)
            {
                log_printf(LOG_WARN, "LIDAR0 returned invalid or weak signal (Dist=%u, Strength=%u)", dist_mm, strength);
            }
            else
            {
                // log_printf(LOG_INFO, "LIDAR0: %3d mm | Strength: %5u | Temp: %.2f°C", dist_mm, strength, temp_c);
            }

            float x = encoderPosition + (0.001 * dist_mm*cosP);
            float z = 0.001 * (LIDAR_HEIGHT + dist_mm*sinP);
            
            enqueue(&qR, z - lidar0Filtered, x);

            lidar0Filtered = lidarHighpass*lidar0Filtered + (1-lidarHighpass)*z;


            irq_set_enabled(UART0_IRQ, true);
            lidar0NewData = false;
        }

        if (lidar1NewData) {
            uint16_t dist_mm = (lidar1Buffer[3] << 8) | lidar1Buffer[2];
            uint16_t strength = (lidar1Buffer[5] << 8) | lidar1Buffer[4];
            float temp_c = ((lidar1Buffer[7] << 8) | lidar1Buffer[6]) / 8.0f - 256;

            if (dist_mm == 65535 || strength < 100)
            {
                log_printf(LOG_WARN, "LIDAR1 returned invalid or weak signal (Dist=%u, Strength=%u)", dist_mm, strength);
            }
            else
            {
                // log_printf(LOG_INFO, "LIDAR1: %3d mm | Strength: %5u | Temp: %.2f°C", dist_mm, strength, temp_c);
            }

            float x = encoderPosition + (0.001 * dist_mm*cosP);
            float z = 0.001 * (LIDAR_HEIGHT + dist_mm*sinP);
            
            enqueue(&qL, z - lidar1Filtered, x);

            lidar1Filtered = lidarHighpass*lidar1Filtered + (1-lidarHighpass)*z;


            irq_set_enabled(UART1_IRQ, true);
            lidar1NewData = false;
        }
    }
}
