#include "lidar.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define LIDAR_UART      uart0
#define LIDAR_TX_PIN    16
#define LIDAR_RX_PIN    17
#define LIDAR_BAUDRATE  115200

static StaticTask_t lidarTaskBuffer;
static StackType_t lidarStackBuffer[1000];
static TaskHandle_t lidarTask;

static volatile uint8_t lidarBuffer[9];
static volatile uint8_t lidarIndex = 0;

// Forward declarations
static void lidarTaskFunc(void *);
static void uart0RxISR(void);

void lidarSetup() {
    uart_init(LIDAR_UART, LIDAR_BAUDRATE);
    gpio_set_function(LIDAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(LIDAR_UART, false, false);
    uart_set_format(LIDAR_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LIDAR_UART, false);

    irq_set_exclusive_handler(UART0_IRQ, uart0RxISR);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(LIDAR_UART, true, false);  // RX interrupt only

    lidarTask = xTaskCreateStatic(
        lidarTaskFunc, "lidarTask",
        sizeof(lidarStackBuffer) / sizeof(StackType_t),
        NULL, 4,
        lidarStackBuffer, &lidarTaskBuffer
    );

    log_printf(LOG_INFO, "TFmini-S LIDAR initialized on UART0 (GPIO %d/%d)", LIDAR_TX_PIN, LIDAR_RX_PIN);
}

static void uart0RxISR(void) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(LIDAR_UART)) {
        uint8_t byte = uart_getc(LIDAR_UART);

        // Sync to 0x59 0x59 frame headers
        if (lidarIndex == 0 && byte != 0x59) continue;
        if (lidarIndex == 1 && byte != 0x59) {
            lidarIndex = 0;
            continue;
        }

        lidarBuffer[lidarIndex++] = byte;

        if (lidarIndex >= 9) {
            // Validate checksum
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) checksum += lidarBuffer[i];

            if (checksum == lidarBuffer[8]) {
                vTaskNotifyGiveFromISR(lidarTask, &higherPriorityTaskWoken);
            } else {
                log_printf(LOG_WARN, "LIDAR checksum mismatch. Dropping frame.");
            }

            lidarIndex = 0;
        }
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static void lidarTaskFunc(void *) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t dist_cm = (lidarBuffer[3] << 8) | lidarBuffer[2];
        uint16_t strength = (lidarBuffer[5] << 8) | lidarBuffer[4];
        float temp_c = ((lidarBuffer[7] << 8) | lidarBuffer[6]) / 8.0f - 256;

        if (dist_cm == 65535 || strength < 100) {
            log_printf(LOG_WARN, "LIDAR returned invalid or weak signal (Dist=%u, Strength=%u)", dist_cm, strength);
        } else {
            log_printf(LOG_INFO, "LIDAR: %3d cm | Strength: %5u | Temp: %.2f°C", dist_cm, strength, temp_c);
        }
    }
}