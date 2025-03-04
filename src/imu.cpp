#include "imu.h"

#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "i2c_dma.h"

#include "LSM6DSOXdefines.h"
#include "LSM6DSOX.h"

#include <stdio.h>
#include "pico/stdlib.h"

// Goal is to get time, accelerometer data, and gyroscope data when triggered by interrupt.
// These can then be passed to the kalman filter via a queue


// Got pins from arduino nano rp2040 connect schematic
#define INT1 24
#define I2C_SDA 12
#define I2C_SCL 13
// i2c0 is the i2c connected to pins 12 and 13
#define I2C_INST i2c0

static i2c_dma_t *i2c_dma;
static LSM6DSOX imu;

StaticTask_t taskBuffer;
StackType_t stackBuffer[2*1000];
TaskHandle_t imuTask;


// Can't pass 64bit data directly in task notification
// So saving it here so the 
static volatile uint64_t imuMicros;


static void imuDataReadyIrqCallback(void);
static void imuTaskFunc (void *);

void imuSetup () {
    i2c_dma_init(&i2c_dma, I2C_INST, 400*1000, I2C_SDA, I2C_SCL);
    imu = LSM6DSOX(i2c_dma);


    // Currently running at 12.5Hz so I can make sure the code is working correctly first
    
    // Copying setup from arduino library with exception of 833Hz instead of 104Hz
    // Accelerometer: 12.5 Hz, +-4g, LPF2 filter
    imu.writeRegister(REG_CTRL1_XL, 0b0001<<4 | 0b10<<2 | 0b1<<1);
    // Gyroscope: 12.5 Hz, +-2000 dps, not +-125dps
    imu.writeRegister(REG_CTRL2_G,  0b0001<<4 | 0b11<<2 | 0b0<<1);
    // Gyroscope: high performance mode, high pass disabled, 16MHz hp cuttof (irrelevant), OIS enable through SPI2, bypass accelerometer user offset, disable OIS (irrelevant)
    imu.writeRegister(REG_CTRL7_G, 0x00);
    // Accelerometer: bandwidth ODR/4, disable highpass reference mode?, enable fast settling mode, use low pass filter, old full scale mode, low pass to 6d functions
    imu.writeRegister(REG_CTRL8_XL, 0b000<<5 | 0b0<<4 | 0b1<<3 | 0b0<<2 | 0b0<<1 | 0b0);

    // Setting up interrupt pin
    // Interrupt on accelerometer or gyroscope data ready
    imu.writeRegister(REG_INT1_CTRL, 0x03);


    // Initialize task
    // Highest priority
    imuTask = xTaskCreateStatic(imuTaskFunc, "imuTask", sizeof(stackBuffer), NULL, 4, stackBuffer, &taskBuffer);



    // Configure pin, add interrupt
    irq_add_shared_handler(IO_IRQ_BANK0, imuDataReadyIrqCallback, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    gpio_set_dir(INT1, 0); // Set pin as input
    gpio_set_irq_enabled(INT1, GPIO_IRQ_EDGE_RISE, true); // Enable interrupt on INT1 rising edge
}


static void imuDataReadyIrqCallback(void) {
    BaseType_t higherPriorityTaskWoken = 0;

    // This function will be called on any GPIO interrupt
    // So we start by ignoring it if it's not the one we care about
    if (gpio_get_irq_event_mask(INT1) & GPIO_IRQ_EDGE_RISE) { // Trigger on INT1 rising edge
        gpio_acknowledge_irq(INT1, GPIO_IRQ_EDGE_RISE); // Acknowledge the request since we're responding to it

        // Get time as soon as data is ready
        imuMicros = to_us_since_boot(get_absolute_time());
        
        // Notify imu task so it can handle the I2C
        vTaskNotifyGiveFromISR(imuTask, &higherPriorityTaskWoken);
        

        // On our architecture, safe for this to be called multiple times
        // So perfectly okay to call this in a shared irq handler
        portYIELD_FROM_ISR( &higherPriorityTaskWoken );
    }
    return;
}

static void imuTaskFunc (void *) {
    // Wait for notification from ISR
    while (true) {
        if (ulTaskNotifyTake(true, portMAX_DELAY)) {
            // For now, just printing values
            // Reading temperature too for the hell of it

            int16_t data[7];
            float temp, Gx, Gy, Gz, Ax, Ay, Az;

            if (readRegisters(REG_OUT_TEMP, (uint8_t*)data, sizeof(data)) == PICO_OK) {
                temp = data[0] * 256.0 / LSM6DSOX_FSR + 25;
                Gx = data[1] * 2000.0 / LSM6DSOX_FSR;
                Gy = data[2] * 2000.0 / LSM6DSOX_FSR;
                Gz = data[3] * 2000.0 / LSM6DSOX_FSR;
                Ax = data[4] * 4.0 / LSM6DSOX_FSR;
                Ay = data[5] * 4.0 / LSM6DSOX_FSR;
                Az = data[6] * 4.0 / LSM6DSOX_FSR;
            } else {
                temp = NAN;
                Gx = NAN;
                Gy = NAN;
                Gz = NAN;
                Ax = NAN;
                Ay = NAN;
                Az = NAN;
            }

            printf("%10lluuus\t% 7.3fC\t% 7.4fgx\t% 7.4fgy\t% 7.4fgz\t% 7.1fdpsx\t% 7.1fdpsy\t% 7.1fdpsz\n", imuMicros, Ax, Ay, Az, Gx, Gy, Gz);

        }
    }
}
