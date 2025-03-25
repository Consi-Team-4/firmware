#include "imu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "i2c_dma.h"

#include "constants.h"
#include "LSM6DSOXdefines.h"
#include "debug.h"

// Goal is to get time, accelerometer data, and gyroscope data when triggered by interrupt.
// These can then be passed to the kalman filter via a queue

// For calibration purposes
#define GXOFFS (0)//(33)
#define GYOFFS (0)//(-50)
#define GZOFFS (0)//(-45)
#define AXOFFS (0)
#define AYOFFS (0)
#define AZOFFS (0)

// Got pins from arduino nano rp2040 connect schematic
#define INT1 24
#define I2C_SDA 12
#define I2C_SCL 13
// i2c0 is the i2c connected to pins 12 and 13
#define I2C_INST i2c0

static i2c_dma_t *i2c_dma;

static StaticTask_t imuTaskBuffer;
static StackType_t imuStackBuffer[1000];
static TaskHandle_t imuTask;

// Can't pass 64bit data directly in task notification
// Saving it here so the task can access it
static volatile uint64_t imuIrqMicros;

// Putting a mutex on this so it doesn't get overwritten while another task is reading it
static imuData_t imuData;
static StaticSemaphore_t imuDataMutexBuffer;
static SemaphoreHandle_t imuDataMutex;



// Forward declaring internal functions

// Reading and writing to registers using directly with the SDK (for setup) and using the DMA driver (non-blocking for main program)
static int readRegistersSDK(uint8_t address, uint8_t* data, size_t length);
static int writeRegisterSDK(uint8_t address, uint8_t value);
static int readRegistersDMA(uint8_t regAddress, uint8_t* buf, size_t length);
static int writeRegisterDMA(uint8_t regAddress, uint8_t value);

static void imuDataReadyIrqCallback(void);
static void imuTaskFunc(void *);




void imuSetup() {
    printf("Setting up SDK I2C...\n");

    i2c_init(I2C_INST, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);


    int res;


    uint8_t buf[1];
    res = readRegistersSDK(REG_WHO_AM_I, buf, 1);
    printf("WHO_AM_I:\t%s\tRead:%X\tShould be:%X\n", errorToString(res), buf[0], WHO_AM_I_VAL); 



    printf("Setting up IMU...\n");

    // Copying setup from arduino library with exception of 833Hz instead of 104Hz
    
    // Accelerometer: 833 Hz, +-4g, LPF2 filter
    res = writeRegisterSDK(REG_CTRL1_XL, 0b0110<<4 | 0b10<<2 | 0b1<<1);
    printf("CTRL1_XL:\t%s\n", errorToString(res));
    // Gyroscope: 833 Hz, +-500 dps, not +-125dps
    res = writeRegisterSDK(REG_CTRL2_G,  0b0110<<4 | 0b01<<2 | 0b0<<1);
    printf("CTRL2_G:\t%s\n", errorToString(res));
    // Gyroscope: high performance mode, high pass disabled, 16MHz hp cuttof (irrelevant), OIS enable through SPI2, bypass accelerometer user offset, disable OIS (irrelevant)
    res = writeRegisterSDK(REG_CTRL7_G, 0x00);
    printf("CTRL7_G:\t%s\n", errorToString(res));
    // Accelerometer: bandwidth ODR/4, disable highpass reference mode?, enable fast settling mode, use low pass filter, old full scale mode, low pass to 6d functions
    res = writeRegisterSDK(REG_CTRL8_XL,0b000<<5 | 0b0<<4 | 0b1<<3 | 0b0<<2 | 0b0<<1 | 0b1);
    printf("CTRL8_XL:\t%s\n", errorToString(res));

    // Interrupt on accelerometer or gyroscope data ready
    res = writeRegisterSDK(REG_INT1_CTRL, 0x03);
    printf("INT1_CTRL:\t%s\n", errorToString(res));



    printf("Setting up pico interrupt...\n");

    // Initialize INT1 pin
    gpio_init(INT1);
    gpio_set_dir(INT1, GPIO_IN);

    // Hook up the IRQ
    irq_add_shared_handler(IO_IRQ_BANK0, imuDataReadyIrqCallback, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    // Make the pin trigger the IRQ
    gpio_set_irq_enabled(INT1, GPIO_IRQ_EDGE_RISE, true);

    // Enable IRQ
    irq_set_enabled(IO_IRQ_BANK0, true);


    printf("Creating Task...\n");
    imuTask = xTaskCreateStatic(imuTaskFunc, "imuTask", sizeof(imuStackBuffer)/sizeof(StackType_t), NULL, 4, imuStackBuffer, &imuTaskBuffer);

    printf("Creating Mutex...\n");
    imuDataMutex = xSemaphoreCreateMutexStatic(&imuDataMutexBuffer);

    printf("Initializing I2C DMA driver...\n");
    res = i2c_dma_init(&i2c_dma, I2C_INST, 400*1000, I2C_SDA, I2C_SCL); // Initialize the dma driver for later
    printf("i2c_dma_init:\t%s\n", errorToString(res));
}

void imuGetData(imuData_t *buf) {
    xSemaphoreTake(imuDataMutex, portMAX_DELAY);
    *buf = imuData;
    xSemaphoreGive(imuDataMutex);
}

// Reading and writing to register using the regular i2c functions (needed for setup before the scheduler is started)
static int readRegistersSDK(uint8_t address, uint8_t* data, size_t length) {
    int numWritten = i2c_write_timeout_us(I2C_INST, LSM6DSOX_ADDRESS, &address, 1, true, 1000);
    if (numWritten != 1) { 
        if (numWritten < 0) {
            return numWritten;
        } else { 
            return PICO_ERROR_GENERIC;
        }
    }

    int numRead = i2c_read_timeout_us(I2C_INST, LSM6DSOX_ADDRESS, data, length, false, 1000);
    if (numRead != length) { 
        if (numRead < 0) {
            return numRead;
        } else { 
            return PICO_ERROR_GENERIC;
        }
    }

    return PICO_OK;
}

static int writeRegisterSDK(uint8_t address, uint8_t value) {
    uint8_t buff[2] = {address, value};
    int numWritten = i2c_write_timeout_us(I2C_INST, LSM6DSOX_ADDRESS, buff, 2, false, 1000);
    if (numWritten != 2) { 
        if (numWritten < 0) {
            return numWritten;
        } else { 
            return PICO_ERROR_GENERIC;
        }
    }
    
    return PICO_OK;
}

// Reading and writing to register using the pico-i2c-dma
static int readRegistersDMA(uint8_t regAddress, uint8_t* buf, size_t length) {
  int result = i2c_dma_write_read(i2c_dma, LSM6DSOX_ADDRESS, &regAddress, 1, buf, length);
  return result;
}

static int writeRegisterDMA(uint8_t regAddress, uint8_t value) {
  uint8_t buff[2] = {regAddress, value};
  int result = i2c_dma_write(i2c_dma, LSM6DSOX_ADDRESS, buff, 2);
  return result;
}



static void imuDataReadyIrqCallback(void) {
    BaseType_t higherPriorityTaskWoken = 0;

    // This function will be called on any GPIO interrupt
    // So we start by ignoring it if it's not the one we care about
    bool respond = false;

    if (gpio_get_irq_event_mask(INT1) & GPIO_IRQ_EDGE_RISE) { // Trigger on INT1 rising edge
        gpio_acknowledge_irq(INT1, GPIO_IRQ_EDGE_RISE); // Acknowledge the request since we're responding to it

        // Get time as soon as data is ready
        imuIrqMicros = to_us_since_boot(get_absolute_time());
        
        // Notify imu task so it can handle the I2C
        vTaskNotifyGiveFromISR(imuTask, &higherPriorityTaskWoken);
        

        // On our architecture, safe for this to be called multiple times
        // So perfectly okay to call this in a shared irq handler
        portYIELD_FROM_ISR( &higherPriorityTaskWoken );
    }
    return;
}

static void imuTaskFunc(void *) {
    { // Read once to get the party started
        int16_t data[7];
        readRegistersDMA(REG_OUT_TEMP, (uint8_t*)data, sizeof(data));
    }

    while (true) {
        

        // Wait for notification from ISR
        uint64_t tmpMicros;
        if (ulTaskNotifyTake(true, pdMS_TO_TICKS(200))) { // If it's been more than 5 milliseconds since last read, read anyways, but don't use micros from irq
            tmpMicros = imuIrqMicros; // Save the current time in case it gets overwritten
        } else {
            tmpMicros = to_us_since_boot(get_absolute_time());
        }

        // printf("Reading IMU\n");


        // Reading temperature too for the hell of it
        uint64_t prevMicros = imuData.micros;

        int16_t data[7];
        float temp;

        if (readRegistersDMA(REG_OUT_TEMP, (uint8_t*)data, sizeof(data)) == PICO_OK) {
            temp = data[0] * 256.0 / LSM6DSOX_FSR + 25;
            xSemaphoreTake(imuDataMutex, portMAX_DELAY);
            imuData.micros = tmpMicros;
            imuData.Gx = (data[1] - GXOFFS) * 500.0 / LSM6DSOX_FSR * M_PI / 180;
            imuData.Gy = (data[2] - GYOFFS) * 500.0 / LSM6DSOX_FSR * M_PI / 180;
            imuData.Gz = (data[3] - GZOFFS) * 500.0 / LSM6DSOX_FSR * M_PI / 180;
            imuData.Ax = (data[4] - AXOFFS) * 4.0 / LSM6DSOX_FSR * GRAVITY ;
            imuData.Ay = (data[5] - AYOFFS) * 4.0 / LSM6DSOX_FSR * GRAVITY ;
            imuData.Az = (data[6] - AZOFFS) * 4.0 / LSM6DSOX_FSR * GRAVITY ;
            xSemaphoreGive(imuDataMutex);
        } else {
            temp = NAN;
            xSemaphoreTake(imuDataMutex, portMAX_DELAY);
            imuData.micros = tmpMicros;
            imuData.Gx = NAN;
            imuData.Gy = NAN;
            imuData.Gz = NAN;
            imuData.Ax = NAN;
            imuData.Ay = NAN;
            imuData.Az = NAN;
            xSemaphoreGive(imuDataMutex);
        }

    }
}
