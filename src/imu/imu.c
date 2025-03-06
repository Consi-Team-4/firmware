#include "imu.h"

#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "i2c_dma.h"

#include <stdio.h>
#include "pico/stdlib.h"

#include "LSM6DSOXdefines.h"
#include "debug.h"

// Goal is to get time, accelerometer data, and gyroscope data when triggered by interrupt.
// These can then be passed to the kalman filter via a queue


// Got pins from arduino nano rp2040 connect schematic
#define INT1 24
#define I2C_SDA 12
#define I2C_SCL 13
// i2c0 is the i2c connected to pins 12 and 13
#define I2C_INST i2c0

static i2c_dma_t *i2c_dma;

static StaticTask_t taskBuffer;
static StackType_t stackBuffer[1000];


// Can't pass 64bit data directly in task notification
// Saving it here so the task can access it
//static volatile uint64_t imuIrqMicros;



// Forward declaring internal functions

// Reading and writing to registers using directly with the SDK (for setup) and using the DMA driver (non-blocking for main program)
static int readRegistersSDK(uint8_t address, uint8_t* data, size_t length);
static int writeRegisterSDK(uint8_t address, uint8_t value);
static int readRegistersDMA(uint8_t regAddress, uint8_t* buf, size_t length);
static int writeRegisterDMA(uint8_t regAddress, uint8_t value);

static void imuDataReadyIrqCallback(void);
static void imuTaskFunc(void *);




void imuSetup () {
    int res;

    printf("Setting up IMU...\n");

    // Absolutely no problem here
    i2c_init(I2C_INST, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // So wtf


    uint8_t buf[1];
    res = readRegistersSDK(REG_WHO_AM_I, buf, 1);
    printf("WHO_AM_I:\t%s\tRead:%X\tShould be:%X\n", errorToString(res), buf[0], WHO_AM_I_VAL); // Tried repeating multiple times, so it's not that the first operation works
    // res = readRegistersSDK(REG_CTRL1_XL, buf, 1);
    // printf("CTRL1_XL:\t%s\tRead:%X\tShould be:%X\n", errorToString(res), buf[0], 0x00); // This register also works fine as long as it's being read and not written to



    // Currently running at 12.5Hz so I can make sure the code is working correctly first
    
    // Copying setup from arduino library with exception of 833Hz instead of 104Hz
    // Accelerometer: 12.5 Hz, +-4g, LPF2 filter
    
    res = writeRegisterSDK(REG_CTRL1_XL, 0x4A);//0b0001<<4 | 0b10<<2 | 0b1<<1);
    printf("CTRL1_XL:\t%s\n", errorToString(res));
    // Gyroscope: 12.5 Hz, +-2000 dps, not +-125dps
    res = writeRegisterSDK(REG_CTRL2_G,  0x4C);//0b0001<<4 | 0b11<<2 | 0b0<<1);
    printf("CTRL2_G:\t%s\n", errorToString(res));
    // Gyroscope: high performance mode, high pass disabled, 16MHz hp cuttof (irrelevant), OIS enable through SPI2, bypass accelerometer user offset, disable OIS (irrelevant)
    res = writeRegisterSDK(REG_CTRL7_G, 0x00);
    printf("CTRL7_G:\t%s\n", errorToString(res));
    // Accelerometer: bandwidth ODR/4, disable highpass reference mode?, enable fast settling mode, use low pass filter, old full scale mode, low pass to 6d functions
    res = writeRegisterSDK(REG_CTRL8_XL, 0x09);//0b000<<5 | 0b0<<4 | 0b1<<3 | 0b0<<2 | 0b0<<1 | 0b1);
    printf("CTRL8_XL:\t%s\n", errorToString(res));

    // Setting up interrupt pin
    // Interrupt on accelerometer or gyroscope data ready
    res = writeRegisterSDK(REG_INT1_CTRL, 0x03);
    printf("INT1_CTRL:\t%s\n", errorToString(res));

    printf("Set up IMU!\n");

    printf("Reading values to test...\n");
    

    int16_t data[7];
    float temp, Gx, Gy, Gz, Ax, Ay, Az;

    if (readRegistersSDK(REG_OUT_TEMP, (uint8_t*)data, sizeof(data)) == PICO_OK) {
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

    printf("% 7.3fC\t% 7.4fgx\t% 7.4fgy\t% 7.4fgz\t% 7.1fdpsx\t% 7.1fdpsy\t% 7.1fdpsz\n", imuIrqMicros, Ax, Ay, Az, Gx, Gy, Gz);
    


    printf("Creating Task and setting up interrupt...\n");
    res = i2c_dma_init(&i2c_dma, I2C_INST, 400*1000, I2C_SDA, I2C_SCL); // Initialize the dma driver for later
    printf("i2c_dma_init:\t%s\n", errorToString(res));


    imuTask = xTaskCreateStatic(imuTaskFunc, "imuTask", sizeof(stackBuffer)/sizeof(StackType_t), NULL, 20, stackBuffer, &taskBuffer);


    // Configure pin, add interrupt
    irq_add_shared_handler(IO_IRQ_BANK0, imuDataReadyIrqCallback, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    gpio_set_dir(INT1, 0); // Set pin as input
    gpio_set_irq_enabled(INT1, GPIO_IRQ_EDGE_RISE, true); // Enable interrupt on INT1 rising edge
    printf("Created task and set up interrupt!\n");
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

static void imuTaskFunc (void *) {
    // Wait for notification from ISR
    while (true) {
        if (ulTaskNotifyTake(true, 50)) {
            printf("Yep\n");
            // For now, just printing values
            // Reading temperature too for the hell of it

            uint64_t micros = imuIrqMicros; // Save the current time in case it gets overwritten

            int16_t data[7];
            float temp, Gx, Gy, Gz, Ax, Ay, Az;

            if (readRegistersDMA(REG_OUT_TEMP, (uint8_t*)data, sizeof(data)) == PICO_OK) {
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

            printf("%10lluus\t% 7.3fC\t% 7.4fgx\t% 7.4fgy\t% 7.4fgz\t% 7.1fdpsx\t% 7.1fdpsy\t% 7.1fdpsz\n", imuIrqMicros, Ax, Ay, Az, Gx, Gy, Gz);
        } else {
            printf("Nope\n");
        }
    }
}
