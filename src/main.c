#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Module headers – each module should have its own initialization routine and task function.
#include "imu.h"                    // IMU sensor (e.g., LSM6DSOX)
#include "lidar.h"                  // LIDAR driver
#include "encoder.h"                // Hall effect sensor/encoder
#include "speed_controller.h"       // Speed control (PID for ESC)
#include "suspension_controller.h"  // Suspension control (servo adjustments)
#include "wifi_task.h"              // Wi-Fi communication task
#include "debug_task.h"             // Telemetry/debug task

// Main entry point
int main(void) {
    // Initialize standard I/O for USB/Serial debugging.
    stdio_init_all();
    
    // Optional: Allow time for the USB serial port to open.
    sleep_ms(1000);

    // --- Initialize hardware and sensor modules ---
    // Setup IMU (this configures I2C, interrupts, DMA, etc.)
    imuSetup();
    
    // Setup LIDAR (configure I2C or SPI communication as needed)
    lidarSetup();
    
    // Setup encoder/hall sensor (for speed measurement)
    encoderSetup();
    
    // Initialize speed controller (configure PID parameters, etc.)
    speedControllerInit();
    
    // Initialize suspension controller (load servo calibration, etc.)
    suspensionControllerInit();
    
    // Initialize Wi-Fi communications (set up Wi-Fi module and any sockets)
    wifiSetup();
    
    // --- Create FreeRTOS Tasks ---
    // Create the IMU task: reads sensor data and updates shared data (priority 4)
    xTaskCreate(imuTaskFunc, "IMU Task", 1024, NULL, 4, NULL);
    
    // Create the speed control task: processes encoder data and controls the ESC (priority 5)
    xTaskCreate(speedControlTask, "Speed Task", 1024, NULL, 5, NULL);
    
    // Create the suspension control task: adjusts servos based on IMU and sensor inputs (priority 6)
    xTaskCreate(suspensionControlTask, "Suspension Task", 1024, NULL, 6, NULL);
    
    // Create the Wi-Fi task: handles incoming commands and updates target parameters (priority 3)
    xTaskCreate(wifiTask, "WiFi Task", 1024, NULL, 3, NULL);
    
    // Create the LIDAR task: polls LIDAR for obstacle data and signals suspension adjustments (priority 4)
    xTaskCreate(lidarTask, "Lidar Task", 1024, NULL, 4, NULL);
    
    // Create a debug/telemetry task: sends status data over USB/serial (priority 1)
    xTaskCreate(debugTask, "Debug Task", 1024, NULL, 1, NULL);
    
    // --- Start the FreeRTOS scheduler ---
    vTaskStartScheduler();

    // If the scheduler returns, loop forever.
    while (1) {
    }

    return 0;
}