#include "wifi_task.h"
#include <WiFiNINA.h>
#include "config.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

// The wifiSetup() function attempts to connect to WiFi until successful.
void wifiSetup(void) {
    int status = WL_IDLE_STATUS;
    
    Serial.println("WiFi Setup: Attempting to connect...");
    // Attempt to connect until connected
    while (status != WL_CONNECTED) {
        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print("Connecting");
        // Wait 2 seconds between retries (nonblocking using FreeRTOS delay)
        vTaskDelay(pdMS_TO_TICKS(2000));
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Connected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

// An example WiFi task that calls wifiSetup() then waits for incoming UDP packets.
void wifiTask(void *pvParameters) {
    (void)pvParameters; // Unused parameter

    // Ensure WiFi is set up and connected
    wifiSetup();
    
    // If you’re using UDP, you could initialize a WiFiUDP instance here.
    // For example:
    // WiFiUDP Udp;
    // Udp.begin(WIFI_UDP_PORT);
    // Then, in your loop, check for packets:
    // char packetBuffer[256];
    // while (true) {
    //     int packetSize = Udp.parsePacket();
    //     if (packetSize > 0) {
    //         int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    //         if (len > 0) {
    //             packetBuffer[len] = '\0';
    //             Serial.print("Received: ");
    //             Serial.println(packetBuffer);
    //             // Process the command here
    //         }
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
    
    // For this example, we'll just loop and print a heartbeat.
    while (1) {
        Serial.println("WiFi Task Running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}