#ifndef WIFI_TASK_H
#define WIFI_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the WiFi module and connects to the network
void wifiSetup(void);

#ifdef __cplusplus
}
#endif

// Also declare your task if needed:
#ifdef __cplusplus
extern "C" {
#endif
void wifiTask(void *pvParameters);
#ifdef __cplusplus
}
#endif

#endif // WIFI_TASK_H