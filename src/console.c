#include "console.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "servo.h"



static StaticTask_t consoleTaskBuffer;
static StackType_t consoleStackBuffer[1000];
TaskHandle_t consoleTask;


void consoleTaskFunc(void *);

void consoleSetup() {
    consoleTask = xTaskCreateStatic(consoleTaskFunc, "console", sizeof(consoleStackBuffer)/sizeof(StackType_t), NULL, 3, consoleStackBuffer, &consoleTaskBuffer);
}

void consoleTaskFunc(void *) {
    char input[32];

    while (true) {
        int i = 0;

        printf("Enter command, value: ");

        // Read one character at a time
        while (true) {
            int c = getchar_timeout_us(0);  // Non-blocking read
            if (c == PICO_ERROR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay
                continue;
            }

            if (c == '\n' || c == '\r') {
                input[i < sizeof(input) ? i : sizeof(input)-1] = '\0';  // Null-terminate
                putchar('\n');
                break;
            }

            if (i < sizeof(input) - 1) {
                input[i] = (char)c;
                i++;
                putchar(c);  // Echo back the input
            }
        }
    
        switch (input[0]) {
            case 'S': // Set a servo. Synatax: S <servoNum> <servoVal>       <servoNum> should be from 0 to SERVO_COUNT-1 and <servoVal> should be from -1000 to 1000
            {
                char *inputContinued;
                int servoNum = strtol(input+1, &inputContinued, 10);
                if (inputContinued == input || servoNum < 0 || servoNum >= SERVO_COUNT) { // Conversion failed
                    printf ("Invalid servo number\n");
                    break;
                }

                int servoVal = strtol(inputContinued, NULL, 10);

                printf("Writing %d to servo number %u\n", servoVal, servoNum);
                servoWrite(servoNum, servoVal);
                break;
            }
        }
    }
}
