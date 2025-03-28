#include "console.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "servo.h"
#include "controller.h"



static StaticTask_t consoleTaskBuffer;
static StackType_t consoleStackBuffer[1000];
TaskHandle_t consoleTask;


void consoleTaskFunc(void *);

void consoleSetup() {
    consoleTask = xTaskCreateStatic(consoleTaskFunc, "console", sizeof(consoleStackBuffer)/sizeof(StackType_t), NULL, 1, consoleStackBuffer, &consoleTaskBuffer);
}

void consoleTaskFunc(void *) {
    char input[32];

    while (true) {
        int i = 0;

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
            // Set a servo.
            // S <servoNum> <servoVal>
            // <servoNum> should be from 0 to SERVO_COUNT-1
            // <servoVal> should be from -1000 to 1000
            case 'S':
            {
                char *next;
                char *start = input+1;

                int servoNum = strtol(start, &next, 10);
                if (next == start || servoNum < 0 || servoNum >= SERVO_COUNT) { // Conversion failed
                    printf ("Invalid servo number\n");
                    break;
                }
                start = next;

                int servoVal = strtol(next, NULL, 10);

                
                if (servoNum = ESC) { escEnable(false); } // Disable if it's the ESC
                servoWrite(servoNum, servoVal);

                printf("Writing %d to servo number %u\n", servoVal, servoNum);
                break;
            }
            // Set KP and KI for the ESC. 
            // K <KP> <KI> <Power>
            // KP is set to KP*10^power
            // KI is set to KI*10^power
            case 'K':
            {
                char *next;
                char *start = input+1;

                int intKP = strtol(start, &next, 10);
                if (next == start) { // Conversion failed
                    printf ("Invalid KP\n");
                    break;
                }
                start = next;

                
                int intKI = strtol(start, &next, 10);
                if (next == start) { // Conversion failed
                    printf ("Invalid KI\n");
                    break;
                }
                start = next;

                int power = strtol(start, &next, 10);
                if (next == start) { // Conversion failed
                    printf ("Invalid power\n");
                    break;
                }
                start = next;

                float KP = intKP * powf(10.0, power);
                float KI = intKI * powf(10.0, power);
                escSetK(KP, KI);

                printf("Setting KP=%f, KI=%f\n", KP, KI);
                break;
            }
            // Enable/Disable ESC feedback
            // F <enable>
            // <enable> should be 0 for false, 1 for true
            case 'F':
            {
                int enable = strtol(input+1, NULL, 10);

                escEnable(enable);

                if (enable) { printf("Enabling feedback\n"); }
                else { printf("Disabling feedback\n"); }
                break;
            }
            // Set ESC setpoint
            // T <speed*1000>
            // Sets setpoint to <speed*1000> / 1000
            case 'T':
            {
                int speed_1000 = strtol(input+1, NULL, 10);
                escSetSetpoint(speed_1000/1000.0);

                printf("Setting setpoint to %7.3f\n", speed_1000/1000.0);
                break;
            }
        }
    }
}
