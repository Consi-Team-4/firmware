#include "console.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "servo.h"
#include "controller.h"
#include "imu.h"



static StaticTask_t consoleTaskBuffer;
static StackType_t consoleStackBuffer[1000];
TaskHandle_t consoleTask;


void consoleTaskFunc(void *);

void consoleSetup() {
    consoleTask = xTaskCreateStatic(consoleTaskFunc, "console", sizeof(consoleStackBuffer)/sizeof(StackType_t), NULL, 2, consoleStackBuffer, &consoleTaskBuffer);
}

void consoleTaskFunc(void *) {
    char input[64];

    while (true) {
        int i = 0;

        // Read one character at a time
        while (true) {
            int c = getchar_timeout_us(0);  // Non-blocking read
            if (c == PICO_ERROR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(1));  // Small delay
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
        
        consoleRunCommand(&input);
    }
}

void consoleRunCommand(char *input) {
    switch (input[0]) {
        case 'S': { // Suspension commands
            switch(input[1]) {
                case 'K': { // SK - Set suspension feedback parameters
                    // SK <KP> <KI> <KD> <Power> <Tau>
                    // KP is set to KP*10^power
                    // KI is set to KI*10^power
                    // KD is set to KD*10^power
                    // Tau is in ms
                    char *next;
                    char *start = input+2;

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

                    int intKD = strtol(start, &next, 10);
                    if (next == start) { // Conversion failed
                        printf ("Invalid KD\n");
                        break;
                    }
                    start = next;

                    int power = strtol(start, &next, 10);
                    if (next == start) { // Conversion failed
                        printf ("Invalid power\n");
                        break;
                    }
                    start = next;

                    int intTau = strtol(start, &next, 10);
                    if (next == start) { // Conversion failed
                        printf ("Invalid Tau\n");
                        break;
                    }
                    start = next;
                    
                    float e = powf(10.0, power);
                    float KP = intKP * e;
                    float KI = intKI * e;
                    float KD = intKD * e;
                    float Tau = intTau / 1000.0; // Convert from ms to seconds
                    suspensionSetK(KP, KI, KD, Tau);

                    printf("Setting esc KP=%f, KI=%f, KD=%f, Tau=%f\n", KP, KI, KD, Tau);
                    break;
                }

                case 'F': { // SF - Enable/Disable suspension feedback
                    // SF <enable>
                    // <enable> should be 0 for false, 1 for true
                    int enable = strtol(input+2, NULL, 10);

                    suspensionEnable(enable);

                    if (enable) { printf("Enabling suspension feedback\n"); }
                    else { printf("Disabling suspension feedback\n"); }
                    break;
                }

                case 'P': { // SP - Set suspension servo position
                    // SP <servoNum> <servoVal>
                    // <servoNum> should be from 0 to 3
                    // <servoVal> should be between the minimum and maximum for that servo. Software will clip value if out of range to avoid damanging hardware.
                    char *next;
                    char *start = input+2;

                    int servoNum = strtol(start, &next, 10);
                    if (next == start || servoNum < 0 || servoNum >= SERVO_COUNT) { // Conversion failed
                        printf ("Invalid servo number\n");
                        break;
                    }
                    start = next;

                    int servoVal = strtol(next, NULL, 10);

                    servoWrite(servoNum, servoVal);

                    printf("Writing %d to servo number %u\n", servoVal, servoNum);
                    break;
                }

                default: if (input[1]) { printf("Unknown command S%c", input[1]); }
            }
            break;
        }

        case 'E': { // ESC commands
            switch(input[1]) {
                case 'K': { // EK - Set ESC feedback parameters
                    // EK <KP> <KI> <Power>
                    // KP is set to KP*10^power
                    // KI is set to KI*10^power
                    char *next;
                    char *start = input+2;

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
                    
                    float e = powf(10.0, power);
                    float KP = intKP * e;
                    float KI = intKI * e;
                    escSetK(KP, KI);

                    printf("Setting esc KP=%f, KI=%f\n", KP, KI);
                    break;
                }
                
                case 'F': { // EF - Enable/Disable ESC feedback
                    // EF <enable>
                    // <enable> should be 0 for false, 1 for true
                    int enable = strtol(input+2, NULL, 10);

                    escEnable(enable);

                    if (enable) { printf("Enabling esc feedback\n"); }
                    else { printf("Disabling esc feedback\n"); }
                    break;
                }
                
                case 'S': { // ES - Set ESC setpoint
                    // ES <speed>
                    // speed is in mm/s
                    int speed_1000 = strtol(input+2, NULL, 10);
                    escSetSetpoint(speed_1000/1000.0);

                    printf("Setting setpoint to %7.3f\n", speed_1000/1000.0);
                    break;
                }

                case 'P': { // ES - Set ESC power/position
                    // EP <power>
                    // Power should be between -1000 and 1000
                    int power = strtol(input+2, NULL, 10);
                    escEnable(false); // Disable feedback if it's currently active
                    servoWrite(ESC, power);

                    printf("Setting ESC power to %d", power);
                    break;
                }

                default: if (input[1]) { printf("Unknown command E%c", input[1]); }
            }
            break; 
        }

        case 'I': { // IMU commands
            switch(input[1]) {
                case 'K': { // IK - Set IMU filter parameters
                    // IK <AngleTau> <LinearTau> <X>
                    // Tau is in ms
                    // X is how much rotation should be biased towards direction of maximum acceleration vs 0 (between 0 and 1000)
                    char *next;
                    char *start = input+2;

                    int intAngleTau = strtol(start, &next, 10);
                    if (next == start) { // Conversion failed
                        printf ("Invalid Angle Tau\n");
                        break;
                    }
                    start = next;

                    int intLinearTau = strtol(start, &next, 10);
                    if (next == start) { // Conversion failed
                        printf ("Invalid Linear Tau\n");
                        break;
                    }
                    start = next;

                    int intX = strtol(start, NULL, 10);
                    
                    float AngleTau = intAngleTau / 1000.0; // Convert from ms to seconds
                    float LinearTau = intLinearTau / 1000.0; // Convert from ms to seconds
                    float x = intX / 1000.0;
                    imuSetK(AngleTau, LinearTau, x);

                    printf("Setting imu AngleTau=%f, LinearTau=%f, x=%f\n", AngleTau, LinearTau, x);
                    break;
                }

                default: if (input[1]) { printf("Unknown command S%c", input[1]); }
            }
            break;
        }
        
        case 'M': { // Misc
            switch (input[1]) {
                case 'S': { // MS - Set steering servo position
                    // MS <position>
                    // Position should be between -300 and 300
                    int position = strtol(input+2, NULL, 10);
                    servoWrite(SERVO_STEER, position);

                    printf("Setting steering position to %d", position);
                    break;
                }

                default: if (input[1]) { printf("Unknown command M%c", input[1]); }
            }
            break;
        }

        default: if (input[0]) {  printf("Unknown command %c", input[0]); }
    }
}
