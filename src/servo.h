#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// === Initialization ===
void servoSetup(void);

// === Individual control ===
void setServo1(int pwm_us);
void setServo2(int pwm_us);
void setServo3(int pwm_us);
void setServo4(int pwm_us);

// === LIDAR-based coordination (optional) ===
void adjustServoPosition(double delta, int lidarReading);

#endif