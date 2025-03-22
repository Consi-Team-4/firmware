#ifndef SUSPENSION_CONTROLLER_H
#define SUSPENSION_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

// Gets the current servo position for a given servo ID.
// Servo IDs: 0 = FRONT_LEFT, 1 = FRONT_RIGHT, 2 = REAR_LEFT, 3 = REAR_RIGHT.
float getServoPosition(int servoID);

#ifdef __cplusplus
}
#endif

#endif // SUSPENSION_CONTROLLER_H