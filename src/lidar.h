#ifndef LIDAR_H
#define LIDAR_H

void lidarSetup();

/**
 * Struct to hold data relevant to a point in time when LiDAR data is collected.
 *
 * Fields:
 *  lidar_reading: integer reading returned from LiDAR, representing distance to nearest object at an angle.
 *  x_position: total x distance covered by the car, returned from the encoder.
 *  servo_setting: setting that the servos should move to, as determined from solely the LiDAR data (w/o feedback loop)
 */
typedef struct
{
    int lidar_reading;
    int x_position;
} LidarData;

#endif
