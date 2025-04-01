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
    float z;
    float x;
} LidarData_t;


#define MAX_QUEUE_SIZE 2048
typedef struct
{
    LidarData_t data[MAX_QUEUE_SIZE];
    // int front;
    // int rear;
    // int size;
    int counter;
    int numPoints;
} LidarQueue_t;

LidarQueue_t *lidarGetQueue0();
LidarQueue_t *lidarGetQueue1();

#endif
