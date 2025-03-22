#ifndef LIDAR_H
#define LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

// Dummy function to get the LIDAR distance reading (e.g., in centimeters)
int getLidarDistance(void);

// Optionally, you could add an initialization function if needed.
void lidarSetup(void);

#ifdef __cplusplus
}
#endif

#endif // LIDAR_H