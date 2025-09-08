#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"

typedef struct
{
    float x;
    float y;
    float z;
} Vector3f;

// Nested struct for all sensor measurements
typedef struct
{
    Vector3f accel;
    Vector3f gyro;
    float temperature;
} MpuSensorData;

typedef struct
{
    const char *team;
    const char *device;
    const char *ip;
    const char *ssid;
    const char *sensor;
    MpuSensorData data;
    char timestamp[21];
} MpuDataPacket;

bool init_mpu6050();
void mpu6050_config();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
void update_vibration();

#endif
