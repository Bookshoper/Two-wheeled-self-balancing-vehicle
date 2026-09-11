#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;
    float ax, ay, az;
    float gx, gy, gz;
    bool valid;
};

void initMPU(uint8_t sda, uint8_t scl);
bool readIMU(IMUData &imu);
void calibrateGyro();
void updateAttitude(IMUData &imu, float dt, float &pitch, float &roll);

extern float gyroBiasX, gyroBiasY, gyroBiasZ;

#endif