#include "mpu6050.h"

#define MPU_ADDR 0x68
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

const float ACC_SCALE = 16384.0f;
const float GYRO_SCALE = 65.5f;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

static bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1) return 0;
    return Wire.read();
}

static bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, length) != length) return false; // 已修复分号问题
    for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
    return true;
}

void initMPU(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl, 400000);
    delay(100);
    readRegister(REG_WHO_AM_I);
    writeRegister(REG_PWR_MGMT_1, 0x00);
    delay(100);
    writeRegister(REG_SMPLRT_DIV, 4);
    writeRegister(REG_CONFIG, 0x03);
    writeRegister(REG_GYRO_CONFIG, 0x08);
    writeRegister(REG_ACCEL_CONFIG, 0x00);
    delay(100);
}

bool readIMU(IMUData &imu) {
    uint8_t buffer[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, 14)) {
        imu.valid = false;
        return false;
    }

    imu.accX  = ((int16_t)buffer[0] << 8) | buffer[1];
    imu.accY  = ((int16_t)buffer[2] << 8) | buffer[3];
    imu.accZ  = ((int16_t)buffer[4] << 8) | buffer[5];
    imu.gyroX = ((int16_t)buffer[8] << 8) | buffer[9]; // 已修复 int1.16t 错误
    imu.gyroY = ((int16_t)buffer[10] << 8) | buffer[11];
    imu.gyroZ = ((int16_t)buffer[12] << 8) | buffer[13];

    imu.ax = (float)imu.accX / ACC_SCALE;
    imu.ay = (float)imu.accY / ACC_SCALE;
    imu.az = (float)imu.accZ / ACC_SCALE;

    imu.gx = (float)imu.gyroX / GYRO_SCALE;
    imu.gy = (float)imu.gyroY / GYRO_SCALE;
    imu.gz = (float)imu.gyroZ / GYRO_SCALE;

    imu.valid = !(imu.gyroX == -32768 || imu.gyroX == 32767);
    return imu.valid;
}

void calibrateGyro() {
    float sumX = 0, sumY = 0, sumZ = 0;
    int validSamples = 0;
    IMUData imu;
    for (int i = 0; i < 500; i++) {
        if (readIMU(imu)) {
            sumX += imu.gx; sumY += imu.gy; sumZ += imu.gz;
            validSamples++;
        }
        delay(5);
    }
    if (validSamples > 0) {
        gyroBiasX = sumX / validSamples;
        gyroBiasY = sumY / validSamples;
        gyroBiasZ = sumZ / validSamples;
    }
}

void updateAttitude(IMUData &imu, float dt, float &pitch, float &roll) {
    float accPitch = atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az)) * 180.0f / M_PI;
    float accRoll  = atan2(imu.ay, imu.az) * 180.0f / M_PI;

    float gx = imu.gx - gyroBiasX;
    float gy = imu.gy - gyroBiasY;

    const float alpha = 0.98f;
    pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * accPitch;
    roll  = alpha * (roll + gx * dt) + (1.0f - alpha) * accRoll;
}