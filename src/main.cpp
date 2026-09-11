#include <Arduino.h>
#include "mpu6050.h"
#include "motor_encoder.h"

struct PID {
    float Kp, Ki, Kd;
};

PID anglePID = {-120.0f, 0.0f, -1.0f};  //-400*0.6*0.5=-120.0       -2*0.6*0.5=-0.6
PID speedPID = {-8.0f, -0.04f, 0.0f}; 
PID turnPID  = {1.5f, 0.0f, 0.1f};   

float targetAngle = 0.0f; 
float targetSpeed = 0.0f;  
float targetTurn  = 0.0f;  

float currentPitch = 0.0f, currentRoll = 0.0f;
IMUData imu;
hw_timer_t *timer = NULL;
volatile bool controlFlag = false;

void IRAM_ATTR onTimer() {
    controlFlag = true;
}

float calcAnglePID(float pitch, float gyroY) {
    float error = pitch - targetAngle;
    return anglePID.Kp * error + anglePID.Kd * (gyroY - gyroBiasY);
}

float speedIntegral = 0;
float calcSpeedPID(int16_t leftEnc, int16_t rightEnc) {
    float currentSpeed = (leftEnc + rightEnc) * 0.5f;
    float speedError = currentSpeed - targetSpeed;
    
    static float speedFilter = 0;
    speedFilter = speedFilter * 0.7f + speedError * 0.3f;

    speedIntegral += speedFilter;
    speedIntegral = constrain(speedIntegral, -3000, 3000);

    return speedPID.Kp * speedFilter + speedPID.Ki * speedIntegral;
}

float calcTurnPID(int16_t leftEnc, int16_t rightEnc, float gyroZ) {
    return turnPID.Kp * targetTurn + turnPID.Kd * (gyroZ - gyroBiasZ);
}

void setup() {
    Serial.begin(115200);

    initMPU(21, 22);
    calibrateGyro();
    initMotorsAndEncoders();

    // ESP32 2.x 硬件定时器 API（1MHz 分频 = 80，5ms = 5000us）
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);

    Serial.println("PlatformIO System Ready.");
}

void loop() {
    if (controlFlag) {
        controlFlag = false;

        if (readIMU(imu)) {
            updateAttitude(imu, 0.005f, currentPitch, currentRoll);
        }

        int16_t leftEnc = 0, rightEnc = 0;
        getEncoderSpeed(leftEnc, rightEnc);

        // 跌倒保护
        if (abs(currentPitch) > 40.0f) {
            setMotorSpeed(0, 0);
            speedIntegral = 0;
            return;
        }

        // 串级 PID
        float speedOut = calcSpeedPID(leftEnc, rightEnc);
        targetAngle = -0.5f + speedOut; 

        float angleOut = calcAnglePID(currentPitch, imu.gy);
        float turnOut  = calcTurnPID(leftEnc, rightEnc, imu.gz);

        int leftPWM  = (int)(angleOut + turnOut);
        int rightPWM = (int)(angleOut - turnOut);

        setMotorSpeed(leftPWM, rightPWM);
    }
}