#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include "mpu6050.h"
#include "motor_encoder.h"

// 1. 引入 Dabble 蓝牙库配置
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

struct PID {
    float Kp, Ki, Kd;
};

// PID 参数
PID anglePID = {-20.0f,   0.0f, -0.1f}; //100  0.8
PID speedPID = { -0.3f, -0.01f,  0.0f}; // 0.32   0.02
PID turnPID  = {   1.2f,   0.0f,  0.5f}; 

// 提速参数配置
const float MAX_TARGET_SPEED = 60.0f; 
const float MAX_TARGET_TURN  = 22.0f; 

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
float calcSpeedPID(int32_t speedLeftAcc, int32_t speedRightAcc) {
    float currentSpeed = (speedLeftAcc + speedRightAcc) * 0.5f;
    float speedError = currentSpeed - targetSpeed;

    static float speedFilter = 0;
    speedFilter = speedFilter * 0.7f + speedError * 0.3f;

    speedIntegral += speedFilter * 0.04f;   
    speedIntegral = constrain(speedIntegral, -400, 400); 

    float out = speedPID.Kp * speedFilter + speedPID.Ki * speedIntegral;
    out = constrain(out, -10.0f, 10.0f); 

    return out;
}

float calcTurnPID(int16_t leftEnc, int16_t rightEnc, float gyroZ) {
    return turnPID.Kp * targetTurn + turnPID.Kd * (gyroZ - gyroBiasZ);
}

void handleDabbleJoystick() {
    Dabble.processInput(); 

    static uint32_t lastBleCheck = 0;
    if (millis() - lastBleCheck > 2000) {
        lastBleCheck = millis();
        BLEDevice::startAdvertising(); 
    }

    int yAxis = GamePad.gety_axis(); 
    int xAxis = GamePad.getx_axis(); 

    targetSpeed = ((float)yAxis / 7.0f) * MAX_TARGET_SPEED;
    targetTurn  = -((float)xAxis / 7.0f) * MAX_TARGET_TURN;
}

void setup() {
    Serial.begin(115200);

    initMPU(21, 22);
    calibrateGyro();
    initMotorsAndEncoders();

    Dabble.begin("Balance_Car");

    // ESP32 硬件定时器 (5ms = 5000us)
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);

    Serial.println("Balance Robot Ready. Debugging active...");
}

void loop() {
    handleDabbleJoystick();

    if (controlFlag) {
        controlFlag = false;

        if (readIMU(imu)) {
            updateAttitude(imu, 0.005f, currentPitch, currentRoll);
        }

        int16_t leftEnc = 0, rightEnc = 0;
        getEncoderSpeed(leftEnc, rightEnc); 

        // 跌倒保护
        if (abs(currentPitch) > 30.0f) {
            setMotorSpeed(0, 0);
            speedIntegral = 0;
            targetSpeed = 0.0f;
            targetTurn = 0.0f;
            return;
        }

        // 速度环降频处理 (40ms)
        static int speedLoopCounter = 0;
        static int32_t speedLeftAcc = 0;   
        static int32_t speedRightAcc = 0;
        static float speedOut = 0.0f;      

        speedLeftAcc += leftEnc;
        speedRightAcc += rightEnc;
        speedLoopCounter++;

        if (speedLoopCounter >= 8) { 
            speedOut = calcSpeedPID(speedLeftAcc, speedRightAcc); 
            
            speedLeftAcc = 0;
            speedRightAcc = 0;
            speedLoopCounter = 0;
        }

        targetAngle = -1.0f - speedOut; 

        // 直立环和转向环 (200Hz)
        float angleOut = calcAnglePID(currentPitch, imu.gy);
        float turnOut  = calcTurnPID(leftEnc, rightEnc, imu.gz);

        int leftPWM  = (int)(angleOut + turnOut);
        int rightPWM = (int)(angleOut - turnOut);

        leftPWM  = constrain(leftPWM, -255, 255);
        rightPWM = constrain(rightPWM, -255, 255);

        setMotorSpeed(leftPWM, rightPWM);

        // ==========================================
        // 串口调试数据打印 (降频至 200ms 打印一次)
        // ==========================================
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 200) {
            lastPrint = millis();
            Serial.printf("Pitch: %.2fdeg | TargetAng: %.2f | AngleOut: %.2f | PWM: %d\n",
                          currentPitch, targetAngle, angleOut, leftPWM);
        }
    }
}