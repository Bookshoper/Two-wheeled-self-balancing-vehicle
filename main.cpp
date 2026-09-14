#include <Arduino.h>
#include "mpu6050.h"
#include "motor_encoder.h"

struct PID {
    float Kp, Ki, Kd;
};

PID anglePID = {-100.0f,   0.0f, -0.8f};  //-400*0.6*0.5=-120.0       -2*0.6*0.5=-0.6
PID speedPID = { -0.32f, -0.02f,  0.0f}; 
PID turnPID  = {   0.0f,   0.0f, 0.5f};   

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

    speedIntegral += speedFilter * 0.04f;   // 按时间积分
    speedIntegral = constrain(speedIntegral, -2000, 2000); // 可适当放宽

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
        getEncoderSpeed(leftEnc, rightEnc); // 读取 5ms 内的脉冲数

        // 跌倒保护
        if (abs(currentPitch) > 30.0f) {
            setMotorSpeed(0, 0);
            speedIntegral = 0;
            sleep(3); // 延时 1 秒，避免连续打印
            Serial.println("Robot has fallen! Stopping motors.");
            return;
        }

        // =======================
        // 关键修改：速度环降频处理 (8 * 5ms = 40ms)
        // =======================
        static int speedLoopCounter = 0;
        static int32_t speedLeftAcc = 0;   // 用于累加脉冲
        static int32_t speedRightAcc = 0;
        static float speedOut = 0.0f;      // 设为静态变量，保持上次的值

        speedLeftAcc += leftEnc;
        speedRightAcc += rightEnc;
        speedLoopCounter++;

        if (speedLoopCounter >= 8) { // 每 40ms 执行一次速度环计算
            // 此时 speedLeftAcc 和 speedRightAcc 是 40ms 内累加的脉冲数，数值会大很多
            speedOut = calcSpeedPID(speedLeftAcc, speedRightAcc); 
            
            // 清零累加器，准备下一个 40ms
            speedLeftAcc = 0;
            speedRightAcc = 0;
            speedLoopCounter = 0;
        }

        // 将速度环的输出叠加给角度环目标值
        targetAngle = -1.0f - speedOut; 

        // =======================
        // 直立环和转向环依然保持 5ms (200Hz) 高频执行
        // =======================
        float angleOut = calcAnglePID(currentPitch, imu.gy);
        
        // 转向环依然用 5ms 的瞬时脉冲即可
        float turnOut  = calcTurnPID(leftEnc, rightEnc, imu.gz);

        int leftPWM  = (int)(angleOut + turnOut);
        int rightPWM = (int)(angleOut - turnOut);

        setMotorSpeed(leftPWM, rightPWM);

        Serial.print("Pitch:"); 
        Serial.print(currentPitch);
        Serial.print(" LeftPWM:"); 
        Serial.println(leftPWM);

    }
}