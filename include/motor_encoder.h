#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

void initMotorsAndEncoders();
void setMotorSpeed(int leftPwm, int rightPwm);
void getEncoderSpeed(int16_t &leftCount, int16_t &rightCount);

#endif