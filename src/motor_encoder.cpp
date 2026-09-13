#include "motor_encoder.h"
#include "driver/pcnt.h"

#define MOTOR_L_PWM 18
#define MOTOR_L_IN1 4
#define MOTOR_L_IN2 5
#define ENC_L_A     34
#define ENC_L_B     35

#define MOTOR_R_PWM 19
#define MOTOR_R_IN1 16
#define MOTOR_R_IN2 17
#define ENC_R_A     32
#define ENC_R_B     33

#define MOTOR_STBY  23

static void setupPCNT(pcnt_unit_t unit, int gpioA, int gpioB) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = gpioA,
        .ctrl_gpio_num  = gpioB,
        .lctrl_mode     = PCNT_MODE_REVERSE,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DEC,
        .counter_h_lim  = 32767,
        .counter_l_lim  = -32768,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void initMotorsAndEncoders() {
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);
    pinMode(MOTOR_STBY,  OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);

    // ESP32 2.x LEDC PWM 绑定 (通道 0 和 1, 频率 20kHz, 10 位分辨率)
    ledcSetup(0, 20000, 10);
    ledcAttachPin(MOTOR_L_PWM, 0);

    ledcSetup(1, 20000, 10);
    ledcAttachPin(MOTOR_R_PWM, 1);

    setupPCNT(PCNT_UNIT_0, ENC_L_A, ENC_L_B);
    setupPCNT(PCNT_UNIT_1, ENC_R_A, ENC_R_B);
}

void setMotorSpeed(int leftPwm, int rightPwm) {
    leftPwm  = constrain(leftPwm, -1023, 1023);
    rightPwm = constrain(rightPwm, -1023, 1023);

    // 左电机
    digitalWrite(MOTOR_L_IN1, leftPwm >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_L_IN2, leftPwm >= 0 ? LOW : HIGH);
    ledcWrite(0, abs(leftPwm));

    // 右电机
    digitalWrite(MOTOR_R_IN1, rightPwm >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_IN2, rightPwm >= 0 ? LOW : HIGH);
    ledcWrite(1, abs(rightPwm));
}

void getEncoderSpeed(int16_t &leftCount, int16_t &rightCount) {
    pcnt_get_counter_value(PCNT_UNIT_0, &leftCount);
    pcnt_get_counter_value(PCNT_UNIT_1, &rightCount);
    rightCount = -rightCount;  // 修正极性，使前进时两侧同号
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
}