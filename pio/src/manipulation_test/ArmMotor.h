#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

class ArmMotor {
public:
    ArmMotor(float kp, float ki, float kd,
             int motor_in1, int motor_in2,
             int encoder_a, int encoder_b, int encoder_x,
             int pwm_ch_fwd, int pwm_ch_rev);

    void setup();
    void update();
    void setTarget(long ticks);
    void returnToZero();
    long getEncoderCount() { return encoder.getCount(); }

    static void staticHandleInterrupt();
    static ArmMotor* instance;

private:
    float kp, ki, kd;
    int pin_in1, pin_in2;
    int pin_enc_a, pin_enc_b, pin_enc_x;   // <-- x back
    int pwm_ch_fwd, pwm_ch_rev;

    long  target_ticks = 0;
    float prev_error   = 0.0f;
    float integral     = 0.0f;

    static constexpr float DT           = 1.0f / 500.0f;
    static constexpr float INTEGRAL_MAX = 80.0f;
    static constexpr float DEADBAND     = 8.0f;
    static constexpr int   PWM_MAX      = 255;
    static constexpr int   PWM_MIN      = 220;

    ESP32Encoder encoder;

    void applyPWM(int pwm);
    void runPID();
    void onIndexPulse();
};