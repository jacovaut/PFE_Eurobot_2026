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
    bool isAtTarget()      { return reached_target; }
    void resetEncoder() { encoder.setCount(0); }
    void disable() { disabled = true; applyPWM(0); }
    void enable()  {
        target_ticks   = encoder.getCount();
        integral       = 0.0f;
        prev_error     = 0.0f;
        reached_target = false;
        disabled       = false;
    }


    static void staticHandleInterrupt();
    static ArmMotor* instance;

private:
    float kp, ki, kd;
    int pin_in1, pin_in2;
    int pin_enc_a, pin_enc_b, pin_enc_x;
    int pwm_ch_fwd, pwm_ch_rev;

    long  target_ticks       = 0;
    float prev_error         = 0.0f;
    float integral           = 0.0f;
    bool  reached_target     = false;
    bool     disabled        = false;
    bool     return_zero_pending = false;
    uint32_t return_zero_timer   = 0;

    static constexpr float DT           = 1.0f / 500.0f;
    static constexpr float INTEGRAL_MAX = 80.0f;
    static constexpr float DEADBAND     = 8.0f;
    static constexpr int   PWM_MAX      = 255;
    static constexpr int   PWM_MIN      = 200;

    ESP32Encoder encoder;

    void applyPWM(int pwm);
    void runPID();
    void onIndexPulse();
};