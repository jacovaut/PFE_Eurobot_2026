#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

class TurnMotor {
public:
    TurnMotor(float kp, float ki, float kd,
              int motor_in1, int motor_in2,
              int encoder_a, int encoder_b,
              int pwm_ch_fwd, int pwm_ch_rev);

    void setup();
    void update();
    void setTarget(long ticks);
    long getEncoderCount() { return encoder.getCount(); }
    bool isAtTarget() { return reached_target; }
    void resetEncoder() { encoder.setCount(0); }
    void disable() { disabled = true;  applyPWM(0); }
    void enable()  { disabled = false; }

private:
    float kp, ki, kd;
    int pin_in1, pin_in2;
    int pin_enc_a, pin_enc_b;
    int pwm_ch_fwd, pwm_ch_rev;

    long  target_ticks    = 0;
    float prev_error      = 0.0f;
    float integral        = 0.0f;
    bool  reached_target  = false;
    bool  disabled        = false;

    static constexpr float DT             = 1.0f / 500.0f;
    static constexpr float INTEGRAL_MAX   = 80.0f;
    static constexpr float DEADBAND       = 2.0f;
    static constexpr int   PWM_MAX        = 255;
    static constexpr int   PWM_MIN        = 180;

    ESP32Encoder encoder;

    void applyPWM(int pwm);
    void runPID();
};