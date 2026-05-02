#include "ArmMotor.h"

ArmMotor* ArmMotor::instance = nullptr;

void ArmMotor::staticHandleInterrupt() {
    if (instance) instance->onIndexPulse();
}

void ArmMotor::onIndexPulse() {
    encoder.setCount(0);   // zero the encoder every time X fires
}

ArmMotor::ArmMotor(float kp, float ki, float kd,
                   int motor_in1, int motor_in2,
                   int encoder_a, int encoder_b, int encoder_x,
                   int pwm_ch_fwd, int pwm_ch_rev)
    : kp(kp), ki(ki), kd(kd),
      pin_in1(motor_in1), pin_in2(motor_in2),
      pin_enc_a(encoder_a), pin_enc_b(encoder_b), pin_enc_x(encoder_x),
      pwm_ch_fwd(pwm_ch_fwd), pwm_ch_rev(pwm_ch_rev)
{}

void ArmMotor::setup() {
    ledcSetup(pwm_ch_fwd, 5000, 8);
    ledcSetup(pwm_ch_rev, 5000, 8);
    ledcAttachPin(pin_in1, pwm_ch_fwd);
    ledcAttachPin(pin_in2, pwm_ch_rev);

    encoder.attachHalfQuad(pin_enc_a, pin_enc_b);
    encoder.setCount(0);

    pinMode(pin_enc_x, INPUT_PULLUP);
    instance = this;
    attachInterrupt(digitalPinToInterrupt(pin_enc_x),
                    staticHandleInterrupt, RISING);
}

void ArmMotor::setTarget(long ticks) {
    target_ticks    = ticks;
    integral        = 0.0f;
    prev_error      = 0.0f;
}

void ArmMotor::returnToZero() {
    setTarget(30);
}

void ArmMotor::applyPWM(int pwm) {
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

    if (pwm > 0) {
        if (pwm < PWM_MIN) pwm = PWM_MIN;
        ledcWrite(pwm_ch_fwd, pwm);
        ledcWrite(pwm_ch_rev, 0);
    } else if (pwm < 0) {
        int mag = -pwm;
        if (mag < PWM_MIN) mag = PWM_MIN;
        ledcWrite(pwm_ch_fwd, 0);
        ledcWrite(pwm_ch_rev, mag);
    } else {
        ledcWrite(pwm_ch_fwd, 1);
        ledcWrite(pwm_ch_rev, 1);
    }
}

void ArmMotor::runPID() {
    long  counts = encoder.getCount();
    float error  = (float)(target_ticks - counts);

    if (abs(error) < DEADBAND) {
        integral   = 0.0f;
        prev_error = 0.0f;
        applyPWM(0);
        return;
    }

    integral += error * DT;
    integral  = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    float derivative = (error - prev_error) / DT;
    float output     = kp * error + ki * integral + kd * derivative;

    prev_error = error;
    applyPWM((int)output);
}

void ArmMotor::update() {
    runPID();
}