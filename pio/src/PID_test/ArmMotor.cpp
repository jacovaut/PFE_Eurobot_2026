#include "ArmMotor.h"

// Static member initialization
ArmMotor* ArmMotor::instance = nullptr;

void ArmMotor::staticHandleInterrupt()
{
    if (ArmMotor::instance != nullptr)
        ArmMotor::instance->handleEncounterReset();
}

ArmMotor::ArmMotor(float kp, float ki, float kd,
                   int motor_in1, int motor_in2,
                   int encoder_a, int encoder_b, int encoder_x)
    : kp(kp), ki(ki), kd(kd),
      motor_in1(motor_in1), motor_in2(motor_in2),
      encoder_a(encoder_a), encoder_b(encoder_b), encoder_x(encoder_x)
{
}

void ArmMotor::setup()
{
    delay(200);

    // Configure LEDC for motor PWM
    const int frequency = 5000;  // 5 kHz PWM frequency
    const int resolution = 8;    // 8-bit resolution (0-255)
    const int channel_in1 = 0;
    const int channel_in2 = 1;

    ledcSetup(channel_in1, frequency, resolution);
    ledcSetup(channel_in2, frequency, resolution);
    ledcAttachPin(motor_in1, channel_in1);
    ledcAttachPin(motor_in2, channel_in2);

    pinMode(encoder_a, INPUT);
    pinMode(encoder_b, INPUT);
    pinMode(encoder_x, INPUT_PULLUP);

    encoder.attachHalfQuad(encoder_a, encoder_b);
    ArmMotor::instance = this;
}

void ArmMotor::handleEncounterReset()
{
    long offset = (long)((HOME_ANGLE_DEG / 360.0f) * COUNTS_PER_REV);
    encoder.setCount(offset);
    if (!homed)
        homed = true;
}

void ArmMotor::home()
{
    while (!digitalRead(encoder_x)) // Wait until X index is detected
    {
        motorWrite(180);
        delay(10);
    }
    handleEncounterReset();
    motorWrite(0);
}

float ArmMotor::pulseToAngle(long counts)
{
    float angle = counts / COUNTS_PER_REV * 360.0f;
    return angle;
}

void ArmMotor::motorWrite(int pwm)
{
    const int channel_in1 = 0;
    const int channel_in2 = 1;

    if (pwm > 0)
    {
        ledcWrite(channel_in1, abs(pwm));
        ledcWrite(channel_in2, 0);
    }
    else if (pwm < 0)
    {
        ledcWrite(channel_in1, 0);
        ledcWrite(channel_in2, abs(pwm));
    }
    else
    {
        ledcWrite(channel_in1, 1);
        ledcWrite(channel_in2, 1);
    }
}

void ArmMotor::updatePID()
{
    if (!homed)
        return;

    float current_angle = pulseToAngle(encoder.getCount());

    // Error calculation
    float err = target_angle - current_angle;

    // PID calculation
    integrale += err * DT;
    if (integrale > INTEGRALE_MAX)
        integrale = INTEGRALE_MAX;
    if (integrale < -INTEGRALE_MAX)
        integrale = -INTEGRALE_MAX;

    float d_error = (err - prev_error) / DT;
    float pid = err * kp + integrale * ki + d_error * kd;

    final_output = (int)pid;

    // Stiction compensation
    if (final_output > 0 && final_output < (int)pwm_min)
        final_output = (int)pwm_min;
    else if (final_output < 0 && final_output > -(int)pwm_min)
        final_output = -(int)pwm_min;

    // Output limits
    if (final_output > 255)
        final_output = 255;
    else if (final_output < -255)
        final_output = -255;

    // Deadband: stop jitter at rest
    if (abs(err) < 5.0f)
    {
        final_output = 0;
        integrale = 0.0f;
    }

    motorWrite(-1 * final_output);

    last_angle = current_angle;
    prev_error = err;
}

void ArmMotor::update()
{
    updatePID();
}

void ArmMotor::setTarget(float angle_deg)
{
    target_angle = angle_deg;
}

bool ArmMotor::targetReached()
{
    float current_angle = pulseToAngle(encoder.getCount());

    if (abs(target_angle - current_angle) < 5.0f)
        return true;
    return false;
}