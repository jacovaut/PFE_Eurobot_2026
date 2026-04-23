#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

class ArmMotor
{
public:
    // Constructor
    ArmMotor(float kp, float ki, float kd, 
             int motor_in1, int motor_in2,
             int encoder_a, int encoder_b, int encoder_x);
    
    // Initialization
    void setup();
    void home();
    
    // Control
    void update();
    void setTarget(float angle_deg);
    
    // Static interrupt handler
    static void staticHandleInterrupt();
    static ArmMotor* instance;

private:
    // PID gains
    float kp;
    float ki;
    float kd;

    // Hardware pins
    int motor_in1;
    int motor_in2;
    int encoder_a;
    int encoder_b;
    int encoder_x;
    bool homed;
    
    float last_angle = 0.0f;
    float prev_error = 0.0f;
    float integrale = 0.0f;
    int final_output = 0;
    // Gravity feedforward — simple compensation for weight, tune up/down as needed
    float FF_GAIN = 170;
    float ff = 0;
    float target_angle = 0.0f;

    // Constants
    static constexpr float COUNTS_PER_REV = 1050.0f;
    static constexpr float HOME_ANGLE_DEG = 0.0f;
    static constexpr float LOOP_HZ = 500.0f;
    static constexpr float DT = 1.0f / LOOP_HZ;
    static constexpr float INTEGRALE_MAX = 80.0f; // anti-windup clamp (PWM units)

    // Tuning parameters
    float pwm_min = 170.0f; // stiction compensation threshold (tune)
    float ff_gain = 35;
    
    // Encoder
    ESP32Encoder encoder;
    
    // Helper functions
    float pulseToAngle(long counts);
    void motorWrite(int pwm);
    void updatePID();
    void handleEncounterReset();
};
