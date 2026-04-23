#ifndef PINS_H
#define PINS_H

// Motor pins
#define MOTOR1_IN1 12
#define MOTOR1_IN2 13
#define MOTOR1_PWM 14
#define MOTOR2_IN1 27
#define MOTOR2_IN2 14
#define MOTOR2_PWM 12
#define MOTOR3_IN1 15
#define MOTOR3_IN2 2
#define MOTOR3_PWM 0
#define MOTOR4_IN1 4
#define MOTOR4_IN2 16
#define MOTOR4_PWM 17

// Encoder pins
#define ENC1_A 25
#define ENC1_B 26
#define ENC2_A 32
#define ENC2_B 33
#define ENC3_A 34
#define ENC3_B 35
#define ENC4_A 36
#define ENC4_B 39

// Servo pins
#define SERVO1_PIN 18
#define SERVO2_PIN 19

// Pump pin
#define PUMP_PIN 21

// PWM channels
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3
#define PWM_CH_PUMP 4

// PWM frequency and resolution
#define PWM_FREQ 1000
#define PWM_RES 8

// Mecanum wheel parameters (in meters, adjust as needed)
#define WHEEL_RADIUS 0.05
#define WHEELBASE_LENGTH 0.1
#define WHEELBASE_WIDTH 0.1
#define MAX_SPEED 255  // Max PWM value

#endif