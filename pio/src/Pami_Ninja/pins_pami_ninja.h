#ifndef PAMI_NINJA_SCRIPTED_PINS_H
#define PAMI_NINJA_SCRIPTED_PINS_H

// Motor pins
#define MOTOR_FL_IN1  21
#define MOTOR_FL_IN2  19
#define MOTOR_FR_IN1  17
#define MOTOR_FR_IN2  16
#define MOTOR_RL_IN1  22
#define MOTOR_RL_IN2  23
#define MOTOR_RR_IN1  5
#define MOTOR_RR_IN2  18

// Encoder pins
#define ENC_FL_A  36
#define ENC_FL_B  39
#define ENC_FR_A  27
#define ENC_FR_B  14
#define ENC_RL_A  34
#define ENC_RL_B  35
#define ENC_RR_A  25
#define ENC_RR_B  26

// Servo pins
#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 12

// Pump and solenoid pins
#define PUMP_PIN 4
#define SOLENOID_PIN 2

// Ultrasonic obstacle sensor pins
#define OBSTACLE_TRIG_PIN 32
#define OBSTACLE_ECHO_PIN 33

// PWM channels
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3
#define PWM_CH_PUMP 4

// Servo/action constants
#define ARM_DOWN_ANGLE   20
#define ARM_UP_ANGLE     130
#define ARM_RELEASE_DOWN_ANGLE 35
#define ARM_MOVE_DELAY   500
#define ARM_SWEEP_STEP   2
#define ARM_SWEEP_DELAY  15

#define MAX_PWM          255
#define STICK_DEADZONE   10

#define PWM_FREQ 1000
#define PWM_RES 8

// LEDC PWM channel assignments
#define CH_FL_IN1  2
#define CH_FL_IN2  3
#define CH_FR_IN1  4
#define CH_FR_IN2  5
#define CH_RL_IN1  6
#define CH_RL_IN2  7
#define CH_RR_IN1  8
#define CH_RR_IN2  9

// Mecanum wheel parameters
#define WHEEL_RADIUS 0.06
#define WHEELBASE_LENGTH 0.1
#define WHEELBASE_WIDTH 0.1
#define MAX_SPEED 255

// Rear-facing ultrasonic obstacle avoidance
#ifndef PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
#define PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE 0
#endif

#ifndef PAMI_OBSTACLE_STOP_DISTANCE_CM
#define PAMI_OBSTACLE_STOP_DISTANCE_CM 50.0f
#endif

#ifndef PAMI_ULTRASONIC_TIMEOUT_US
#define PAMI_ULTRASONIC_TIMEOUT_US 25000UL
#endif

#ifndef PAMI_REAR_AVOIDANCE_FORWARD_PWM
#define PAMI_REAR_AVOIDANCE_FORWARD_PWM 100
#endif

#endif
