#ifndef PINS_H
#define PINS_H

// Motor pins
#define MOTOR_FL_IN1  21    // Front-Left
#define MOTOR_FL_IN2  19
#define MOTOR_FR_IN1  17    // Front-Right
#define MOTOR_FR_IN2  16
#define MOTOR_RL_IN1  22   // Rear-Left
#define MOTOR_RL_IN2  23
#define MOTOR_RR_IN1  5    // Rear-Right
#define MOTOR_RR_IN2  18

// Encoder pins
// GPIO 34, 35, 36, 39 are input-only — perfect for encoders
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

// Pump pin
#define PUMP_PIN 4
#define SOLENOID_PIN 2

// PWM channels
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3
#define PWM_CH_PUMP 4

// ============================================================
//  TUNABLE CONSTANTS
// ============================================================

#define ARM_DOWN_ANGLE   20     // Degrees: arms fully lowered
#define ARM_UP_ANGLE     130
#define ARM_RELEASE_DOWN_ANGLE 35
     // Degrees: arms fully raised
#define ARM_MOVE_DELAY   500    // ms to wait for servo to reach position
#define ARM_SWEEP_STEP   2      // Degrees per step for continuous servo sweep
#define ARM_SWEEP_DELAY  15     // ms between sweep steps

#define MAX_PWM          255    // Maximum motor PWM (0-255)
#define STICK_DEADZONE   10     // Ignore stick noise within ±this range

#define PWM_FREQ 1000
#define PWM_RES 8

// ============================================================
//  LEDC PWM CHANNEL ASSIGNMENTS  (ESP32 has 16 channels)
// ============================================================

#define CH_FL_IN1  2
#define CH_FL_IN2  3
#define CH_FR_IN1  4
#define CH_FR_IN2  5
#define CH_RL_IN1  6
#define CH_RL_IN2  7
#define CH_RR_IN1  8
#define CH_RR_IN2  9

// Mecanum wheel parameters (in meters, adjust as needed)
#define WHEEL_RADIUS 0.06
#define WHEELBASE_LENGTH 0.1
#define WHEELBASE_WIDTH 0.1
#define MAX_SPEED 255 // Max PWM value


// Keyboard layout for robot control
#define KEY_FORWARD 'W'
#define KEY_BACKWARD 'S'
#define KEY_LEFT 'A'
#define KEY_RIGHT 'D'
#define KEY_ROTATE_CW 'E'
#define KEY_ROTATE_CCW 'Q'
#define KEY_STOP ' '  // Space bar
#define KEY_PICK 'P'
#define KEY_RELEASE 'R'
#define KEY_SERVO_UP 'U'
#define KEY_SERVO_DOWN 'O'
#define KEY_SERVO_SWEEP 'I'
#define KEY_PUMP_TOGGLE 'J'
#define KEY_ENC_TOGGLE 'N'
#define KEY_ENC_RESET 'L'
#define KEY_SPEED_UP '+'
#define KEY_SPEED_DOWN '-'
#define KEY_OMEGA_SPEED_UP ']'
#define KEY_OMEGA_SPEED_DOWN '['



#endif
