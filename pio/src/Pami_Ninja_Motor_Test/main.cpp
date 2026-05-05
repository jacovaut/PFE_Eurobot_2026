#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pins_pami_ninja.h"

// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// Track motor speeds
int motorSpeeds[4] = {0, 0, 0, 0};

// Track motor IN1 and IN2 outputs
int motorIN1[4] = {0, 0, 0, 0};  // PWM value on IN1
int motorIN2[4] = {0, 0, 0, 0};  // PWM value on IN2

// Encoder reading control
bool encoderReadingEnabled = false;

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_RL_IN1, OUTPUT);
  pinMode(MOTOR_RL_IN2, OUTPUT);
  pinMode(MOTOR_RR_IN1, OUTPUT);
  pinMode(MOTOR_RR_IN2, OUTPUT);

  // PWM setup for all 8 motor control pins (IN1 and IN2 for each motor)
  ledcSetup(CH_FL_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FL_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FR_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_FR_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RR_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RR_IN2, PWM_FREQ, PWM_RES);

  // Attach PWM pins for both IN1 and IN2
  ledcAttachPin(MOTOR_FL_IN1, CH_FL_IN1);
  ledcAttachPin(MOTOR_FL_IN2, CH_FL_IN2);
  ledcAttachPin(MOTOR_FR_IN1, CH_FR_IN1);
  ledcAttachPin(MOTOR_FR_IN2, CH_FR_IN2);
  ledcAttachPin(MOTOR_RL_IN1, CH_RL_IN1);
  ledcAttachPin(MOTOR_RL_IN2, CH_RL_IN2);
  ledcAttachPin(MOTOR_RR_IN1, CH_RR_IN1);
  ledcAttachPin(MOTOR_RR_IN2, CH_RR_IN2);

  // Encoders
  encoder1.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  encoder2.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  encoder3.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  encoder4.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  Serial.println("========================================");
  Serial.println("Wheelbase Motor Test - Individual Motor Control");
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  m1 <speed>  - Test motor 1 (Front Left)");
  Serial.println("  m2 <speed>  - Test motor 2 (Front Right)");
  Serial.println("  m3 <speed>  - Test motor 3 (Rear Left)");
  Serial.println("  m4 <speed>  - Test motor 4 (Rear Right)");
  Serial.println("  stop        - Stop all motors");
  Serial.println("  enc on/off  - Enable/disable encoder reading");
  Serial.println("  enc reset   - Reset all encoder counts");
  Serial.println("========================================");
  Serial.println("Speed range: -255 to 255 (negative = reverse)");
  Serial.println("========================================");
  Serial.println("Robot initialized");
}

void setMotor(int motor, int speed) {
  int in1, in2, ch_in1, ch_in2;

  switch (motor) {
    case 1:
      in1 = MOTOR_FL_IN1; in2 = MOTOR_FL_IN2; ch_in1 = CH_FL_IN1; ch_in2 = CH_FL_IN2; break;
    case 2:
      in1 = MOTOR_FR_IN1; in2 = MOTOR_FR_IN2; ch_in1 = CH_FR_IN1; ch_in2 = CH_FR_IN2; break;
    case 3:
      in1 = MOTOR_RL_IN1; in2 = MOTOR_RL_IN2; ch_in1 = CH_RL_IN1; ch_in2 = CH_RL_IN2; break;
    case 4:
      in1 = MOTOR_RR_IN1; in2 = MOTOR_RR_IN2; ch_in1 = CH_RR_IN1; ch_in2 = CH_RR_IN2; break;
    default:
      return;
  }

  motorSpeeds[motor - 1] = speed;
  int pwm = abs(speed);

  if (speed > 0) {
    // Forward: PWM on IN1, LOW on IN2
    Serial.printf("Setting Motor %d forward: IN1=%d, IN2=%d, PWM=%d\n", motor, in1, in2, pwm);
    motorIN1[motor - 1] = pwm;
    motorIN2[motor - 1] = 0;
    ledcWrite(ch_in1, pwm);    // Write PWM to IN1 channel
    ledcWrite(ch_in2, 0);       // Set IN2 to 0

  } else if (speed < 0) {
    // Reverse: PWM on IN2, LOW on IN1
    Serial.printf("Setting Motor %d reverse: IN1=%d, IN2=%d, PWM=%d\n", motor, in1, in2, pwm);
    motorIN1[motor - 1] = 0;
    motorIN2[motor - 1] = pwm;
    ledcWrite(ch_in1, 0);       // Set IN1 to 0
    ledcWrite(ch_in2, pwm);     // Write PWM to IN2 channel

  } else {
    // Stop: PWM to 0, both pins LOW
    Serial.printf("Stopping Motor %d: IN1=%d, IN2=%d\n", motor, in1, in2);
    motorIN1[motor - 1] = 0;
    motorIN2[motor - 1] = 0;
    ledcWrite(ch_in1, 0);       // Set IN1 to 0
    ledcWrite(ch_in2, 0);       // Set IN2 to 0
  }
}

void stopAllMotors() {
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  setMotor(4, 0);
}

void printMotorStatus() {
  Serial.print("Motor Speeds: FL=");
  Serial.print(motorSpeeds[0]);
  Serial.print(", FR=");
  Serial.print(motorSpeeds[1]);
  Serial.print(", RL=");
  Serial.print(motorSpeeds[2]);
  Serial.print(", RR=");
  Serial.println(motorSpeeds[3]);
  
  Serial.print("Motor Outputs (IN1/IN2): FL=");
  Serial.print(motorIN1[0]);
  Serial.print("/");
  Serial.print(motorIN2[0]);
  Serial.print(", FR=");
  Serial.print(motorIN1[1]);
  Serial.print("/");
  Serial.print(motorIN2[1]);
  Serial.print(", RL=");
  Serial.print(motorIN1[2]);
  Serial.print("/");
  Serial.print(motorIN2[2]);
  Serial.print(", RR=");
  Serial.print(motorIN1[3]);
  Serial.print("/");
  Serial.println(motorIN2[3]);
}

void printEncoderStatus() {
  Serial.print("Encoders: FL=");
  Serial.print(encoder1.getCount());
  Serial.print(", FR=");
  Serial.print(encoder2.getCount());
  Serial.print(", RL=");
  Serial.print(encoder3.getCount());
  Serial.print(", RR=");
  Serial.println(encoder4.getCount());
}

void loop() {
  // Print encoder values (if enabled)
  if (encoderReadingEnabled) {
    printEncoderStatus();
  }

  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("m1 ")) {
      // Test motor 1 (Front Left)
      int speed = command.substring(3).toInt();
      speed = constrain(speed, -255, 255);
      setMotor(1, speed);
      Serial.print("Motor 1 (Front Left) set to speed: ");
      Serial.println(speed);
      printMotorStatus();
      
    } else if (command.startsWith("m2 ")) {
      // Test motor 2 (Front Right)
      int speed = command.substring(3).toInt();
      speed = constrain(speed, -255, 255);
      setMotor(2, speed);
      Serial.print("Motor 2 (Front Right) set to speed: ");
      Serial.println(speed);
      printMotorStatus();
      
    } else if (command.startsWith("m3 ")) {
      // Test motor 3 (Rear Left)
      int speed = command.substring(3).toInt();
      speed = constrain(speed, -255, 255);
      setMotor(3, speed);
      Serial.print("Motor 3 (Rear Left) set to speed: ");
      Serial.println(speed);
      printMotorStatus();
      
    } else if (command.startsWith("m4 ")) {
      // Test motor 4 (Rear Right)
      int speed = command.substring(3).toInt();
      speed = constrain(speed, -255, 255);
      setMotor(4, speed);
      Serial.print("Motor 4 (Rear Right) set to speed: ");
      Serial.println(speed);
      printMotorStatus();
      
    } else if (command == "stop") {
      // Stop all motors
      stopAllMotors();
      Serial.println("All motors stopped");
      printMotorStatus();
      
    } else if (command == "enc on") {
      encoderReadingEnabled = true;
      Serial.println("Encoder reading enabled");
      
    } else if (command == "enc off") {
      encoderReadingEnabled = false;
      Serial.println("Encoder reading disabled");
      
    } else if (command == "enc reset") {
      encoder1.clearCount();
      encoder2.clearCount();
      encoder3.clearCount();
      encoder4.clearCount();
      Serial.println("All encoder counts reset");
      printEncoderStatus();
      
    } else if (command == "status") {
      Serial.println("--- Current Status ---");
      printMotorStatus();
      printEncoderStatus();
      Serial.print("Encoder reading: ");
      Serial.println(encoderReadingEnabled ? "ENABLED" : "DISABLED");
      
    } else {
      Serial.println("Unknown command. Use:");
      Serial.println("  m1 <speed>  - Test motor 1 (Front Left)");
      Serial.println("  m2 <speed>  - Test motor 2 (Front Right)");
      Serial.println("  m3 <speed>  - Test motor 3 (Rear Left)");
      Serial.println("  m4 <speed>  - Test motor 4 (Rear Right)");
      Serial.println("  stop        - Stop all motors");
      Serial.println("  enc on/off  - Enable/disable encoder reading");
      Serial.println("  enc reset   - Reset all encoder counts");
      Serial.println("  status      - Print current status");
    }
  }

  delay(100);
}
