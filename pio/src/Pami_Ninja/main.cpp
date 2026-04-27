#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "pins.h"

// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// Servo objects
Servo servo1;
Servo servo2;

// Track motor speeds
int motorSpeeds[4] = {0, 0, 0, 0};

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

  // Pump
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // PWM setup

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);

  // Attach default PWM pins (IN1 side)
  ledcAttachPin(MOTOR_FL_IN1, PWM_CH1);
  ledcAttachPin(MOTOR_FR_IN1, PWM_CH2);
  ledcAttachPin(MOTOR_RL_IN1, PWM_CH3);
  ledcAttachPin(MOTOR_RR_IN1, PWM_CH4);

  // Encoders
  encoder1.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  encoder2.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  encoder3.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  encoder4.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  // Servos
  servo1.attach(SERVO_LEFT_PIN);
  servo2.attach(SERVO_RIGHT_PIN);

  Serial.println("Robot initialized");
}

void setMotor(int motor, int speed) {
  int in1, in2, channel;

  switch (motor) {
    case 1:
      in1 = MOTOR_FL_IN1; in2 = MOTOR_FL_IN2; channel = PWM_CH1; break;
    case 2:
      in1 = MOTOR_FR_IN1; in2 = MOTOR_FR_IN2; channel = PWM_CH2; break;
    case 3:
      in1 = MOTOR_RL_IN1; in2 = MOTOR_RL_IN2; channel = PWM_CH3; break;
    case 4:
      in1 = MOTOR_RR_IN1; in2 = MOTOR_RR_IN2; channel = PWM_CH4; break;
    default:
      return;
  }

  motorSpeeds[motor - 1] = speed;
  int pwm = abs(speed);

  if (speed > 0) {
    // Forward
    ledcDetachPin(in2);
    ledcAttachPin(in1, channel);
    digitalWrite(in2, LOW);
    ledcWrite(channel, pwm);

  } else if (speed < 0) {
    // Reverse
    ledcDetachPin(in1);
    ledcAttachPin(in2, channel);
    digitalWrite(in1, LOW);
    ledcWrite(channel, pwm);

  } else {
    // Stop
    ledcWrite(channel, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setServo(int servo, int angle) {
  if (servo == 1) {
    servo1.write(angle);
  } else if (servo == 2) {
    servo2.write(angle);
  }
}

void setPump(bool state) {
  digitalWrite(PUMP_PIN, state ? HIGH : LOW);
}

void setMecanumSpeeds(float vx, float vy, float omega) {
  float L = WHEELBASE_LENGTH / 2.0;
  float W = WHEELBASE_WIDTH / 2.0;
  float R = WHEEL_RADIUS;

  float w1 = (vx - vy - (L + W) * omega) / R;
  float w2 = (vx + vy + (L + W) * omega) / R;
  float w3 = (vx + vy - (L + W) * omega) / R;
  float w4 = (vx - vy + (L + W) * omega) / R;

  float scale = MAX_SPEED / 1.0;

  int pwm1 = constrain((int)(w1 * scale), -MAX_SPEED, MAX_SPEED);
  int pwm2 = constrain((int)(w2 * scale), -MAX_SPEED, MAX_SPEED);
  int pwm3 = constrain((int)(w3 * scale), -MAX_SPEED, MAX_SPEED);
  int pwm4 = constrain((int)(w4 * scale), -MAX_SPEED, MAX_SPEED);

  setMotor(1, pwm1);
  setMotor(2, pwm2);
  setMotor(3, pwm3);
  setMotor(4, pwm4);
}


void loop() {
  // Print encoder values only when moving
  if (motorSpeeds[0] != 0 || motorSpeeds[1] != 0 || motorSpeeds[2] != 0 || motorSpeeds[3] != 0) {
    Serial.print("Encoders: ");
    Serial.print(encoder1.getCount());
    Serial.print(", ");
    Serial.print(encoder2.getCount());
    Serial.print(", ");
    Serial.print(encoder3.getCount());
    Serial.print(", ");
    Serial.println(encoder4.getCount());
  }

  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("f ")) {
      // Forward: f <speed>
      int speed = command.substring(2).toInt();
      setMecanumSpeeds(speed / 255.0, 0, 0);
      Serial.print("Moving forward at speed ");
      Serial.println(speed);
    } else if (command.startsWith("b ")) {
      // Backward: b <speed>
      int speed = command.substring(2).toInt();
      setMecanumSpeeds(-speed / 255.0, 0, 0);
      Serial.print("Moving backward at speed ");
      Serial.println(speed);
    } else if (command.startsWith("l ")) {
      // Left strafe: l <speed>
      int speed = command.substring(2).toInt();
      setMecanumSpeeds(0, speed / 255.0, 0);
      Serial.print("Strafing left at speed ");
      Serial.println(speed);
    } else if (command.startsWith("r ")) {
      // Right strafe: r <speed>
      int speed = command.substring(2).toInt();
      setMecanumSpeeds(0, -speed / 255.0, 0);
      Serial.print("Strafing right at speed ");
      Serial.println(speed);
    } else if (command.startsWith("cw ")) {
      // Clockwise rotation: cw <speed>
      int speed = command.substring(3).toInt();
      setMecanumSpeeds(0, 0, speed / 255.0);
      Serial.print("Rotating clockwise at speed ");
      Serial.println(speed);

    } else if (command.startsWith("ccw ")) {
      // Counter-clockwise rotation: ccw <speed>
      int speed = command.substring(4).toInt();
      setMecanumSpeeds(0, 0, -speed / 255.0);
      Serial.print("Rotating counter-clockwise at speed ");
      Serial.println(speed);

    } else if (command.startsWith("stop")) {
      // Stop all motors
      setMecanumSpeeds(0, 0, 0);
      Serial.println("Stopped");

    } else if (command.startsWith("s ")) {
      // Servo command: s <angle>
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int angle = command.substring(spaceIndex + 1).toInt();
        setServo(1, angle);
        setServo(2, angle);
        Serial.print("Set servo 1 and 2 to angle ");
        Serial.println(angle);
      }
    } else if (command.startsWith("p ")) {
      // Pump command: p on or p off
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        String state = command.substring(spaceIndex + 1);
        if (state == "on") {
          setPump(true);
          Serial.println("Pump ON");
        } else if (state == "off") {
          setPump(false);
          Serial.println("Pump OFF");
        } else {
          Serial.println("Pump: use 'p on' or 'p off'");
        }
      }
    } else if (command == "pick") {
      // Pick up block: lower servo, activate pump, wait, then lift
      Serial.println("Picking up block...");
      setServo(1, ARM_DOWN_ANGLE);
      setServo(2, ARM_DOWN_ANGLE);
      delay(ARM_MOVE_DELAY);  // Wait for servo to reach position
      setPump(true);
      Serial.println("Pump activated");
      delay(500);  // Wait 1 second for pump to compress
      setServo(1, ARM_UP_ANGLE);
      setServo(2, ARM_UP_ANGLE);
      Serial.println("Block picked up");
    }
    
    else if (command == "release") {
      // Release block: lower servo, deactivate pump, wait, then lift
      Serial.println("Releasing block...");
      setServo(1, ARM_DOWN_ANGLE);
      setServo(2, ARM_DOWN_ANGLE);
      delay(ARM_MOVE_DELAY);  // Wait for servo to reach position
      setPump(false);
      Serial.println("Pump deactivated");
      delay(500);  // Wait 1 second for block to release
      setServo(1, ARM_UP_ANGLE);
      setServo(2, ARM_UP_ANGLE);
      Serial.println("Block released");
    } else {
      Serial.println("Unknown command. Use: f/b/l/r/cw/ccw <speed>, stop, s <angle>, p on/off, pickup, release");
    }
  }

  delay(100);
}

