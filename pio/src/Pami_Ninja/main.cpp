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

// Track motor speeds to know if robot is moving
int motorSpeeds[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_RL_IN1, OUTPUT);
  pinMode(MOTOR_RL_IN2, OUTPUT);
  pinMode(MOTOR_RR_IN1, OUTPUT);
  pinMode(MOTOR_RR_IN2, OUTPUT);

  // Initialize pump pin
  pinMode(PUMP_PIN, OUTPUT);

  // Initialize PWM
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR1_PWM, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR2_PWM, PWM_CH2);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR3_PWM, PWM_CH3);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR4_PWM, PWM_CH4);

  // Initialize pump pin (digital on/off)
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Initialize encoders
  encoder1.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  encoder2.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  encoder3.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  encoder4.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  // Initialize servos
  servo1.attach(SERVO_LEFT_PIN);
  servo2.attach(SERVO_Right_PIN);

  Serial.println("Robot initialized");
}

void setMotor(int motor, int speed) {
  int in1, in2, pwm_ch;
  switch (motor) {
    case 1:
      in1 = MOTOR_FL_IN1;
      in2 = MOTOR_FL_IN2;
      pwm_ch = PWM_CH1;
      break;
    case 2:
      in1 = MOTOR_FR_IN1;
      in2 = MOTOR_FR_IN2;
      pwm_ch = PWM_CH2;
      break;
    case 3:
      in1 = MOTOR_RL_IN1;
      in2 = MOTOR_RL_IN2;
      pwm_ch = PWM_CH3;
      break;
    case 4:
      in1 = MOTOR_RR_IN1;
      in2 = MOTOR_RR_IN2;
      pwm_ch = PWM_CH4;
      break;
    default:
      return;
  }

  // Track motor speed
  motorSpeeds[motor - 1] = speed;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwm_ch, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwm_ch, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwm_ch, 0);
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
  if (state) {
    digitalWrite(PUMP_PIN, HIGH);
  } else {
    digitalWrite(PUMP_PIN, LOW);
  }
}

void setMecanumSpeeds(float vx, float vy, float omega) {
  // Mecanum wheel kinematics
  // Assuming wheel 1: front left, 2: front right, 3: rear right, 4: rear left
  float L = WHEELBASE_LENGTH / 2.0;
  float W = WHEELBASE_WIDTH / 2.0;
  float R = WHEEL_RADIUS;

  // Calculate wheel speeds (m/s)
  float wheel1_speed = (vx - vy - (L + W) * omega) / R;
  float wheel2_speed = (vx + vy + (L + W) * omega) / R;
  float wheel3_speed = (vx + vy - (L + W) * omega) / R;
  float wheel4_speed = (vx - vy + (L + W) * omega) / R;

  // Convert to PWM (assuming max speed corresponds to MAX_SPEED)
  // You may need to tune the scaling factor based on your motors
  float scale = MAX_SPEED / 1.0;  // Adjust 1.0 to your max wheel speed in m/s
  int pwm1 = (int)(wheel1_speed * scale);
  int pwm2 = (int)(wheel2_speed * scale);
  int pwm3 = (int)(wheel3_speed * scale);
  int pwm4 = (int)(wheel4_speed * scale);

  // Clamp to -255 to 255
  pwm1 = constrain(pwm1, -MAX_SPEED, MAX_SPEED);
  pwm2 = constrain(pwm2, -MAX_SPEED, MAX_SPEED);
  pwm3 = constrain(pwm3, -MAX_SPEED, MAX_SPEED);
  pwm4 = constrain(pwm4, -MAX_SPEED, MAX_SPEED);

  // Set motor speeds
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
    } else if (command.startsWith("s")) {
      // Servo command: s <angle>
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int angle = command.substring(spaceIndex + 1).toInt();
        setServo(1, angle);
        setServo(2, angle);
        Serial.print("Set servo 1 and 2 to angle ");
        Serial.println(angle);
      }
    } else if (command.startsWith("p")) {
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
    } else {
      Serial.println("Unknown command. Use: f/b/l/r/cw/ccw <speed>, stop, s <angle>, p on/off");
    }
  }

  delay(100);
}