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
  ledcSetup(PWM_CH_PUMP, PWM_FREQ, PWM_RES);
  ledcAttachPin(PUMP_PIN, PWM_CH_PUMP);

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

void setPump(int speed) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  ledcWrite(PWM_CH_PUMP, speed);
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
  // Print encoder values
  Serial.print("Encoders: ");
  Serial.print(encoder1.getCount());
  Serial.print(", ");
  Serial.print(encoder2.getCount());
  Serial.print(", ");
  Serial.print(encoder3.getCount());
  Serial.print(", ");
  Serial.println(encoder4.getCount());

  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("m")) {
      // Motor command: m<motor> <speed>
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int motor = command.substring(1, spaceIndex).toInt();
        int speed = command.substring(spaceIndex + 1).toInt();
        setMotor(motor, speed);
        Serial.print("Set motor ");
        Serial.print(motor);
        Serial.print(" to speed ");
        Serial.println(speed);
      }
    } else if (command.startsWith("s")) {
      // Servo command: s<servo> <angle>
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int servo = command.substring(1, spaceIndex).toInt();
        int angle = command.substring(spaceIndex + 1).toInt();
        setServo(servo, angle);
        Serial.print("Set servo ");
        Serial.print(servo);
        Serial.print(" to angle ");
        Serial.println(angle);
      }
    } else if (command.startsWith("p")) {
      // Pump command: p <speed>
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int speed = command.substring(spaceIndex + 1).toInt();
        setPump(speed);
        Serial.print("Set pump to speed ");
        Serial.println(speed);
      }
    } else if (command.startsWith("move")) {
      // Mecanum move command: move <vx> <vy> <omega>
      // Parse three floats
      int space1 = command.indexOf(' ');
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);
      if (space1 > 0 && space2 > 0 && space3 > 0) {
        float vx = command.substring(space1 + 1, space2).toFloat();
        float vy = command.substring(space2 + 1, space3).toFloat();
        float omega = command.substring(space3 + 1).toFloat();
        setMecanumSpeeds(vx, vy, omega);
        Serial.print("Set mecanum speeds: vx=");
        Serial.print(vx);
        Serial.print(", vy=");
        Serial.print(vy);
        Serial.print(", omega=");
        Serial.println(omega);
      }
    }
  }

  delay(100);
}