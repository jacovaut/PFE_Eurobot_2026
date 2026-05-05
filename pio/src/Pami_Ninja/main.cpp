#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "pins_pami_ninja.h"

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

// Valve and pump states
bool valveState = false;
bool pumpState = false;

// Encoder reading control
bool encoderReadingEnabled = false;

// Keyboard control - track currently pressed keys
bool keyPressed[256] = {false};  // Track state of each ASCII key
unsigned long keyLastReceived[256] = {0};  // Track last time each key was received

// Key press/release timeout (in milliseconds)
#define KEY_TIMEOUT 150  // If key not received for 150ms, consider it released

// Maximum speed for keyboard control
#define MAX_KEYBOARD_SPEED 200


// Functions declaration
void setMecanumSpeeds(float vx, float vy, float omega);
void setMotor(int motor, int speed);
void setServo(int servo, int angle);
void setPump(bool state);
void setValve(bool state);
void resetEncoders();
void printKeyboardHelp();
void processKeyboardInput();


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

  // Valve (Solenoid)
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

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

  // Servos
  servo1.attach(SERVO_LEFT_PIN);
  servo2.attach(SERVO_RIGHT_PIN);
  Serial.println("Robot initialized");
}


void loop() {
  // Print encoder values (if enabled)
  if (encoderReadingEnabled) {
    Serial.print("Encoders: ");
    Serial.print(encoder1.getCount());
    Serial.print(", ");
    Serial.print(encoder2.getCount());
    Serial.print(", ");
    Serial.print(encoder3.getCount());
    Serial.print(", ");
    Serial.println(encoder4.getCount());
  }

  // Process keyboard input - hold keys for continuous movement
  processKeyboardInput();

  delay(100);
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
    ledcWrite(ch_in1, pwm);    // Write PWM to IN1 channel
    ledcWrite(ch_in2, 0);       // Set IN2 to 0
    Serial.printf("Motor %d - IN1: %d, IN2: 0\n", motor, pwm);

  } else if (speed < 0) {
    // Reverse: PWM on IN2, LOW on IN1
    ledcWrite(ch_in1, 0);       // Set IN1 to 0
    ledcWrite(ch_in2, pwm);     // Write PWM to IN2 channel
    Serial.printf("Motor %d - IN1: 0, IN2: %d\n", motor, pwm);

  } else {
    // Stop: PWM to 0, both pins LOW
    ledcWrite(ch_in1, 0);       // Set IN1 to 0
    ledcWrite(ch_in2, 0);       // Set IN2 to 0
    Serial.printf("Motor %d - IN1: 0, IN2: 0\n", motor);
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
  pumpState = state;
  digitalWrite(PUMP_PIN, state ? HIGH : LOW);
}

void setValve(bool state) {
  valveState = state;
  digitalWrite(SOLENOID_PIN, state ? HIGH : LOW);
}

void resetEncoders() {
  encoder1.clearCount();
  encoder2.clearCount();
  encoder3.clearCount();
  encoder4.clearCount();
  Serial.println("All encoders reset");
}

void printKeyboardHelp() {
  Serial.println("\n========================================");
  Serial.println("KEYBOARD CONTROL - Hold keys to move");
  Serial.println("========================================");
  Serial.println("MOVEMENT:");
  Serial.println("  W - Forward");
  Serial.println("  S - Backward");
  Serial.println("  A - Strafe Left");
  Serial.println("  D - Strafe Right");
  Serial.println("  Q - Rotate Counter-Clockwise");
  Serial.println("  E - Rotate Clockwise");
  Serial.println("  SPACE - Stop all motion");
  Serial.println("\nBLOCK HANDLING:");
  Serial.println("  P - Pick up block");
  Serial.println("  R - Release block");
  Serial.println("\nSERVO CONTROL:");
  Serial.println("  U - Raise servo");
  Serial.println("  O - Lower servo");
  Serial.println("\nPUMP CONTROL:");
  Serial.println("  J - Pump Toggle ON/OFF");
  Serial.println("\nENCODER CONTROL:");
  Serial.println("  N - Encoder Toggle ON/OFF");
  Serial.println("  L - Reset Encoders");
  Serial.println("========================================\n");
}

void processKeyboardInput() {
  unsigned long now = millis();
  
  // Check if there's data available to read
  if (Serial.available()) {
    char key = Serial.read();
    key = toupper(key);  // Convert to uppercase
    
    if (key >= 32 && key <= 126) {  // Valid ASCII range
      keyPressed[key] = true;  // Mark key as pressed
      keyLastReceived[key] = now;  // Update timestamp
    }
  }
  
  // Check for key timeouts (key release detection)
  if (keyPressed[KEY_FORWARD] && (now - keyLastReceived[KEY_FORWARD] > KEY_TIMEOUT)) {
    keyPressed[KEY_FORWARD] = false;
  }
  if (keyPressed[KEY_BACKWARD] && (now - keyLastReceived[KEY_BACKWARD] > KEY_TIMEOUT)) {
    keyPressed[KEY_BACKWARD] = false;
  }
  if (keyPressed[KEY_LEFT] && (now - keyLastReceived[KEY_LEFT] > KEY_TIMEOUT)) {
    keyPressed[KEY_LEFT] = false;
  }
  if (keyPressed[KEY_RIGHT] && (now - keyLastReceived[KEY_RIGHT] > KEY_TIMEOUT)) {
    keyPressed[KEY_RIGHT] = false;
  }
  if (keyPressed[KEY_ROTATE_CW] && (now - keyLastReceived[KEY_ROTATE_CW] > KEY_TIMEOUT)) {
    keyPressed[KEY_ROTATE_CW] = false;
  }
  if (keyPressed[KEY_ROTATE_CCW] && (now - keyLastReceived[KEY_ROTATE_CCW] > KEY_TIMEOUT)) {
    keyPressed[KEY_ROTATE_CCW] = false;
  }
  
  // Process movement keys
  if (keyPressed[KEY_FORWARD]) {
    setMecanumSpeeds(MAX_KEYBOARD_SPEED / 255.0, 0, 0);
  }
  else if (keyPressed[KEY_BACKWARD]) {
    setMecanumSpeeds(-MAX_KEYBOARD_SPEED / 255.0, 0, 0);
  }
  else if (keyPressed[KEY_LEFT]) {
    setMecanumSpeeds(0, MAX_KEYBOARD_SPEED / 255.0, 0);
  }
  else if (keyPressed[KEY_RIGHT]) {
    setMecanumSpeeds(0, -MAX_KEYBOARD_SPEED / 255.0, 0);
  }
  else if (keyPressed[KEY_ROTATE_CW]) {
    setMecanumSpeeds(0, 0, MAX_KEYBOARD_SPEED / 255.0);
  }
  else if (keyPressed[KEY_ROTATE_CCW]) {
    setMecanumSpeeds(0, 0, -MAX_KEYBOARD_SPEED / 255.0);
  }
  else {
    setMecanumSpeeds(0, 0, 0);
  }
}

void setMecanumSpeeds(float vx, float vy, float omega) {
  float L = WHEELBASE_LENGTH / 2.0;
  float W = WHEELBASE_WIDTH / 2.0;
  float R = WHEEL_RADIUS;
  Serial.printf("Calculating speeds for vx=%.2f, vy=%.2f, omega=%.2f\n", vx, vy, omega);
  float w1 = (vx - vy - (L + W) * omega) / R;
  float w2 = (vx + vy + (L + W) * omega) / R;
  float w3 = (vx + vy - (L + W) * omega) / R;
  float w4 = (vx - vy + (L + W) * omega) / R;
  Serial.printf("Raw wheel speeds: w1=%.3f, w2=%.3f, w3=%.3f, w4=%.3f\n", w1, w2, w3, w4);
  float scale = MAX_SPEED / 1.0;

  // Use proper rounding instead of truncation
  int pwm1 = constrain((int)round(w1 * scale), -MAX_SPEED, MAX_SPEED);
  int pwm2 = constrain((int)round(w2 * scale), -MAX_SPEED, MAX_SPEED);
  int pwm3 = constrain((int)round(w3 * scale), -MAX_SPEED, MAX_SPEED);
  int pwm4 = constrain((int)round(w4 * scale), -MAX_SPEED, MAX_SPEED);

  setMotor(1, pwm1);
  setMotor(2, pwm2);
  setMotor(3, pwm3);
  setMotor(4, pwm4);
  if(pwm1 == 0 && pwm2 == 0 && pwm3 == 0 && pwm4 == 0) {
    Serial.println("Stopping all motors");
  } else {
    Serial.printf("Setting motors - FL: %d, FR: %d, RL: %d, RR: %d\n", pwm1, pwm2, pwm3, pwm4);
  }
}