#ifndef PAMI_NINJA_SCRIPTED_CONTROL_H
#define PAMI_NINJA_SCRIPTED_CONTROL_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "pins_pami_ninja.h"

class PamiNinjaScriptedControl {
public:
  void begin() {
    pinMode(MOTOR_FL_IN1, OUTPUT);
    pinMode(MOTOR_FL_IN2, OUTPUT);
    pinMode(MOTOR_FR_IN1, OUTPUT);
    pinMode(MOTOR_FR_IN2, OUTPUT);
    pinMode(MOTOR_RL_IN1, OUTPUT);
    pinMode(MOTOR_RL_IN2, OUTPUT);
    pinMode(MOTOR_RR_IN1, OUTPUT);
    pinMode(MOTOR_RR_IN2, OUTPUT);

    pinMode(PUMP_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);

#if PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
    pinMode(OBSTACLE_TRIG_PIN, OUTPUT);
    pinMode(OBSTACLE_ECHO_PIN, INPUT);
    digitalWrite(OBSTACLE_TRIG_PIN, LOW);
#endif

    ledcSetup(CH_FL_IN1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FL_IN2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FR_IN1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FR_IN2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RL_IN1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RL_IN2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RR_IN1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RR_IN2, PWM_FREQ, PWM_RES);

    ledcAttachPin(MOTOR_FL_IN1, CH_FL_IN1);
    ledcAttachPin(MOTOR_FL_IN2, CH_FL_IN2);
    ledcAttachPin(MOTOR_FR_IN1, CH_FR_IN1);
    ledcAttachPin(MOTOR_FR_IN2, CH_FR_IN2);
    ledcAttachPin(MOTOR_RL_IN1, CH_RL_IN1);
    ledcAttachPin(MOTOR_RL_IN2, CH_RL_IN2);
    ledcAttachPin(MOTOR_RR_IN1, CH_RR_IN1);
    ledcAttachPin(MOTOR_RR_IN2, CH_RR_IN2);

    encoder1.attachHalfQuad(ENC_FL_A, ENC_FL_B);
    encoder2.attachHalfQuad(ENC_FR_A, ENC_FR_B);
    encoder3.attachHalfQuad(ENC_RL_A, ENC_RL_B);
    encoder4.attachHalfQuad(ENC_RR_A, ENC_RR_B);

    servo1.attach(SERVO_LEFT_PIN);
    servo2.attach(SERVO_RIGHT_PIN);
    servoUp();
    stop();
  }

  void stop() {
    setMecanumSpeeds(0.0f, 0.0f, 0.0f);
  }

  void forward(uint32_t durationMs) {
    runTimedMotion(0.0f, linearPwm / 255.0f, 0.0f, durationMs);
  }

  void backward(uint32_t durationMs) {
    runTimedMotion(0.0f, -linearPwm / 255.0f, 0.0f, durationMs);
  }

  void left(uint32_t durationMs) {
    runTimedMotion(linearPwm / 255.0f, 0.0f, 0.0f, durationMs);
  }

  void right(uint32_t durationMs) {
    runTimedMotion(-linearPwm / 255.0f, 0.0f, 0.0f, durationMs);
  }

  void rotateCcw(uint32_t durationMs) {
    runTimedMotion(0.0f, 0.0f, omegaPwm / 255.0f, durationMs);
  }

  void rotateCw(uint32_t durationMs) {
    runTimedMotion(0.0f, 0.0f, -omegaPwm / 255.0f, durationMs);
  }

  void wait(uint32_t durationMs) {
    stopFor(durationMs);
  }

  void stopFor(uint32_t durationMs) {
    runTimedMotion(0.0f, 0.0f, 0.0f, durationMs);
  }

  void pickup() {
    pickupBlock();
  }

  void pickup(uint32_t waitAfterMs) {
    pickupBlock();
    delay(waitAfterMs);
  }

  void release() {
    releaseBlock();
  }

  void release(uint32_t waitAfterMs) {
    releaseBlock();
    delay(waitAfterMs);
  }

  void pumpOn(uint32_t durationMs = 0) {
    setPump(true);
    delay(durationMs);
  }

  void pumpOff(uint32_t durationMs = 0) {
    setPump(false);
    delay(durationMs);
  }

  void armsUp(uint32_t durationMs = 0) {
    servoUp();
    delay(durationMs);
  }

  void armsDown(uint32_t durationMs = 0) {
    servoDown();
    delay(durationMs);
  }

  void toggleArmSweep() {
    const int minAngle = min(ARM_DOWN_ANGLE, ARM_UP_ANGLE);
    const int maxAngle = max(ARM_DOWN_ANGLE, ARM_UP_ANGLE);

    armSweepEnabled = !armSweepEnabled;

    if (armSweepEnabled) {
      currentServoAngle = constrain(currentServoAngle, minAngle, maxAngle);
      armSweepDirection = currentServoAngle >= maxAngle ? -1 : 1;
      lastArmSweepStepTime = 0;
      Serial.printf("Arm sweep: ON (%d <-> %d)\n", minAngle, maxAngle);
    } else {
      Serial.printf("Arm sweep: OFF at %d\n", currentServoAngle);
    }
  }

  void setArmSweep(bool enabled) {
    if (armSweepEnabled != enabled) {
      toggleArmSweep();
    }
  }

  bool isArmSweepEnabled() const {
    return armSweepEnabled;
  }

  void updateArmSweep() {
    if (!armSweepEnabled) {
      return;
    }

    const uint32_t now = millis();
    if (now - lastArmSweepStepTime < ARM_SWEEP_DELAY) {
      return;
    }

    const int minAngle = min(ARM_DOWN_ANGLE, ARM_UP_ANGLE);
    const int maxAngle = max(ARM_DOWN_ANGLE, ARM_UP_ANGLE);
    const int step = ARM_SWEEP_STEP > 0 ? ARM_SWEEP_STEP : 1;

    currentServoAngle += armSweepDirection * step;

    if (currentServoAngle >= maxAngle) {
      currentServoAngle = maxAngle;
      armSweepDirection = -1;
    } else if (currentServoAngle <= minAngle) {
      currentServoAngle = minAngle;
      armSweepDirection = 1;
    }

    writeServos(currentServoAngle);
    lastArmSweepStepTime = now;
  }

  void setLinearSpeed(int pwm) {
    linearPwm = constrain(pwm, 0, MAX_SPEED);
  }

  void setOmegaSpeed(int pwm) {
    omegaPwm = constrain(pwm, 0, MAX_SPEED);
  }

  void resetEncoders() {
    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();
    encoder4.clearCount();
  }

#if PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
  float readRearObstacleDistanceCm() {
    digitalWrite(OBSTACLE_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(OBSTACLE_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(OBSTACLE_TRIG_PIN, LOW);

    const unsigned long durationUs =
      pulseIn(OBSTACLE_ECHO_PIN, HIGH, PAMI_ULTRASONIC_TIMEOUT_US);

    if (durationUs == 0) {
      return -1.0f;
    }

    return durationUs / 58.0f;
  }

  bool isRearObstacleDetected() {
    const float distanceCm = readRearObstacleDistanceCm();
    return distanceCm > 0.0f && distanceCm <= PAMI_OBSTACLE_STOP_DISTANCE_CM;
  }

  void updateRearObstacleAvoidance() {
    if (isRearObstacleDetected()) {
      setMecanumSpeeds(0.0f, PAMI_REAR_AVOIDANCE_FORWARD_PWM / 255.0f, 0.0f);
    } else {
      stop();
    }

    updateArmSweep();
  }
#endif

private:
  ESP32Encoder encoder1;
  ESP32Encoder encoder2;
  ESP32Encoder encoder3;
  ESP32Encoder encoder4;
  Servo servo1;
  Servo servo2;

  int motorSpeeds[4] = {0, 0, 0, 0};
  int linearPwm = 120;
  int omegaPwm = 110;
  bool pumpState = false;
  bool armSweepEnabled = false;
  int armSweepDirection = 1;
  int currentServoAngle = ARM_UP_ANGLE;
  uint32_t lastArmSweepStepTime = 0;

  void runTimedMotion(float vx, float vy, float omega, uint32_t durationMs) {
    const uint32_t start = millis();
    do {
#if PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
      if (vy < 0.0f && isRearObstacleDetected()) {
        stop();
        Serial.println("Rear obstacle detected: stopping backward motion");
        return;
      }
#endif
      setMecanumSpeeds(vx, vy, omega);
      updateArmSweep();
      delay(20);
    } while (millis() - start < durationMs);
    stop();
  }

  void setMecanumSpeeds(float vx, float vy, float omega) {
    const float halfLength = WHEELBASE_LENGTH / 2.0f;
    const float halfWidth = WHEELBASE_WIDTH / 2.0f;

    float w1 = vx - vy + (halfLength + halfWidth) * omega;
    float w2 = vx + vy + (halfLength + halfWidth) * omega;
    float w3 = vx + vy - (halfLength + halfWidth) * omega;
    float w4 = vx - vy - (halfLength + halfWidth) * omega;

    int pwm1 = constrain(static_cast<int>(round(w1 * MAX_SPEED)), -MAX_SPEED, MAX_SPEED);
    int pwm2 = constrain(static_cast<int>(round(w2 * MAX_SPEED)), -MAX_SPEED, MAX_SPEED);
    int pwm3 = constrain(static_cast<int>(round(w3 * MAX_SPEED)), -MAX_SPEED, MAX_SPEED);
    int pwm4 = constrain(static_cast<int>(round(w4 * MAX_SPEED)), -MAX_SPEED, MAX_SPEED);

    setMotor(1, pwm1);
    setMotor(2, pwm2);
    setMotor(3, pwm3);
    setMotor(4, pwm4);
  }

  void setMotor(int motor, int speed) {
    int chIn1;
    int chIn2;

    switch (motor) {
      case 1:
        chIn1 = CH_FL_IN1;
        chIn2 = CH_FL_IN2;
        break;
      case 2:
        chIn1 = CH_FR_IN1;
        chIn2 = CH_FR_IN2;
        break;
      case 3:
        chIn1 = CH_RL_IN1;
        chIn2 = CH_RL_IN2;
        break;
      case 4:
        chIn1 = CH_RR_IN1;
        chIn2 = CH_RR_IN2;
        break;
      default:
        return;
    }

    if (motorSpeeds[motor - 1] == speed) {
      return;
    }

    motorSpeeds[motor - 1] = speed;
    int pwm = abs(speed);

    if (speed > 0) {
      ledcWrite(chIn1, pwm);
      ledcWrite(chIn2, 0);
    } else if (speed < 0) {
      ledcWrite(chIn1, 0);
      ledcWrite(chIn2, pwm);
    } else {
      ledcWrite(chIn1, 0);
      ledcWrite(chIn2, 0);
    }
  }

  void setPump(bool state) {
    pumpState = state;
    digitalWrite(PUMP_PIN, state ? HIGH : LOW);
    Serial.printf("Pump: %s\n", pumpState ? "ON" : "OFF");
  }

  void servoUp() {
    armSweepEnabled = false;
    currentServoAngle = ARM_UP_ANGLE;
    writeServos(currentServoAngle);
    delay(ARM_MOVE_DELAY);
  }

  void servoDown() {
    armSweepEnabled = false;
    currentServoAngle = ARM_DOWN_ANGLE;
    writeServos(currentServoAngle);
    delay(ARM_MOVE_DELAY);
  }

  void writeServos(int angle) {
    servo1.write(angle);
    servo2.write(angle);
  }

  void pickupBlock() {
    Serial.println("Pickup block");
    setPump(true);
    servoDown();
    servoUp();
  }

  void releaseBlock() {
    Serial.println("Release block");
    armSweepEnabled = false;
    currentServoAngle = ARM_RELEASE_DOWN_ANGLE;
    writeServos(currentServoAngle);
    delay(ARM_MOVE_DELAY);
    setPump(false);
    servoUp();
  }
};

#endif
