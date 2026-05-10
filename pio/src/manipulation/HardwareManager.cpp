#include "HardwareManager.h"
#include <ESP32PWM.h>

namespace {
    constexpr int MOTOR_TURN1 = 2;
    constexpr int MOTOR_TURN2 = 15;
    constexpr int MOTOR_LIFT1 = 5;
    constexpr int MOTOR_LIFT2 = 4;

    constexpr int ENCODER_LIFT_A = 19;
    constexpr int ENCODER_LIFT_B = 21;
    constexpr int ENCODER_TURN_A = 23;
    constexpr int ENCODER_TURN_B = 22;

    constexpr int PWM_CHANNEL_LIFT1 = 4;
    constexpr int PWM_CHANNEL_LIFT2 = 5;
    constexpr int PWM_CHANNEL_TURN1 = 6;
    constexpr int PWM_CHANNEL_TURN2 = 7;
}

constexpr int HardwareManager::PUMP_PINS[4];

HardwareManager::HardwareManager()
    : armMotor(
          1.0f, 0.0f, 0.0f,
          MOTOR_LIFT1, MOTOR_LIFT2,
          ENCODER_LIFT_A, ENCODER_LIFT_B,
          PWM_CHANNEL_LIFT1, PWM_CHANNEL_LIFT2),
      turnMotor(
          0.8f, 0.05f, 0.01f,
          MOTOR_TURN1, MOTOR_TURN2,
          ENCODER_TURN_A, ENCODER_TURN_B,
          PWM_CHANNEL_TURN1, PWM_CHANNEL_TURN2)
{}

void HardwareManager::init() {
    if (initialized) return;

    for (int pin : PUMP_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    stopperServo.setPeriodHertz(50);
    flipperServo.setPeriodHertz(50);
    stopperServo.attach(STOP_PIN, 500, 2400);
    flipperServo.attach(FLIP_PIN, 500, 2400);
    setStopperPosition(false);
    setFlipperStop();

    armMotor.setup();
    turnMotor.setup();
    armMotor.disable();
    turnMotor.disable();

    initialized = true;
}

void HardwareManager::updateMotors() {
    armMotor.update();
    turnMotor.update();
}

bool HardwareManager::pauseWithUpdate(uint32_t duration_ms) {
    const uint32_t start = millis();
    while (millis() - start < duration_ms) {
        updateMotors();
        delay(2);
    }
    return true;
}

bool HardwareManager::waitForArmTarget(uint32_t timeout_ms) {
    const uint32_t start = millis();
    while (!armMotor.isAtTarget()) {
        updateMotors();
        if (millis() - start > timeout_ms) return false;
        delay(2);
    }
    return true;
}

bool HardwareManager::waitForTurnTarget(uint32_t timeout_ms) {
    const uint32_t start = millis();
    while (!turnMotor.isAtTarget()) {
        updateMotors();
        if (millis() - start > timeout_ms) return false;
        delay(2);
    }
    return true;
}

void HardwareManager::setPumpState(uint8_t pump_number, bool on) {
    if (pump_number < 1 || pump_number > 4) return;
    digitalWrite(PUMP_PINS[pump_number - 1], on ? HIGH : LOW);
}

void HardwareManager::allPumpsOff() {
    for (uint8_t i = 1; i <= 4; ++i) setPumpState(i, false);
}

void HardwareManager::setStopperPosition(bool open) {
    stopperServo.write(open ? STOP_OPEN_ANGLE : STOP_CLOSED_ANGLE);
}

void HardwareManager::setFlipperStop() {
    flipperServo.write(FLIP_STOP);
}

// ----------------------------------------
// pick — moves to pick position, activates
// requested pumps, wiggles up, stops at
// arm 1000. Pumps stay ON.
// Call home() after to return to rest.
// ----------------------------------------
bool HardwareManager::pick(const bool requested_pumps[4]) {
    armMotor.resetEncoder();
    turnMotor.resetEncoder();
    armMotor.enable();
    turnMotor.enable();

    armMotor.setTarget(500);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    turnMotor.setTarget(450);
    if (!waitForTurnTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    armMotor.setTarget(1100);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    turnMotor.setTarget(1050);
    if (!waitForTurnTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    armMotor.setTarget(1350);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    for (int i = 0; i < 4; ++i) {
        if (requested_pumps[i]) setPumpState(i + 1, true);
    }
    if (!pauseWithUpdate(500)) return false;

    armMotor.setTarget(1450);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(1000))
        return false;

    // Lift clear, pumps still ON — call home() to return
    armMotor.setTarget(1000);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS)) return false;

    return true;
}

// ----------------------------------------
// home — returns arm and turn to rest,
// turns pumps off. Call after pick() or
// thermo().
// ----------------------------------------
bool HardwareManager::home() {
    turnMotor.setTarget(500);
    if (!waitForTurnTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    armMotor.setTarget(500);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    turnMotor.setTarget(0);
    if (!waitForTurnTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    armMotor.setTarget(200);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS)) return false;

    allPumpsOff();
    if (!pauseWithUpdate(500)) return false;

    armMotor.setTarget(0);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS)) return false;

    armMotor.disable();
    turnMotor.disable();
    return true;
}

// ----------------------------------------
// thermo — picks with all 4 pumps,
// holds at wiggle position.
// Call home() after to return to rest.
// ----------------------------------------
bool HardwareManager::thermo() {
    armMotor.resetEncoder();
    turnMotor.resetEncoder();
    armMotor.enable();
    turnMotor.enable();

    armMotor.setTarget(500);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    turnMotor.setTarget(450);
    if (!waitForTurnTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    armMotor.setTarget(1100);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    turnMotor.setTarget(1050);
    if (!waitForTurnTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    armMotor.setTarget(1350);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(500))
        return false;

    for (int i = 0; i < 4; ++i) setPumpState(i + 1, true);
    if (!pauseWithUpdate(500)) return false;

    armMotor.setTarget(1450);
    if (!waitForArmTarget(MOTION_TIMEOUT_MS) || !pauseWithUpdate(1000))
        return false;

    // Holds here with all 4 pumps ON — call home() to return
    return true;
}

// ----------------------------------------
// runFlipSequence — internal, runs one
// stopper + flipper cycle to given angle
// ----------------------------------------
bool HardwareManager::runFlipSequence(int end_angle) {
    setStopperPosition(true);
    if (!pauseWithUpdate(500)) return false;

    setStopperPosition(false);
    if (!pauseWithUpdate(500)) return false;

    flipperServo.write(50);    // start position
    if (!pauseWithUpdate(500)) return false;

    flipperServo.write(end_angle);
    if (!pauseWithUpdate(500)) return false;

    setFlipperStop();          // return to 60
    if (!pauseWithUpdate(500)) return false;

    return true;
}

bool HardwareManager::dispense_flip() {
    return runFlipSequence(140);
}

bool HardwareManager::dispense_keep() {
    return runFlipSequence(40);
}