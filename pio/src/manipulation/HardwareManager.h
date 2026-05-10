#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "ArmMotor.h"
#include "TurnMotor.h"

class HardwareManager {
private:
    ArmMotor  armMotor;
    TurnMotor turnMotor;
    Servo     stopperServo;
    Servo     flipperServo;
    bool      initialized = false;

    static constexpr int      PUMP_PINS[4]       = {25, 26, 27, 14};
    static constexpr int      STOP_PIN           = 13;
    static constexpr int      FLIP_PIN           = 12;
    static constexpr int      STOP_CLOSED_ANGLE  = 180;
    static constexpr int      STOP_OPEN_ANGLE    = 120;
    static constexpr int      FLIP_STOP          = 60;
    static constexpr uint32_t MOTION_TIMEOUT_MS  = 10000;

    bool pauseWithUpdate(uint32_t duration_ms);
    bool waitForArmTarget(uint32_t timeout_ms);
    bool waitForTurnTarget(uint32_t timeout_ms);
    void updateMotors();
    void setPumpState(uint8_t pump_number, bool on);
    void allPumpsOff();
    void setStopperPosition(bool open);
    void setFlipperStop();
    bool runFlipSequence(int end_angle);

public:
    HardwareManager();
    void init();

    bool pick(const bool requested_pumps[4]);
    bool home();
    bool thermo();
    bool dispense_flip();
    bool dispense_keep();
};