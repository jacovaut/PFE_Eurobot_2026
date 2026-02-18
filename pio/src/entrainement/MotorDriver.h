// MotorDriver header file

#pragma once
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include "FastAccelStepper.h"

class MotorDriver {
public:
    MotorDriver(uint8_t stepPin, uint8_t dirPin, uint8_t csPin);

    void begin(FastAccelStepperEngine &engine);
    void Enabledriver(bool en);
    void setSpeedRPM(float rpm);
    void runForward();
    void runBackward();
    void stop();
    void clearStatus();
    void print_status();
    void step();

private:
    uint8_t stepPin, dirPin, csPin;
    HighPowerStepperDriver sd;
    FastAccelStepper* stepper = nullptr;

    static constexpr int MicroSteps = 32;
    static constexpr int StepsPerRev = 200 * MicroSteps;

    static constexpr float Accel = 200.0; // RPM/s
    static constexpr int current = 1500; // mA
};
