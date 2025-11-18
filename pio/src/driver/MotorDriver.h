#pragma once
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include "FastAccelStepper.h"

class MotorDriver {
public:
    MotorDriver(uint8_t stepPin, uint8_t dirPin, uint8_t csPin);

    void begin(FastAccelStepperEngine &engine);
    void setSpeedRPM(float rpm);
    void runForward();
    void runBackward();

private:
    uint8_t stepPin, dirPin, csPin;
    HighPowerStepperDriver sd;
    FastAccelStepper* stepper = nullptr;

    static constexpr float MicroSteps = 32.0;
    static constexpr float StepsPerRev = 200.0 * MicroSteps;
};
