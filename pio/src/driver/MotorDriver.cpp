#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t stepPin, uint8_t dirPin, uint8_t csPin)
    : stepPin(stepPin), dirPin(dirPin), csPin(csPin) {}

void MotorDriver::begin(FastAccelStepperEngine &engine) {
    sd.setChipSelectPin(csPin);
    delay(1);

    sd.resetSettings();
    sd.clearStatus();
    sd.setDecayMode(HPSDDecayMode::AutoMixed);
    sd.setCurrentMilliamps36v4(1500);
    sd.setStepMode(HPSDStepMode::MicroStep32);
    sd.enableDriver();

    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);

    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, HIGH);

    stepper = engine.stepperConnectToPin(stepPin);
    if (!stepper) return;

    stepper->setDirectionPin(dirPin);

    stepper->setAcceleration(200.0 * StepsPerRev / 60);
}

void MotorDriver::setSpeedRPM(float rpm) {
    if (stepper)
        stepper->setSpeedInHz(rpm * StepsPerRev / 60.0);
}

void MotorDriver::runForward() {
    if (stepper) stepper->runForward();
}

void MotorDriver::runBackward() {
    if (stepper) stepper->runBackward();
}
