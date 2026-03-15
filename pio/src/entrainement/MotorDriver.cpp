// MotorDriver implementation file

#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t stepPin, uint8_t dirPin, uint8_t csPin)
    : stepPin(stepPin), dirPin(dirPin), csPin(csPin) {}

void MotorDriver::begin(FastAccelStepperEngine &engine) {
    sd.setChipSelectPin(csPin);
    delay(1);

    sd.resetSettings();
    sd.clearStatus();

    // *** Edit settings as needed
    sd.setDecayMode(HPSDDecayMode::AutoMixed);
    sd.setStepMode(HPSDStepMode::MicroStep32);
    // ***

    sd.setCurrentMilliamps36v4(current);

    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);

    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, HIGH);

    // Use RMT driver instead of MCPWM/PCNT to avoid conflicts with ESP32Encoder
    stepper = engine.stepperConnectToPin(stepPin, DRIVER_RMT);
    if (!stepper) return;

    stepper->setDirectionPin(dirPin);

    stepper->setAcceleration(Accel * StepsPerRev / 60);
}

void MotorDriver::Enabledriver(bool en) {
    if (en) {
        sd.enableDriver();
    } else {
        sd.disableDriver();
    }
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

void MotorDriver::stop() {
    if (stepper) stepper->stopMove();
}

void MotorDriver::print_status() {
    uint8_t status = sd.readStatus();

    if (status & (1 << (uint8_t)HPSDStatusBit::OTS))
    {
        Serial.println("Motor overtemperature shutdown detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::AOCP))
    {
        Serial.println("Motor Channel A overcurrent shutdown detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::BOCP))
    {
        Serial.println("Motor Channel B overcurrent shutdown detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::APDF))
    {
        Serial.println("Motor Channel A predriver fault detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::BPDF))
    {
        Serial.println("Motor Channel B predriver fault detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::UVLO))
    {
        Serial.println("Motor undervoltage lockout detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::STD))
    {
        Serial.println("Motor stall detected!");
    }
    if (status & (1 << (uint8_t)HPSDStatusBit::STDLAT))
    {
        Serial.println("Motor latched stall detect!");
    }
    if (status == 0)
    {
        Serial.println("Motor status OK.");
    }
}

void MotorDriver::clearStatus() {
    sd.clearStatus();
}

void MotorDriver::step() {
    for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
    delayMicroseconds(2000);
  }
}