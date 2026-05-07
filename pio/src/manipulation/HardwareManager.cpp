#include "HardwareManager.h"
#include <Arduino.h>

// Pin definitions mirrored from main.cpp
// M1/M2 = conveyor motors, MOS1..7 = pump + valves
constexpr int _M1_I1 = 4;
constexpr int _M1_I2 = 5;
constexpr int _M2_I1 = 15;
constexpr int _M2_I2 = 2;
constexpr int _MOS1  = 32; // Pump
constexpr int _MOS2  = 25; // Valve 1
constexpr int _MOS3  = 26; // Valve 2
constexpr int _MOS4  = 27; // Valve 3
constexpr int _MOS5  = 14; // Valve 4
constexpr int _MOS6  = 12; // Valve_out
constexpr int _MOS7  = 33; // Thermo
constexpr int _STOP  = 13;

HardwareManager::HardwareManager(BlockManager* blockList) : BlockList(blockList) {}

void HardwareManager::init() {
    pinMode(_M1_I1, OUTPUT); digitalWrite(_M1_I1, LOW);
    pinMode(_M1_I2, OUTPUT); digitalWrite(_M1_I2, LOW);
    pinMode(_M2_I1, OUTPUT); digitalWrite(_M2_I1, LOW);
    pinMode(_M2_I2, OUTPUT); digitalWrite(_M2_I2, LOW);
    pinMode(_MOS1,  OUTPUT); digitalWrite(_MOS1,  LOW);
    pinMode(_MOS2,  OUTPUT); digitalWrite(_MOS2,  LOW);
    pinMode(_MOS3,  OUTPUT); digitalWrite(_MOS3,  LOW);
    pinMode(_MOS4,  OUTPUT); digitalWrite(_MOS4,  LOW);
    pinMode(_MOS5,  OUTPUT); digitalWrite(_MOS5,  LOW);
    pinMode(_MOS6,  OUTPUT); digitalWrite(_MOS6,  LOW);
    pinMode(_MOS7,  OUTPUT); digitalWrite(_MOS7,  LOW);
    pinMode(_STOP,  INPUT);
}

void HardwareManager::pickUp(int* cups) {
    // TODO: implement pick sequence per cup slot
    // cups[i]: 0 = not present, 1 = pick as-is, 2 = pick and flip
}

void HardwareManager::dropOff() {
    // TODO: implement dispense sequence
}