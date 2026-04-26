#include <Arduino.h>
#include "ArmMotor.h"

ArmMotor armMotor(1.0f, 0.0f, 0.001f, 5, 4, 21, 19, 18);

constexpr int Pump1 = 25; // Pump for suction cup 1
constexpr int Pump2 = 26; // Pump for suction cup 2
constexpr int Pump3 = 27; // Pump for suction cup 3
constexpr int Pump4 = 14; // Pump for suction cup 4
static uint32_t timer;

void setup() {

    Serial.begin(115200);

    pinMode(15, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(15, LOW);
    digitalWrite(2, LOW);

    pinMode(Pump1, OUTPUT);
    pinMode(Pump2, OUTPUT);
    pinMode(Pump3, OUTPUT);
    pinMode(Pump4, OUTPUT);
    digitalWrite(Pump1, HIGH);
    digitalWrite(Pump2, HIGH);
    digitalWrite(Pump3, HIGH);
    digitalWrite(Pump4, HIGH);

    armMotor.setup();
    armMotor.home();

    Serial.print("Arm motor homed. Entering control loop...\n");
    while (!Serial.available())
    {
        Serial.print(".");
        delay(100);
    }
    Serial.flush();
    timer = millis();
}

void loop() {
    static uint32_t last = micros();
    uint32_t now = micros();
    const uint32_t interval = 2000;  // 2 ms for 500 Hz (1/500 * 1e6 microseconds)

    if (now - last >= interval) {
        last += interval;
        armMotor.update();
    }

    if (millis() - timer >= 1000) // Print status every second
    {
        armMotor.setTarget(500.0f); // Example: set target to 90 degrees
        // timer = millis();
    }

    if (millis() - timer >= 3000) // Print status every second
    {
        armMotor.setTarget(625.0f); // Example: set target to 90 degrees
        // timer = millis();
    }

    if(millis() - timer >= 5000) // Print status every second
    {
        digitalWrite(Pump1, LOW);
        digitalWrite(Pump2, LOW);
        digitalWrite(Pump3, LOW);
        digitalWrite(Pump4, LOW);

        delay(1000); // Keep pumps on for 1 second

        delay(3000); // Wait for 1 second at target position
        armMotor.setTarget(300.0f); // Example: set target to 90 degrees
        Serial.println("Moving to 300");
        
        while (!armMotor.targetReached())
        {
            static uint32_t last = micros();
            uint32_t now = micros();
            const uint32_t interval = 2000;  // 2 ms for 500 Hz (1/500 * 1e6 microseconds)
            
            if (now - last >= interval) {
                last += interval;
                armMotor.update();
            }
        }

        delay(3000); // Wait for 1 second at target position
        Serial.println("Moving to 0");
        
        armMotor.setTarget(0.0f); // Example: set target to 90 degrees
        
        while (!armMotor.targetReached())
        {
            static uint32_t last = micros();
            uint32_t now = micros();
            const uint32_t interval = 2000;  // 2 ms for 500 Hz (1/500 * 1e6 microseconds)
            
            if (now - last >= interval) {
                last += interval;
                armMotor.update();
            }
        }

        digitalWrite(Pump1, HIGH);
        digitalWrite(Pump2, HIGH);
        digitalWrite(Pump3, HIGH);
        digitalWrite(Pump4, HIGH);

        delay(1000); // Keep pumps off for 1 second

        timer = millis();
    }
}