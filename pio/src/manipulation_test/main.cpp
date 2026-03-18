#include <Arduino.h>
#include <stdint.h>

// ---- FROM PCB MANIPULATION ---- //
constexpr int M1_I1 = 4;
constexpr int M1_I2 = 5;
constexpr int M2_I1 = 15;
constexpr int M2_I2 = 2;
constexpr int pump = 32; // Control the Pump
constexpr int Valve1 = 25; // Control the Valve1
constexpr int Valve2 = 26; // Control the Valve2
constexpr int Valve3 = 27; // Control the Valve3
constexpr int Valve4 = 14; // Control the Valve4
constexpr int Valve_out = 12; // Control the Valve_out
constexpr int Thermo = 33; // Control Thermo?
constexpr int STOP = 13; // Connected to Stopper?

void setup(){
    Serial.begin(115200);
    Serial.write("Starting Manipulation Test...\n");

    pinMode(M1_I1, OUTPUT);
    pinMode(M1_I2, OUTPUT);
    pinMode(M2_I1, OUTPUT);
    pinMode(M2_I2, OUTPUT);
    pinMode(Valve1, OUTPUT);
    pinMode(Valve2, OUTPUT);
    pinMode(Valve3, OUTPUT);
    pinMode(Valve4, OUTPUT);
    pinMode(Valve_out, OUTPUT);
    pinMode(Thermo, OUTPUT);
    pinMode(STOP, OUTPUT);
    pinMode(pump, OUTPUT);

    digitalWrite(M1_I1, LOW);
    digitalWrite(M1_I2, LOW);
    digitalWrite(M2_I1, LOW);
    digitalWrite(M2_I2, LOW);
    digitalWrite(pump, LOW);
    digitalWrite(Valve1, LOW);
    digitalWrite(Valve2, LOW);
    digitalWrite(Valve3, LOW);
    digitalWrite(Valve4, LOW);
    digitalWrite(Valve_out, LOW);
    digitalWrite(Thermo, LOW);
    digitalWrite(STOP, LOW);


    analogWrite(pump, 50);
    delay(2000);
    analogWrite(pump, 100);
    delay(2000);
    analogWrite(pump, 150);
    delay(2000);
    analogWrite(pump, 200);
    delay(2000);
    analogWrite(pump, 255);
    delay(2000);
    digitalWrite(Valve1, HIGH);
    delay(1000);
    digitalWrite(Valve1, LOW);
    digitalWrite(Valve2, HIGH);
    delay(1000);
    digitalWrite(Valve2, LOW);
    digitalWrite(Valve3, HIGH);
    delay(1000);
    digitalWrite(Valve3, LOW);
    digitalWrite(Valve4, HIGH);
    delay(1000);
    digitalWrite(Valve4, LOW);
    digitalWrite(pump, LOW);

}

void loop(){

}