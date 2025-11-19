#include "MotorDriver.h"

FastAccelStepperEngine engine;

// Motor definitions
MotorDriver motor1(17, 16, 4);
MotorDriver motor2(15, 13, 12);
MotorDriver motor3(26, 25, 33);
MotorDriver motor4(23,22,21);

// Robot motion and dimmention variables
float vx = 0; // m/s
float vy = 0; // m/s
float w  = 0; // rad/s

float L = 0.12; // m
float W = 0.12; // m
float A = 0.24; // m

float R = 0.05; //m   (100mm mecanum)

void calculateWheelSpeeds(float vx, float vy, float w, float* wheelSpeeds);
void setSpeeds(float new_vx, float new_vy, float new_w);

void setup() {
    pinMode(2, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(19, OUTPUT);

    digitalWrite(2, HIGH);
    digitalWrite(27, HIGH);
    digitalWrite(32, HIGH);
    digitalWrite(19, HIGH);

    Serial.begin(115200);

    SPI.begin(18,14,5);
    engine.init();
    
    // motor1.begin(engine);
    // motor2.begin(engine);
    motor3.begin(engine);
    // motor4.begin(engine);

    // motor1.setSpeedRPM(10);
    // motor2.setSpeedRPM(10);
    motor3.setSpeedRPM(10);
    // motor4.setSpeedRPM(10);

    // motor1.runForward();
    // motor2.runForward();
    motor3.runForward();
    // motor4.runForward();
}

void loop()
{
    // if(Serial.available()) {
    //     String input = Serial.readString();
        
    //     Serial.print("Vx = ");
    //     Serial.println(input.toFloat());

    //     setSpeeds(input.toFloat(), 0, 0);
    // }
}

void setSpeeds(float new_vx, float new_vy, float new_w) {
    vx = new_vx;
    vy = new_vy;
    w  = new_w;

    if (vx == 0 && vy == 0 && w == 0) {
        motor1.setSpeedRPM(0);
        motor2.setSpeedRPM(0);
        motor3.setSpeedRPM(0);
        motor4.setSpeedRPM(0);
        return;
    }

    float wheelSpeeds [4] = {0, 0, 0, 0}; // FL, FR, RL, RR

    calculateWheelSpeeds(vx, vy, w, wheelSpeeds);

    motor1.setSpeedRPM(wheelSpeeds[0] * 60 / (2 * PI));
    motor2.setSpeedRPM(wheelSpeeds[1] * 60 / (2 * PI));
    motor3.setSpeedRPM(wheelSpeeds[2] * 60 / (2 * PI));
    motor4.setSpeedRPM(wheelSpeeds[3] * 60 / (2 * PI));

    motor1.runForward();
    motor2.runForward();
    motor3.runForward();
    motor4.runForward();

    Serial.printf("Wheel Speeds (rad/s): FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f\n", 
                  wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
}

void calculateWheelSpeeds(float vx, float vy, float w, float* wheelSpeeds) {
    wheelSpeeds[0] = (1/R) * ( vx - vy - w*A );
    wheelSpeeds[1] = (1/R) * ( vx + vy + w*A );
    wheelSpeeds[2] = (1/R) * ( vx + vy - w*A );
    wheelSpeeds[3] = (1/R) * ( vx - vy + w*A );
}