// main motion control for a 4-wheeled mecanum robot
// subscriber :
//      - cmd_vel
//      - odometry (optional, not implemented yet)
// publisher :
//      - wheel speeds (optional, not implemented yet)
//
// wheel configuration:
//        Front        ⊤
//  [0] \        / [1] |
//                     |
//                     |   
//                     | lx
//                     |
//  [2] /        \ [3] |
//        Back         ⊥
//  ⊢-----------------⊣
//           ly
//
// to change acceleraiton, or current, edit MotorDriver.h
// to change microstepping or decay mode, edit MotorDriver.cpp
//
// not sure if needed and not implemented yet:
//      - PID control on wheel speeds based on odometry feedback
//      - acceleration/deceleration ramps
//      - max speed limits
// 
//
// jacov2024

#include <Arduino.h>
#include "MotorDriver.h"

FastAccelStepperEngine engine;

// Motor definitions (StepPin, DirPin, CSPin)
MotorDriver motor1(17, 16, 4);
MotorDriver motor2(15, 13, 12);
MotorDriver motor3(26, 25, 33);
MotorDriver motor4(23,22,21);

// Robot motion and dimmention variables
float vx = 0; // m/s
float vy = 0; // m/s
float w  = 0; // rad/s

float lxy = 0.12 + 0.12; // lx (m) + ly (m) = 240mm

float r = 0.05; //m   (100mm mecanum)

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
    
    motor1.begin(engine);
    // motor2.begin(engine);
    motor3.begin(engine);
    // motor4.begin(engine);

    motor1.setSpeedRPM(60);
    // motor2.setSpeedRPM(60);
    motor3.setSpeedRPM(60);
    // motor4.setSpeedRPM(60);

    motor1.runForward();
    // motor2.runForward();
    motor3.runForward();
    // motor4.runForward();

    // motor1.step();
    // motor2.step();
    // motor3.step();
    // motor4.step();

    motor1.clearStatus();
    // motor2.clearStatus();
    motor3.clearStatus();
    // motor4.clearStatus();

    delay(3000);

    motor1.print_status();
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
    wheelSpeeds[0] = (1/r) * ( vx - vy - w*lxy );
    wheelSpeeds[1] = (1/r) * ( vx + vy + w*lxy );
    wheelSpeeds[2] = (1/r) * ( vx + vy - w*lxy );
    wheelSpeeds[3] = (1/r) * ( vx - vy + w*lxy );
}