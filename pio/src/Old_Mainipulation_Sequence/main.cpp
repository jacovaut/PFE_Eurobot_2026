/*
=========================================================
 MANIPULATOR CONTROL - FULL CLEAN VERSION
---------------------------------------------------------
 Commands:

   P        -> Full automatic Pump1 sequence
   N         -> Home both axes
   C         -> Read encoders
   R         -> Reset encoders

 Servo:
   S <angle>
   F CW
   F CCW
   F CENTER
   F <angle>

=========================================================
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

// =====================================================
// PINS
// =====================================================

constexpr int Pump1 = 25;
constexpr int Pump2 = 26;
constexpr int Pump3 = 27;
constexpr int Pump4 = 14;

constexpr int STOP = 13;
constexpr int FLIP = 18;

constexpr int MOTOR_TURN1 = 15;
constexpr int MOTOR_TURN2 = 2;
constexpr int MOTOR_LIFT1 = 4;
constexpr int MOTOR_LIFT2 = 5;

constexpr int ENCODER_LIFT_A = 21;
constexpr int ENCODER_LIFT_B = 19;
constexpr int ENCODER_TURN_A = 22;
constexpr int ENCODER_TURN_B = 23;

// =====================================================
// PWM
// =====================================================

constexpr int PWM_FREQ = 5000;
constexpr int PWM_RES  = 8;

constexpr int CH_L1 = 4;
constexpr int CH_L2 = 5;
constexpr int CH_T1 = 6;
constexpr int CH_T2 = 7;

// =====================================================
// EASY PARAMETERS
// =====================================================

// ---------- P sequence ----------
int activePump = 1;

const int  LIFT_FAST = 230;
const int  LIFT_SLOW = 170;
const long LIFT_TARGET = 880;
const long LIFT_SLOW_AT = 760;

const int  TURN_FAST = 255;
const int  TURN_SLOW = 120;
const long TURN_TARGET = 1400;
const long TURN_SLOW_AT = 1300;

const long TURN_START_AT_LIFT = 200;

const long PRE_DROP_TICKS = -5;

// ---------- HOMING ----------
const int  HOME_LIFT_FAST = 210;
const int  HOME_LIFT_SLOW = 170;
const long HOME_LIFT_TARGET = -790;
const long HOME_LIFT_SLOW_AT = 300;

const int  HOME_TURN_FAST = 255;
const int  HOME_TURN_SLOW = 160;
const long HOME_TURN_TARGET = -1420;
const long HOME_TURN_SLOW_AT = 1200;

// =====================================================
// OBJECTS
// =====================================================

Servo stopper_servo;
Servo flipper_servo;

ESP32Encoder lift_encoder;
ESP32Encoder turn_encoder;

// =====================================================
// STATES
// =====================================================

enum AutoState {
  IDLE,
  PRE_DROP,
  RUNNING,
  HOMING
};

AutoState autoState = IDLE;

bool liftActive = false;
bool turnActive = false;

bool turnStarted = false;

bool homingLift = false;
bool homingTurn = false;

// movement tracking
long liftStart = 0;
long turnStart = 0;

long liftMoveTarget = 0;
long turnMoveTarget = 0;

// =====================================================
// HELPERS
// =====================================================

void setPump(int n, bool on) {

  int pin = 0;

  if (n == 1) pin = Pump1;
  if (n == 2) pin = Pump2;
  if (n == 3) pin = Pump3;
  if (n == 4) pin = Pump4;

  digitalWrite(pin, on ? HIGH : LOW);
}

void liftBrake() {
  ledcWrite(CH_L1, 255);
  ledcWrite(CH_L2, 255);
}

void turnBrake() {
  ledcWrite(CH_T1, 255);
  ledcWrite(CH_T2, 255);
}

void releaseAllMotors() {

  liftActive = false;
  turnActive = false;

  ledcWrite(CH_L1, 0);
  ledcWrite(CH_L2, 0);

  ledcWrite(CH_T1, 0);
  ledcWrite(CH_T2, 0);

  Serial.println("Motors Released");
}

// =====================================================
// START MOVES
// =====================================================

void startLift(long ticks, bool homeMove = false) {

  homingLift = homeMove;

  liftStart = lift_encoder.getCount();
  liftMoveTarget = abs(ticks);
  liftActive = true;

  int speed = homeMove ? HOME_LIFT_FAST : LIFT_FAST;

  if (ticks > 0) {
    ledcWrite(CH_L1, speed);
    ledcWrite(CH_L2, 0);
  } else {
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, speed);
  }
}

void startTurn(long ticks, bool homeMove = false) {

  homingTurn = homeMove;

  turnStart = turn_encoder.getCount();
  turnMoveTarget = abs(ticks);
  turnActive = true;

  int speed = homeMove ? HOME_TURN_FAST : TURN_FAST;

  if (ticks > 0) {
    ledcWrite(CH_T1, speed);
    ledcWrite(CH_T2, 0);
  } else {
    ledcWrite(CH_T1, 0);
    ledcWrite(CH_T2, speed);
  }
}

// =====================================================
// UPDATE MOVES
// =====================================================

void updateLift() {

  if (!liftActive) return;

  long delta = abs(lift_encoder.getCount() - liftStart);

  int slowAt  = homingLift ? HOME_LIFT_SLOW_AT : LIFT_SLOW_AT;
  int slowSpd = homingLift ? HOME_LIFT_SLOW    : LIFT_SLOW;

  if (delta >= slowAt && delta < liftMoveTarget) {
    if (ledcRead(CH_L1) > 0) ledcWrite(CH_L1, slowSpd);
    if (ledcRead(CH_L2) > 0) ledcWrite(CH_L2, slowSpd);
  }

  if (delta >= liftMoveTarget) {
    liftBrake();
    liftActive = false;
  }
}

void updateTurn() {

  if (!turnActive) return;

  long delta = abs(turn_encoder.getCount() - turnStart);

  int slowAt  = homingTurn ? HOME_TURN_SLOW_AT : TURN_SLOW_AT;
  int slowSpd = homingTurn ? HOME_TURN_SLOW    : TURN_SLOW;

  if (delta >= slowAt && delta < turnMoveTarget) {
    if (ledcRead(CH_T1) > 0) ledcWrite(CH_T1, slowSpd);
    if (ledcRead(CH_T2) > 0) ledcWrite(CH_T2, slowSpd);
  }

  if (delta >= turnMoveTarget) {
    turnBrake();
    turnActive = false;
  }
}

// =====================================================
// P1 SEQUENCE
// =====================================================

void startPumpSequence(int pumpNumber) {

  if (autoState != IDLE) return;

  activePump = pumpNumber;

  Serial.print("P");
  Serial.print(pumpNumber);
  Serial.println(" Start");

  setPump(activePump, true);

  startLift(PRE_DROP_TICKS);

  autoState = PRE_DROP;
}

// =====================================================
// HOME
// =====================================================

void goHome() {

  if (autoState != IDLE) return;

  Serial.println("Homing");

  startLift(HOME_LIFT_TARGET, true);
  startTurn(HOME_TURN_TARGET, true);

  autoState = HOMING;
}

// =====================================================
// AUTOMATIC STATE MACHINE
// =====================================================

void updateAutoSequence() {

  // after pre-drop
  if (autoState == PRE_DROP && !liftActive) {

    lift_encoder.setCount(0);
    turn_encoder.setCount(0);

    turnStarted = false;

    startLift(LIFT_TARGET);

    autoState = RUNNING;
  }

  // main sequence
  if (autoState == RUNNING) {

    long liftPos = lift_encoder.getCount();

    if (!turnStarted && liftPos >= TURN_START_AT_LIFT) {
      startTurn(TURN_TARGET);
      turnStarted = true;
    }

    if (!liftActive && !turnActive) {

      setPump(activePump, false);

      autoState = IDLE;

      Serial.print("P");
      Serial.print(activePump);
      Serial.println(" Complete");
    }
  }

  // homing
  if (autoState == HOMING) {

    if (!liftActive && !turnActive) {

      lift_encoder.setCount(0);
      turn_encoder.setCount(0);

      autoState = IDLE;

      Serial.println("Home Complete - Encoders Reset");
    }
  }
}

// =====================================================
// SERVOS
// =====================================================

void setStopper(int angle) {
  stopper_servo.write(constrain(angle,0,180));
}

void setFlipper(int angle) {
  flipper_servo.write(constrain(angle,0,180));
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  pinMode(Pump1, OUTPUT);
  pinMode(Pump2, OUTPUT);
  pinMode(Pump3, OUTPUT);
  pinMode(Pump4, OUTPUT);

  digitalWrite(Pump1, LOW);
  digitalWrite(Pump2, LOW);
  digitalWrite(Pump3, LOW);
  digitalWrite(Pump4, LOW);

  lift_encoder.attachHalfQuad(ENCODER_LIFT_A, ENCODER_LIFT_B);
  turn_encoder.attachHalfQuad(ENCODER_TURN_A, ENCODER_TURN_B);

  lift_encoder.setCount(0);
  turn_encoder.setCount(0);

  ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_T1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_T2, PWM_FREQ, PWM_RES);

  ledcAttachPin(MOTOR_LIFT1, CH_L1);
  ledcAttachPin(MOTOR_LIFT2, CH_L2);
  ledcAttachPin(MOTOR_TURN1, CH_T1);
  ledcAttachPin(MOTOR_TURN2, CH_T2);

  liftBrake();
  turnBrake();

  ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_T1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_T2, PWM_FREQ, PWM_RES);

  ledcAttachPin(MOTOR_LIFT1, CH_L1);
  ledcAttachPin(MOTOR_LIFT2, CH_L2);
  ledcAttachPin(MOTOR_TURN1, CH_T1);
  ledcAttachPin(MOTOR_TURN2, CH_T2);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  stopper_servo.setPeriodHertz(50);
  flipper_servo.setPeriodHertz(50);

  stopper_servo.attach(STOP, 500, 2400);
  flipper_servo.attach(FLIP, 500, 2400);

  setStopper(15);
  setFlipper(90);

  Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  updateLift();
  updateTurn();
  updateAutoSequence();

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "P1") startPumpSequence(1);
    else if (cmd == "P2") startPumpSequence(2);
    else if (cmd == "P3") startPumpSequence(3);
    else if (cmd == "P4") startPumpSequence(4);

    else if (cmd == "N") {
      goHome();
    }

    else if (cmd == "C") {
      Serial.print("Lift: ");
      Serial.print(lift_encoder.getCount());
      Serial.print("   Turn: ");
      Serial.println(turn_encoder.getCount());
    }

    else if (cmd == "R") {
      lift_encoder.setCount(0);
      turn_encoder.setCount(0);
      Serial.println("Encoders Reset");
    }

    else if (cmd == "S") {

      liftActive = false;
      liftBrake();

      Serial.println("Lift Brake");
    }

    else if (cmd == "X") {

      turnActive = false;
      turnBrake();

      Serial.println("Turn Brake");
    }

    else if (cmd == "W") {

      autoState = IDLE;
      releaseAllMotors();
    }

    else if (cmd.startsWith("S ")) {
      int a = cmd.substring(2).toInt();
      setStopper(a);
    }

    else if (cmd.startsWith("F ")) {

      String v = cmd.substring(2);

      if (v == "CW") setFlipper(35);
      else if (v == "CCW") setFlipper(145);
      else if (v == "CENTER") setFlipper(90);
      else setFlipper(v.toInt());
    }
  }
}


