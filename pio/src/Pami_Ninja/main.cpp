#include <Arduino.h>
#include "pami_ninja_scripted_control.h"

#define PAMI_SEQUENCE_TRIGGER_SERIAL 0
#define PAMI_SEQUENCE_TRIGGER_MICROROS 1

#ifndef PAMI_SEQUENCE_TRIGGER_MODE
#define PAMI_SEQUENCE_TRIGGER_MODE PAMI_SEQUENCE_TRIGGER_SERIAL
#endif

PamiNinjaScriptedControl pami;

bool hasRun = false;
volatile bool startRequested = false;

#if PAMI_SEQUENCE_TRIGGER_MODE == PAMI_SEQUENCE_TRIGGER_MICROROS
#include "pami_ninja_microros_setup.h"
#endif

void runAutonomousScript() {
  // Edit these timed commands to program the robot.
  pami.forward(1000);
  pami.backward(1000);
  
  pami.stopFor(250);
  pami.left(500);
  pami.stopFor(250);
  pami.rotateCw(400);
  pami.stopFor(250);
  
}

void printHelp() {
  Serial.println();
  Serial.println("PAMI Ninja scripted firmware");
#if PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
  Serial.println("Mode: standalone rear obstacle avoidance");
  Serial.printf("Rear obstacle <= %.1f cm: move forward away from obstacle\n",
                PAMI_OBSTACLE_STOP_DISTANCE_CM);
#elif PAMI_SEQUENCE_TRIGGER_MODE == PAMI_SEQUENCE_TRIGGER_MICROROS
  Serial.println("Mode: micro-ROS trigger");
  Serial.println("ROS topic:");
  Serial.println("  match/running - std_msgs/Bool true runs script");
#else
  Serial.println("Mode: serial keyboard trigger");
  Serial.println("Commands:");
  Serial.println("  G - run script");
  Serial.println("  R - reset script so it can run again");
  Serial.println("  I - toggle continuous arm sweep");
  Serial.println("  X - stop motors");
#endif
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pami.begin();
  pami.setLinearSpeed(120);
  pami.setOmegaSpeed(110);

  printHelp();
#if PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
  Serial.println("Standalone rear obstacle avoidance is active.");
#elif PAMI_SEQUENCE_TRIGGER_MODE == PAMI_SEQUENCE_TRIGGER_MICROROS
  setupMicroRosTrigger();
#else
  Serial.println("Press G to run the script.");
#endif
}

void loop() {
#if PAMI_ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE
  pami.updateRearObstacleAvoidance();
#elif PAMI_SEQUENCE_TRIGGER_MODE == PAMI_SEQUENCE_TRIGGER_MICROROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));

  if (startRequested && !hasRun) {
    startRequested = false;
    hasRun = true;
    runAutonomousScript();
    Serial.println("Script complete. Send false then true to run again.");
  }
#else
  if (Serial.available()) {
    char key = toupper(Serial.read());
    if (key == 'G') {
      if (!hasRun) {
        hasRun = true;
        runAutonomousScript();
        Serial.println("Script complete. Press R then G to run again.");
      } else {
        Serial.println("Script already ran. Press R then G to run again.");
      }
    } else if (key == 'R') {
      hasRun = false;
      Serial.println("Script reset.");
    } else if (key == 'I') {
      pami.toggleArmSweep();
    } else if (key == 'X') {
      pami.stop();
      Serial.println("Stopped.");
    }
  }
#endif

  pami.updateArmSweep();
  delay(20);
}
