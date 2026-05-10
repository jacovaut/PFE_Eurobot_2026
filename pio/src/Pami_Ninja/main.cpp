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
volatile bool stopRequested = false;
volatile uint8_t selectedAutonomousRun = 1;

#if PAMI_SEQUENCE_TRIGGER_MODE == PAMI_SEQUENCE_TRIGGER_MICROROS
#include "pami_ninja_microros_setup.h"
#endif

bool shouldStopAutonomousRun() {
#if PAMI_SEQUENCE_TRIGGER_MODE == PAMI_SEQUENCE_TRIGGER_MICROROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
#endif

  while (Serial.available()) {
    const char key = toupper(Serial.read());
    if (key == 'X') {
      stopRequested = true;
    } else if (key == 'K') {
      pami.setArmSweep(false);
    }
  }

  if (stopRequested) {
    pami.stop();
    return true;
  }

  return false;
}

#include "pami_ninja_script_functions.h"

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
  Serial.println("  match/running false stops and resets the script");
  Serial.println("  /team_color - std_msgs/String yellow=run 1, blue=run 2");
#else
  Serial.println("Mode: serial keyboard trigger");
  Serial.println("Commands:");
  Serial.println("  1 - select yellow run");
  Serial.println("  2 - select blue run");
  Serial.println("  G - run script");
  Serial.println("  R - reset script so it can run again");
  Serial.println("  I - toggle servo sweep");
  Serial.println("  K - turn servo sweep off");
  Serial.println("  X - stop current run");
#endif
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pami.begin();
  pami.setLinearSpeed(70);
  pami.setOmegaSpeed(60);
  selectedAutonomousRun = 1;
  stopRequested = false;

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
    stopRequested = false;
    hasRun = true;
    runAutonomousScript();
    pami.setArmSweep(false);
    if (stopRequested) {
      Serial.println("Script stopped. Send false then true to run again.");
    } else {
      Serial.println("Script complete. Send false then true to run again.");
    }
  }
#else
  if (Serial.available()) {
    char key = toupper(Serial.read());
    if (key == 'G') {
      if (!hasRun) {
        stopRequested = false;
        hasRun = true;
        runAutonomousScript();
        pami.setArmSweep(false);
        if (stopRequested) {
          Serial.println("Script stopped. Press R then G to run again.");
        } else {
          Serial.println("Script complete. Press R then G to run again.");
        }
      } else {
        Serial.println("Script already ran. Press R then G to run again.");
      }
    } else if (key == 'R') {
      hasRun = false;
      stopRequested = false;
      Serial.println("Script reset.");
    } else if (key == '1') {
      selectedAutonomousRun = 1;
      hasRun = false;
      Serial.println("Selected yellow run.");
    } else if (key == '2') {
      selectedAutonomousRun = 2;
      hasRun = false;
      Serial.println("Selected blue run.");
    } else if (key == 'I') {
      pami.toggleArmSweep();
    } else if (key == 'K') {
      pami.setArmSweep(false);
    } else if (key == 'X') {
      stopRequested = true;
      pami.stop();
      Serial.println("Stopped.");
    }
  }
#endif

  pami.updateArmSweep();
  delay(20);
}
