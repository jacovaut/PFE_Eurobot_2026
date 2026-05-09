#ifndef PAMI_NINJA_SCRIPT_FUNCTIONS_H
#define PAMI_NINJA_SCRIPT_FUNCTIONS_H

#include <Arduino.h>
#include "pami_ninja_scripted_control.h"

extern PamiNinjaScriptedControl pami;
extern volatile uint8_t selectedAutonomousRun;

inline void runYellowAutonomousRun() {
  // Edit these timed commands for the yellow run.
  pami.pickup();

 /* pami.backward(500);
  pami.right(4000);
  pami.forward(500);
  pami.pickup();
  pami.backward(250);
  pami.left(4000);
  pami.release();
  pami.rotateCcw(250);
  pami.stopFor(250);*/
}

inline void runBlueAutonomousRun() {
  // First Set of Blocs
  pami.right(2650);      
  pami.forward(700);  
  pami.stopFor(500);     
  pami.backward(900);
  pami.stopFor(500);
  
  // Second Set of Blocs
  pami.left(1500);       
  pami.forward(700);
  pami.stopFor(500);      
  pami.backward(900);    
  pami.stopFor(500); 
  
  // Big Set of Blocs Round 1
  pami.left(1500);
  pami.forward(700);
  pami.stopFor(500);    
  pami.backward(900); 
  pami.stopFor(500);

  // Big Set of Blocs Round 2
  pami.left(300);
  pami.forward(700);
  pami.stopFor(500);
  pami.backward(900);
  pami.stopFor(500);

  // Get to Toggle Area
  pami.right(1700);
  pami.forward(600);
  pami.toggleArmSweep();

}

inline void runAutonomousScript() {
  const uint8_t run = selectedAutonomousRun;

  if (run == 2) {
    Serial.println("Running blue autonomous script");
    runBlueAutonomousRun();
  } else {
    Serial.println("Running yellow autonomous script");
    runYellowAutonomousRun();
  }
}

#endif
