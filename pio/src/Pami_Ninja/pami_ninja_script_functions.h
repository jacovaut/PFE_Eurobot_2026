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
 
  // Set Bloc 1 
  pami.right(2300);
  pami.stopFor(500);
  pami.backward(200);
  pami.stopFor(500);
  pami.forward(650);
  pami.stopFor(500);
  pami.backward(750);
  pami.stopFor(500);
  /* Set Bloc 2
  pami.left(1300);
  pami.stopFor(500);
  pami.backward(200);
  pami.stopFor(500);
  pami.forward(600);
  pami.stopFor(500);
  pami.backward(900);
  pami.stopFor(500);

  // Get to first Black Block
  pami.left(2000);
  pami.stopFor(500);
  pami.backward(200);
  pami.stopFor(500);
  pami.forward(400);
  pami.stopFor(500);
  pami.pickup(1500);
  pami.backward(500);
  pami.stopFor(500);

  // Drop first Black Bloc in first slot
  pami.right(2500); 
  pami.stopFor(500);
  pami.backward(200);
  pami.stopFor(500);
  pami.forward(100);
  pami.stopFor(500);
  pami.release(1500);
  pami.backward(200);
  pami.stopFor(500);


  /*Big Set of Blocs #1
  pami.left(1000);       
  pami.forward0(700);
  pami.stopFor(500);      
  pami.backward(900);    
  pami.stopFor(500); 
  // Big Set of Blocs #2

  pami.right(500);       
  pami.forward(700);
  pami.stopFor(500);      
  pami.backward(900);    
  pami.stopFor(500); 
  
  // Set of Bloc 1
  pami.right(1300);
  pami.forward(700);
  pami.stopFor(500);    
  pami.backward(900); 
  pami.stopFor(500);

  // Set of Bloc 2
  pami.right(1300);
  pami.forward(700);
  pami.stopFor(500);
  pami.backward(900);
  pami.stopFor(500);

  // Get to Toggle Area
  pami.left(500);
  pami.forward(600);
  pami.toggleArmSweep();
  */

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
