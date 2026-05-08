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
  // Slot 1
  pami.right(3000);      // 1: 'd' 3000ms (t=3.00s)
  pami.pickup();         // 2: 'p' 1039ms (t=4.04s)
  pami.right(400);       // 3: 'd' 400ms (t=4.44s)
  pami.stopFor(200);     // 4: '.' 200ms (t=4.64s)
  pami.rotateCcw(625);   // 5: 'q' 625ms (t=5.26s)
  pami.left(200);        // 6: 'a' 200ms (t=5.46s)
  pami.stopFor(389);     // 7: '.' 389ms (t=5.85s)
  pami.forward(2500);    // 8: 'w' 2500ms (t=8.35s)
  pami.pumpOff(500);     // 9: 'f' 500ms (t=8.85s)
  pami.forward(200);     // 10: 'w' 200ms (t=9.05s)
  pami.stopFor(1000);    // 11: '.' 1000ms (t=10.05s)
  pami.rotateCw(600);    // 12: 'e' 600ms (t=10.65s)
  pami.stopFor(200);     // 13: '.' 200ms (t=10.85s)
  pami.backward(400);    // 14: 's' 400ms (t=11.25s)
  pami.stopFor(200);     // 15: '.' 200ms (t=11.45s)
  pami.left(400);        // 16: 'a' 400ms (t=11.85s)
  pami.stopFor(200);     // 17: '.' 200ms (t=12.05s)
  // Slot 2
  pami.right(2700);      // 1: 'd' 2700ms (t=2.70s)
  pami.pickup();         // 2: 'p' 1039ms (t=3.74s)
  pami.stopFor(200);     // 3: '.' 200ms (t=3.94s)
  pami.rotateCcw(620);   // 4: 'q' 620ms (t=4.56s)
  pami.stopFor(389);     // 5: '.' 389ms (t=4.95s)
  pami.left(200);        // 6: 'a' 200ms (t=5.15s)
  pami.forward(2400);    // 7: 'w' 2400ms (t=7.55s)
  pami.pumpOff(750);     // 8: 'f' 750ms (t=8.30s)
  pami.stopFor(1000);    // 9: '.' 1000ms (t=9.30s)
  pami.rotateCw(600);    // 10: 'e' 600ms (t=9.90s)
  pami.stopFor(200);     // 11: '.' 200ms (t=10.10s)
  pami.backward(200);    // 12: 's' 200ms (t=10.30s)
  pami.stopFor(225);     // 13: '.' 225ms (t=10.52s)
  pami.left(400);        // 14: 'a' 400ms (t=10.92s)
  pami.backward(100);    // 15: 's' 100ms (t=11.02s)
  pami.stopFor(200);     // 16: '.' 200ms (t=11.22s)
  // Slot 3
  pami.right(1900);      // 1: 'd' 1900ms (t=1.90s)
  pami.stopFor(200);     // 2: '.' 200ms (t=2.10s)
  pami.forward(150);     // 3: 'w' 150ms (t=2.25s)
  pami.pickup();         // 4: 'p' 805ms (t=3.06s)
  pami.stopFor(200);     // 5: '.' 200ms (t=3.25s)
  pami.rotateCcw(471);   // 6: 'q' 471ms (t=3.73s)
  pami.stopFor(200);     // 7: '.' 200ms (t=3.93s)
  pami.forward(1452);    // 8: 'w' 1452ms (t=5.38s)
  pami.pumpOff(1348);    // 9: 'f' 1348ms (t=6.73s)
  pami.stopFor(1000);    // 10: '.' 1000ms (t=7.73s)
  pami.rotateCw(539);    // 11: 'e' 539ms (t=8.27s)
  pami.stopFor(200);     // 12: '.' 200ms (t=8.46s)
  pami.backward(172);    // 13: 's' 172ms (t=8.64s)
  pami.stopFor(200);     // 14: '.' 200ms (t=8.84s)
  pami.left(300);        // 15: 'a' 300ms (t=9.14s)
  pami.backward(150);    // 16: 's' 150ms (t=9.29s)
  pami.stopFor(200);     // 17: '.' 200ms (t=9.49s)
  // Slot 4
  pami.right(1650);      // 1: 'd' 1650ms (t=1.65s)
  pami.stopFor(200);     // 2: '.' 200ms (t=1.85s)
  pami.forward(150);     // 3: 'w' 150ms (t=2.00s)
  pami.pickup();         // 4: 'p' 805ms (t=2.81s)
  pami.stopFor(300);     // 5: '.' 300ms (t=3.10s)
  pami.rotateCcw(471);   // 6: 'q' 471ms (t=3.58s)
  pami.stopFor(200);     // 7: '.' 200ms (t=3.78s)
  pami.forward(1452);    // 8: 'w' 1452ms (t=5.23s)
  pami.pumpOff(1200);    // 9: 'f' 1200ms (t=6.43s)
  pami.stopFor(1000);    // 10: '.' 1000ms (t=7.43s)
  pami.rotateCw(539);    // 11: 'e' 539ms (t=7.97s)
  pami.stopFor(200);     // 12: '.' 200ms (t=8.17s)
  pami.backward(172);    // 13: 's' 172ms (t=8.34s)
  pami.stopFor(200);     // 14: '.' 200ms (t=8.54s)
  pami.left(300);        // 15: 'a' 300ms (t=8.84s)
  pami.backward(200);    // 16: 's' 200ms (t=9.04s)
  pami.stopFor(200);     // 17: '.' 200ms (t=9.24s)
  // Slot 5
  pami.right(700);       // 1: 'd' 700ms (t=0.70s)
  pami.stopFor(200);     // 2: '.' 200ms (t=0.90s)
  pami.forward(210);     // 3: 'w' 210ms (t=1.11s)
  pami.pickup();         // 4: 'p' 900ms (t=2.01s)
  pami.stopFor(200);     // 5: '.' 200ms (t=2.21s)
  pami.backward(200);    // 6: 's' 200ms (t=2.41s)
  pami.rotateCw(379);    // 7: 'e' 379ms (t=2.79s)
  pami.stopFor(200);     // 8: '.' 200ms (t=2.99s)
  pami.forward(1105);    // 9: 'w' 1105ms (t=4.09s)
  pami.release();        // 10-12: 'j' 200ms, 'f' 300ms, 'u' 600ms (t=5.19s)
  pami.stopFor(200);     // 13: '.' 200ms (t=5.39s)
  pami.right(852);       // 14: 'd' 852ms (t=6.25s)
  pami.stopFor(200);     // 15: '.' 200ms (t=6.45s)
  pami.backward(1900);   // 16: 's' 1900ms (t=8.35s)
  pami.stopFor(200);     // 17: '.' 200ms (t=8.55s)
  pami.rotateCcw(500);   // 18: 'q' 500ms (t=9.05s)
  pami.stopFor(200);     // 19: '.' 200ms (t=9.25s)
  pami.backward(157);    // 20: 's' 157ms (t=9.40s)
  pami.stopFor(200);     // 21: '.' 200ms (t=9.60s)
  pami.left(400);        // 22: 'a' 400ms (t=10.00s)
  pami.backward(200);    // 23: 's' 200ms (t=10.20s)
  pami.stopFor(200);     // 24: '.' 200ms (t=10.40s)
  // Slot 6
  pami.right(500);       // 1: 'd' 500ms (t=0.50s)
  pami.stopFor(200);     // 2: '.' 200ms (t=0.70s)
  pami.forward(230);     // 3: 'w' 230ms (t=0.93s)
  pami.pickup();         // 4: 'p' 900ms (t=1.83s)
  pami.stopFor(200);     // 5: '.' 200ms (t=2.03s)
  pami.backward(200);    // 6: 's' 200ms (t=2.23s)
  pami.rotateCw(379);    // 7: 'e' 379ms (t=2.61s)
  pami.stopFor(200);     // 8: '.' 200ms (t=2.81s)
  pami.forward(600);     // 9: 'w' 600ms (t=3.41s)
  pami.rotateCcw(50);    // 10: 'q' 50ms (t=3.46s)
  pami.release();        // 11-13: 'j' 200ms, 'f' 300ms, 'u' 600ms (t=4.56s)
  pami.stopFor(200);     // 14: '.' 200ms (t=4.76s)
  pami.rotateCw(200);    // 15: 'e' 200ms (t=4.96s)
  pami.right(852);       // 16: 'd' 852ms (t=5.81s)
  pami.stopFor(200);     // 17: '.' 200ms (t=6.01s)
  pami.backward(1400);   // 18: 's' 1400ms (t=7.41s)
  pami.stopFor(200);     // 19: '.' 200ms (t=7.61s)
  pami.rotateCcw(500);   // 20: 'q' 500ms (t=8.11s)
  pami.stopFor(200);     // 21: '.' 200ms (t=8.31s)
  pami.backward(157);    // 22: 's' 157ms (t=8.47s)
  pami.stopFor(200);     // 23: '.' 200ms (t=8.67s)
  pami.left(264);        // 24: 'a' 264ms (t=8.93s)
  pami.stopFor(200);     // 25: '.' 200ms (t=9.13s)
  pami.backward(200);    // 26: 's' 200ms (t=9.33s)
  // Slot 7
  pami.right(300);       // 1: 'd' 300ms (t=0.30s)
  pami.stopFor(200);     // 2: '.' 200ms (t=0.50s)
  pami.forward(245);     // 3: 'w' 245ms (t=0.74s)
  pami.pickup();         // 4: 'p' 900ms (t=1.65s)
  pami.stopFor(200);     // 5: '.' 200ms (t=1.84s)
  pami.rotateCw(379);    // 6: 'e' 379ms (t=2.22s)
  pami.stopFor(200);     // 7: '.' 200ms (t=2.42s)
  pami.forward(600);     // 8: 'w' 600ms (t=3.02s)
  pami.rotateCcw(125);   // 9: 'q' 125ms (t=3.15s)
  pami.release();        // 10-12: 'j' 200ms, 'f' 300ms, 'u' 600ms (t=4.25s)
  pami.stopFor(200);     // 13: '.' 200ms (t=4.45s)
  pami.rotateCw(200);    // 14: 'e' 200ms (t=4.65s)
  pami.right(852);       // 15: 'd' 852ms (t=5.50s)
  pami.stopFor(200);     // 16: '.' 200ms (t=5.70s)
  pami.backward(1300);   // 17: 's' 1300ms (t=7.00s)
  pami.stopFor(200);     // 18: '.' 200ms (t=7.20s)
  pami.rotateCcw(500);   // 19: 'q' 500ms (t=7.70s)
  pami.stopFor(200);     // 20: '.' 200ms (t=7.90s)
  pami.backward(157);    // 21: 's' 157ms (t=8.06s)
  pami.stopFor(200);     // 22: '.' 200ms (t=8.26s)
  pami.left(264);        // 23: 'a' 264ms (t=8.52s)
  pami.backward(200);    // 24: 's' 200ms (t=8.72s)
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
