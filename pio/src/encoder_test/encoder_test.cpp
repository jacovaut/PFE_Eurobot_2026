#include <Arduino.h>

/*********** CHANGE THESE PINS ***********/
#define E1A 2
#define E1B 19
#define E2A 36
#define E2B 34
#define E3A 35
#define E3B 32
/****************************************/

volatile long t1 = 0, t2 = 0, t3 = 0;

void IRAM_ATTR isr1(){ if (digitalRead(E1A) == digitalRead(E1B)) t1++; else t1--; }
void IRAM_ATTR isr2(){ if (digitalRead(E2A) == digitalRead(E2B)) t2++; else t2--; }
void IRAM_ATTR isr3(){ if (digitalRead(E3A) == digitalRead(E3B)) t3++; else t3--; }

void setup() {
  Serial.begin(115200);

  pinMode(E1A, INPUT_PULLUP); pinMode(E1B, INPUT_PULLUP);
  pinMode(E2A, INPUT_PULLUP); pinMode(E2B, INPUT_PULLUP);
  pinMode(E3A, INPUT_PULLUP); pinMode(E3B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(E1A), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2A), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E3A), isr3, CHANGE);

  Serial.print("3 Encoder Test Started\r\n");
}

void loop() {
  // take a snapshot (avoid printing changing values mid-print)
  long a = t1, b = t2, c = t3;

  Serial.printf("E1:%ld E2:%ld E3:%ld\r\n", a, b, c);
  delay(200); // 5 Hz output
}