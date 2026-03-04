#define ENC_A 2
#define ENC_B 19

volatile long ticks = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
    ticks++;
  } else {
    ticks--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  Serial.println("Single Encoder Test");
}

void loop() {
  static long last = 0;

  if (last != ticks) {
    Serial.println(ticks);
    last = ticks;
  }
}