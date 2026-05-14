#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Serial echo ready");
}

void loop()
{
  if (!Serial.available()) {
    return;
  }

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() > 0) {
    Serial.print(input);
    Serial.println(" from esp");
  }
}
