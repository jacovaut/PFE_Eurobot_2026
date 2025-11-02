#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder;
ESP32Encoder encoder2;

// timer and flag for example, not needed for encoders
unsigned long encoder2lastToggled;
bool encoder2Paused = false;

void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

	// use pin 19 and 18 for the first encoder
	encoder.attachHalfQuad(23, 22);
		
	// set starting count value after attaching
	encoder.clearCount();
}

void loop(){
	// Loop and read the count
	Serial.println("Encoder count = " + String((int32_t)encoder.getCount()));
	delay(100);
}
