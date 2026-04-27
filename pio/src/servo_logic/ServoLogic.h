#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoLogic {
public:
	enum FlipperState : uint8_t {
		STATE_CENTER = 0,
		STATE_CW = 1,
		STATE_CCW = 2,
		STATE_CUSTOM = 3,
	};

	enum BlockColor : uint8_t {
		COLOR_UNKNOWN = 0,
		COLOR_YELLOW = 1,
		COLOR_BLUE = 2,
	};

	ServoLogic(int pin, int center_angle = 90, int cw_angle = 35, int ccw_angle = 145);

	void begin();
	void setAngle(int angle);
	void center();
	void flipClockwise();
	void flipCounterClockwise();
	bool flipForColor(BlockColor color);

	int getAngle() const;
	BlockColor getLastColor() const;
	FlipperState getState() const;
	const char *getStateName() const;
	const char *getLastColorName() const;

private:
	Servo servo;
	int servo_pin;
	int center_angle;
	int clockwise_angle;
	int counter_clockwise_angle;
	int current_angle;
	FlipperState current_state = STATE_CENTER;
	BlockColor last_color = COLOR_UNKNOWN;
};

class BlockerServo {
public:
	enum BlockerState : uint8_t {
		STATE_CLOSED = 0,
		STATE_OPEN = 1,
		STATE_CUSTOM = 2,
	};

	BlockerServo(int pin, int closed_angle = 15, int open_angle = 95);

	void begin();
	void setAngle(int angle);
	void open();
	void close();
	bool isOpen() const;

	int getAngle() const;
	BlockerState getState() const;
	const char *getStateName() const;

private:
	Servo servo;
	int servo_pin;
	int closed_angle;
	int open_angle;
	int current_angle;
	BlockerState current_state = STATE_CLOSED;
};
