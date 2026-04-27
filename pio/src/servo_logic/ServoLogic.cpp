#include "ServoLogic.h"

ServoLogic::ServoLogic(int pin, int center_angle_value, int cw_angle_value, int ccw_angle_value)
	: servo_pin(pin),
	  center_angle(center_angle_value),
	  clockwise_angle(cw_angle_value),
	  counter_clockwise_angle(ccw_angle_value),
	  current_angle(center_angle_value) {}

void ServoLogic::begin() {
	servo.setPeriodHertz(50);
	servo.attach(servo_pin, 500, 2400);
	center();
}

void ServoLogic::setAngle(int angle) {
	current_angle = constrain(angle, 0, 180);
	servo.write(current_angle);

	if (current_angle == clockwise_angle) {
		current_state = STATE_CW;
	} else if (current_angle == counter_clockwise_angle) {
		current_state = STATE_CCW;
	} else if (current_angle == center_angle) {
		current_state = STATE_CENTER;
	} else {
		current_state = STATE_CUSTOM;
	}
}

void ServoLogic::center() {
	setAngle(center_angle);
}

void ServoLogic::flipClockwise() {
	setAngle(clockwise_angle);
}

void ServoLogic::flipCounterClockwise() {
	setAngle(counter_clockwise_angle);
}

bool ServoLogic::flipForColor(BlockColor color) {
	last_color = color;

	if (color == COLOR_YELLOW) {
		flipClockwise();
		return true;
	}

	if (color == COLOR_BLUE) {
		flipCounterClockwise();
		return true;
	}

	center();
	return false;
}

int ServoLogic::getAngle() const {
	return current_angle;
}

ServoLogic::BlockColor ServoLogic::getLastColor() const {
	return last_color;
}

ServoLogic::FlipperState ServoLogic::getState() const {
	return current_state;
}

const char *ServoLogic::getStateName() const {
	switch (current_state) {
		case STATE_CENTER:
			return "CENTER";
		case STATE_CW:
			return "CW";
		case STATE_CCW:
			return "CCW";
		case STATE_CUSTOM:
			return "CUSTOM";
		default:
			return "UNKNOWN";
	}
}

const char *ServoLogic::getLastColorName() const {
	switch (last_color) {
		case COLOR_YELLOW:
			return "YELLOW";
		case COLOR_BLUE:
			return "BLUE";
		case COLOR_UNKNOWN:
		default:
			return "UNKNOWN";
	}
}

BlockerServo::BlockerServo(int pin, int closed_angle_value, int open_angle_value)
	: servo_pin(pin),
	  closed_angle(closed_angle_value),
	  open_angle(open_angle_value),
	  current_angle(closed_angle_value) {}

void BlockerServo::begin() {
	servo.setPeriodHertz(50);
	servo.attach(servo_pin, 500, 2400);
	close();
}

void BlockerServo::setAngle(int angle) {
	current_angle = constrain(angle, 0, 180);
	servo.write(current_angle);

	if (current_angle == open_angle) {
		current_state = STATE_OPEN;
	} else if (current_angle == closed_angle) {
		current_state = STATE_CLOSED;
	} else {
		current_state = STATE_CUSTOM;
	}
}

void BlockerServo::open() {
	setAngle(open_angle);
}

void BlockerServo::close() {
	setAngle(closed_angle);
}

bool BlockerServo::isOpen() const {
	return current_angle > ((closed_angle + open_angle) / 2);
}

int BlockerServo::getAngle() const {
	return current_angle;
}

BlockerServo::BlockerState BlockerServo::getState() const {
	return current_state;
}

const char *BlockerServo::getStateName() const {
	switch (current_state) {
		case STATE_CLOSED:
			return "CLOSED";
		case STATE_OPEN:
			return "OPEN";
		case STATE_CUSTOM:
			return "CUSTOM";
		default:
			return "UNKNOWN";
	}
}
