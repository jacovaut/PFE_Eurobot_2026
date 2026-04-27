#include <Arduino.h>
#include <ESP32Servo.h>
#include "ServoLogic.h"

namespace {
constexpr int FLIP_PIN = 18;
constexpr int BLOCKER_PIN = 13;
constexpr int FLIP_CENTER_ANGLE = 90;
constexpr int FLIP_PRELOAD_ANGLE = 80;
constexpr int FLIP_YELLOW_ANGLE = 60;
constexpr int FLIP_BLUE_ANGLE = 170;
constexpr unsigned long DROP_WINDOW_MS = 300;
constexpr unsigned long STOPPER_SETTLE_MS = 200;
constexpr unsigned long FLIP_PRELOAD_MS = 1000;
constexpr unsigned long FLIP_SETTLE_MS = 700;

ServoLogic flipper(FLIP_PIN, FLIP_CENTER_ANGLE, FLIP_YELLOW_ANGLE, FLIP_BLUE_ANGLE);
BlockerServo stopper(BLOCKER_PIN);

enum TeamColor : uint8_t {
	TEAM_BLUE = 0,
	TEAM_YELLOW = 1,
};

struct SorterConfig {
	TeamColor default_team;
};

constexpr SorterConfig SORTER_CONFIG{
	TEAM_BLUE,
};

enum SequenceStep : uint8_t {
	SEQUENCE_IDLE = 0,
	SEQUENCE_DROP_OPEN,
	SEQUENCE_DROP_CLOSE,
	SEQUENCE_FLIP_CENTER,
	SEQUENCE_FLIP_PRELOAD,
	SEQUENCE_FLIP,
	SEQUENCE_RESET,
};

SequenceStep sequence_step = SEQUENCE_IDLE;
ServoLogic::BlockColor queued_color = ServoLogic::COLOR_UNKNOWN;
unsigned long sequence_deadline_ms = 0;
TeamColor current_team = SORTER_CONFIG.default_team;

const char *getTeamName() {
	return current_team == TEAM_BLUE ? "BLUE" : "YELLOW";
}

const char *getActionName(ServoLogic::BlockColor color) {
	return color == ServoLogic::COLOR_BLUE ? "FLIP" : "STAND";
}

const char *getBlockBehaviorName(ServoLogic::BlockColor block_color) {
	if (block_color == ServoLogic::COLOR_UNKNOWN) {
		return "UNKNOWN";
	}

	bool should_stand =
		(current_team == TEAM_BLUE && block_color == ServoLogic::COLOR_BLUE) ||
		(current_team == TEAM_YELLOW && block_color == ServoLogic::COLOR_YELLOW);
	return should_stand ? "STAND" : "FLIP";
}

void printHelp() {
	Serial.println("Commands:");
	Serial.println("  flip      -> drop one block then flip side");
	Serial.println("  stand     -> drop one block then send block to non-flip side");
	Serial.println("  blue      -> sort a blue block using the current team");
	Serial.println("  yellow    -> sort a yellow block using the current team");
	Serial.println("  team blue -> set team to blue");
	Serial.println("  team yellow -> set team to yellow");
	Serial.println("  center   -> center flipper");
	Serial.println("  open     -> open stopper");
	Serial.println("  close    -> close stopper");
	Serial.println("  status   -> print current servo state");
	Serial.println("  help     -> show commands");
}

void printStatus() {
	Serial.print("Flipper state: ");
	Serial.print(flipper.getStateName());
	Serial.print(" | angle: ");
	Serial.print(flipper.getAngle());
	Serial.print(" | last color: ");
	Serial.println(flipper.getLastColorName());

	Serial.print("Stopper state: ");
	Serial.print(stopper.getStateName());
	Serial.print(" | angle: ");
	Serial.print(stopper.getAngle());
	Serial.print(" | open: ");
	Serial.println(stopper.isOpen() ? "YES" : "NO");

	Serial.print("Team: ");
	Serial.print(getTeamName());
	Serial.print(" | blue block: ");
	Serial.print(getBlockBehaviorName(ServoLogic::COLOR_BLUE));
	Serial.print(" | yellow block: ");
	Serial.println(getBlockBehaviorName(ServoLogic::COLOR_YELLOW));

	Serial.print("Sequence: ");
	switch (sequence_step) {
		case SEQUENCE_IDLE:
			Serial.println("IDLE");
			break;
		case SEQUENCE_DROP_OPEN:
			Serial.println("DROP_OPEN");
			break;
		case SEQUENCE_DROP_CLOSE:
			Serial.println("DROP_CLOSE");
			break;
		case SEQUENCE_FLIP_CENTER:
			Serial.println("FLIP_CENTER");
			break;
		case SEQUENCE_FLIP_PRELOAD:
			Serial.println("FLIP_PRELOAD");
			break;
		case SEQUENCE_FLIP:
			Serial.println("FLIP");
			break;
		case SEQUENCE_RESET:
			Serial.println("RESET");
			break;
	}
}

bool startSortSequence(ServoLogic::BlockColor color) {
	if (sequence_step != SEQUENCE_IDLE) {
		return false;
	}

	queued_color = color;
	sequence_step = SEQUENCE_DROP_OPEN;
	sequence_deadline_ms = millis() + DROP_WINDOW_MS;
	stopper.open();

	Serial.print("Sorting sequence started for ");
	Serial.println(getActionName(color));
	return true;
	}

bool startFlipSequence() {
	return startSortSequence(ServoLogic::COLOR_BLUE);
}

bool startStandSequence() {
	return startSortSequence(ServoLogic::COLOR_YELLOW);
}

bool startBlockSortSequence(ServoLogic::BlockColor block_color) {
	if (block_color == ServoLogic::COLOR_UNKNOWN) {
		return false;
	}

	bool should_stand =
		(current_team == TEAM_BLUE && block_color == ServoLogic::COLOR_BLUE) ||
		(current_team == TEAM_YELLOW && block_color == ServoLogic::COLOR_YELLOW);
	return should_stand ? startStandSequence() : startFlipSequence();
}

void handleSequence() {
	if (sequence_step == SEQUENCE_IDLE) {
		return;
	}

	unsigned long now = millis();
	if (now < sequence_deadline_ms) {
		return;
	}

	switch (sequence_step) {
		case SEQUENCE_DROP_OPEN:
			stopper.close();
			sequence_step = SEQUENCE_DROP_CLOSE;
			sequence_deadline_ms = now + STOPPER_SETTLE_MS;
			Serial.println("Stopper closed to keep only one block in the chute");
			break;
		case SEQUENCE_DROP_CLOSE:
			flipper.center();
			sequence_step = SEQUENCE_FLIP_CENTER;
			sequence_deadline_ms = now + FLIP_PRELOAD_MS;
			Serial.println("Flipper moved to center before receiving the block");
			break;
		case SEQUENCE_FLIP_CENTER:
			flipper.setAngle(FLIP_PRELOAD_ANGLE);
			sequence_step = SEQUENCE_FLIP_PRELOAD;
			sequence_deadline_ms = now + FLIP_PRELOAD_MS;
			Serial.println("Flipper moved to preload angle 80");
			break;
		case SEQUENCE_FLIP_PRELOAD:
			flipper.flipForColor(queued_color);
			sequence_step = SEQUENCE_FLIP;
			sequence_deadline_ms = now + FLIP_SETTLE_MS;
			Serial.print("Flipping block toward ");
			Serial.println(flipper.getLastColorName());
			break;
		case SEQUENCE_FLIP:
			flipper.center();
			sequence_step = SEQUENCE_RESET;
			sequence_deadline_ms = now + STOPPER_SETTLE_MS;
			Serial.println("Flipper centered and ready for the next block");
			break;
		case SEQUENCE_RESET:
			sequence_step = SEQUENCE_IDLE;
			queued_color = ServoLogic::COLOR_UNKNOWN;
			Serial.println("Sorting sequence complete");
			printStatus();
			break;
		case SEQUENCE_IDLE:
			break;
	}
	}

void handleCommand(String command) {
	command.trim();
	command.toLowerCase();

	if (command.length() == 0) {
		return;
	}

	if (command == "flip") {
		if (!startFlipSequence()) {
			Serial.println("Sequence already running");
		}
		return;
	}

	if (command == "stand") {
		if (!startStandSequence()) {
			Serial.println("Sequence already running");
		}
		return;
	}

	if (command == "blue") {
		if (!startBlockSortSequence(ServoLogic::COLOR_BLUE)) {
			Serial.println("Sequence already running");
		}
		return;
	}

	if (command == "yellow") {
		if (!startBlockSortSequence(ServoLogic::COLOR_YELLOW)) {
			Serial.println("Sequence already running");
		}
		return;
	}

	if (command == "team blue") {
		current_team = TEAM_BLUE;
		Serial.println("Team set to BLUE");
		printStatus();
		return;
	}

	if (command == "team yellow") {
		current_team = TEAM_YELLOW;
		Serial.println("Team set to YELLOW");
		printStatus();
		return;
	}

	if (command == "center") {
		flipper.center();
		Serial.println("Flipper centered");
		return;
	}

	if (command == "open") {
		stopper.open();
		Serial.println("Stopper opened");
		return;
	}

	if (command == "close") {
		stopper.close();
		Serial.println("Stopper closed");
		return;
	}

	if (command == "status") {
		printStatus();
		return;
	}

	if (command == "help") {
		printHelp();
		return;
	}

	Serial.print("Unknown command: ");
	Serial.println(command);
	printHelp();
}
}

void setup() {
	Serial.begin(115200);
	delay(1000);

	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);

	flipper.begin();
	stopper.begin();

	Serial.println("ServoLogic sorter ready");
	printHelp();
	printStatus();
}

void loop() {
	handleSequence();

	if (Serial.available() > 0) {
		String command = Serial.readStringUntil('\n');
		handleCommand(command);
	}
}
