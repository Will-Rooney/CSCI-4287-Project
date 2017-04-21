/*
Name:    DStarSearch_Arduino.ino
Created: 4/20/2017 10:51:06 AM
Author:  Will
*/

#include <dstar.h>

#define ledPin 13

struct tempCar {
	byte row;
	byte col;
};


DStar search;
struct tempCar car;

byte action = BYTE_INF;
byte actionCount = BYTE_INF;
byte stepCount = 0;
byte stepsToObstacle = 2;

bool ACTION_COMPLETE = true;
bool FAIL = false;
bool GOAL = false;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	pinMode(ledPin, OUTPUT);
	Serial.println("Init");
	search.init();
	search.computeShortestPath();
	car.row = 0;
	car.col = 0;
}

// the loop function runs over and over again until power down or reset
void loop() {
	// check if we are at the goal
	//Serial.println("New loop");
	if (GOAL) {
		digitalWrite(ledPin, HIGH);
		delay(500);
		digitalWrite(ledPin, LOW);
		delay(500);
	}
	else if (FAIL) {
		digitalWrite(ledPin, HIGH);
		delay(50);
		digitalWrite(ledPin, LOW);
		delay(50);
	}
	else if (!GOAL) {
		digitalWrite(ledPin, HIGH);
		delay(1000);
		stepCount++;

		// if we've completed the action
		if (stepCount == actionCount)
			ACTION_COMPLETE = true;

		// Get the next action to take and how many blocks to take it
		if (ACTION_COMPLETE) {
			search.getNextAction(action, actionCount);
			if (action == BYTE_INF) {
				FAIL = true;
				Serial.println("Unable to move!");
				return;
			}
			ACTION_COMPLETE = false;
			stepCount = 0;
			Serial.println();
			Serial.print("New action: ");
			switch (action) {
			case 0: // up
				Serial.print("'UP'");
				break;
			case 1: // right
				Serial.print("'RIGHT'");
				break;
			case 2: // down
				Serial.print("'DOWN'");
				break;
			case 3: // left
				Serial.print("'LEFT'");
				break;
			}
			Serial.print("\n\tSteps to take: ");
			Serial.print(actionCount);
			Serial.println();
			Serial.println();
		}

		// Obstacle detected?
		if (stepCount == stepsToObstacle) {
			stepsToObstacle++;
			stepCount = 0;

			// Debugging info
			Serial.print("\nObstacle detected at (");
			switch (action) {
			case 0: // up
				Serial.print(car.row - 1);
				Serial.print(", ");
				Serial.print(car.col);
				Serial.print(")\n");
				break;
			case 1: // right
				Serial.print(car.row);
				Serial.print(", ");
				Serial.print(car.col + 1);
				Serial.print(")\n");
				break;
			case 2: // down
				Serial.print(car.row + 1);
				Serial.print(", ");
				Serial.print(car.col);
				Serial.print(")\n");
				break;
			case 3: // left
				Serial.print(car.row);
				Serial.print(", ");
				Serial.print(car.col - 1);
				Serial.print(")\n");
				break;
			}

			// Check if we need to return to the previous node after recomputing the path
			search.recomputeShortestPath(car.row, car.col, action, actionCount);
			if (actionCount == 0)
				ACTION_COMPLETE = true;
			else
				ACTION_COMPLETE = false;
			return;
		}

		// Move car and update car's position
		Serial.print("Performing action: ");
		switch (action) {
		case 0: // up
			Serial.print("'UP'\n");
			car.row--;
			break;
		case 1: // right
			Serial.print("'RIGHT'\n");
			car.col++;
			break;
		case 2: // down
			Serial.print("'DOWN'\n");
			car.row++;
			break;
		case 3: // left
			Serial.print("'LEFT'\n");
			car.col--;
			break;
		}

		// Goal reached?
		if (car.row == 9 && car.col == 9) {
			GOAL = true;
			Serial.println("Goal!");
		}

		digitalWrite(ledPin, LOW);
		delay(1000);
	}
}
