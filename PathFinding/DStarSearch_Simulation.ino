/*
  Name:    DStarSearch_Arduino.ino
  Created: 4/20/2017 10:51:06 AM
  Author:  Will
*/

#include "dstar.h"

#define ledPin 13

// SIMULATION STRUCT
struct tempCar {
  byte row;         // 0-9
  byte col;         // 0-9
  byte orientation; // 0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT
};


DStar search;                 // Create a new pathfinding object called 'search'
struct tempCar car;           // SIMULATION object

byte action = BYTE_INF;       // action contains the next action to take (0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT, 255 = BYTE_INF = UNABLE TO REACH GOAL)
byte actionCount = BYTE_INF;  // the number of times the action must be taken (e.g., action == UP && actionCount == 3: Move 3 squares up or move up 3 ft.)
byte orientAction = 0;        // action needed to correct the car's orientation (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
byte stepCount = 0;           // SIMULATION - for keeping track of the cars position
byte stepsToObstacle = 3;     // SIMULATION - for adding in obstacles to the simulation

bool ACTION_COMPLETE = true;  // has the car finished taking the given action? INITIALLY TRUE - action is initialization
bool FAIL = false;            // is the goal unreachable?
bool GOAL = false;            // have we reached the goal?


// Get reorientation action for the car, based on current action
// (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
byte getReorientAct() {
  byte orientation = car.orientation;
  if (orientation == action)
    return 0; // Already in the correct orientation
  if (actionCount > 10)
    return 0; // Can't reach goal - why reorient - May never be true

  if (orientation == 0) {// car -> UP
    if (action == 1) { // action -> RIGHT
      return 2; // Turn right
    }
    else if (action == 2) { // action -> DOWN
      return 3; // Turn the car around (180 degrees)
    }
    else if (action == 3) { // action -> LEFT
      return 1; // Turn the car left
    }
    else if (action == 4) { // action -> RIGHT
      return 2; // Turn right
    }
  }
  else if (orientation == 1) {// car -> RIGHT
    if (action == 0) { // action -> UP
      return 1; // Turn the car left
    }
    else if (action == 2) { // action -> DOWN
      return 2; // Turn right
    }
    else if (action == 3) { // action -> LEFT
      return 3; // Turn the car around (180 degrees)
    }
    else if (action == 5) { // action -> UP
      return 1; // Turn the car left
    }
  }
  else if (orientation == 2) {// car -> DOWN
    if (action == 0) { // action -> UP
      return 3; // Turn the car around (180 degrees)
    }
    else if (action == 1) { // action -> RIGHT
      return 1; // Turn the car left
    }
    else if (action == 3) { // action -> LEFT
      return 2; // Turn right
    }
    else if (action == 4) { // action -> RIGHT
      return 1; // Turn the car left
    }
    else if (action == 5) { // action -> UP
      return 3; // Turn the car around (180 degrees)
    }
  }
  else if (orientation == 3) {// car -> LEFT
    if (action == 0) { // action -> UP
      return 2; // Turn right
    }
    else if (action == 1) { // action -> RIGHT
      return 3; // Turn the car around (180 degrees)
    }
    else if (action == 2) { // action -> DOWN
      return 1; // Turn the car left
    }
    else if (action == 4) { // action -> RIGHT
      return 3; // Turn the car around (180 degrees)
    }
    else if (action == 5) { // action -> UP
      return 2; // Turn right
    }
  }
  else
    return 0; // Car's stored orientation is invalid
}

/***** SETUP *****/
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  Serial.println("Init");

  // Initialize pathfinding and compute initial path
  search.init();
  search.computeShortestPath();

  // SIMULATION - Initialize sim-car
  // Start position: (0, 0)
  car.row = 0;
  car.col = 0;
  car.orientation = 1; // RIGHT
}

/***** LOOP *****/
void loop() {
  if (GOAL) { // GOAL REACHED
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
  else if (FAIL) { // CANNOT REACH GOAL
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
  }
  else if (!GOAL) { // MAIN LOOP - Navigate to the goal
    digitalWrite(ledPin, HIGH);
    delay(1000);
    stepCount++; // The number of steps taken since the last action was completed

    // if we've completed the action
    if (stepCount == actionCount) // This check will be done by checking sensor data to verify car's position
      ACTION_COMPLETE = true;

    // Get the next action to take and how many blocks to take it
    if (ACTION_COMPLETE) {
      search.getNextAction(action, actionCount); // Get the next action and the number of times to take that action
      ACTION_COMPLETE = false;  // New action -> reset ACTION_COMPLETE
      stepCount = 0;            // SIMULATION New action -> reset stepCount
      if (action == BYTE_INF || actionCount > 10) { // BYTE_INF is a custom globally defined variable part of my pathfinding code: BYTE_INF == 255
        FAIL = true;
        Serial.println("Unable to reach the goal!");
        return;
      }

      // Determine if the car needs to reorient for the next action
      orientAction = getReorientAct();
      if (orientAction != 0) { // Need to reorient
        // Reorient the car
        if (orientAction == 1) {
          /* TODO - MOTOR CONTROL */
          // turn the car left
          Serial.println("\nTurning the car 90 degrees counter-clockwise");
          if (car.orientation != 0)
            car.orientation--;
          else
            car.orientation = 3;
        }
        else if (orientAction == 2) {
          /* TODO - MOTOR CONTROL */
          // turn the car right
          Serial.println("\nTurning the car 90 degrees clockwise");
          if (car.orientation != 3)
            car.orientation++;
          else
            car.orientation = 0;
        }
        else if (orientAction == 3) {
          /* TODO - MOTOR CONTROL */
          // Turn the car around (180 degrees)
          Serial.println("\nTurning the car around 180 degrees");
          if (car.orientation > 1)
            car.orientation -= 2;
          else
            car.orientation += 2;
        }
      }

      /* FOR TESTING PURPOSES -  NOT NEEDED IN FINAL IMPLEMENTATION */
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
        case 4: // RIGHT/DOWN CURVE
          Serial.print("'RIGHT/DOWN CURVE'");
          break;
        case 5: // UP/LEFT CURVE
          Serial.print("'UP/LEFT CURVE'");
          break;
        case 6: // RIGHT/DOWN CURVE REVERSE
          Serial.print("Returning from: 'RIGHT/DOWN CURVE'");
          break;
        case 7: // UP/LEFT CURVE REVERSE
          Serial.print("Returning from: 'UP/LEFT CURVE'");
          break;
      }
      Serial.print("\n\tSteps to take: ");
      Serial.print(actionCount);
      Serial.println();
      Serial.println();
    }

    /* check for obstacles */

    /* Obstacle detected? - SIMULATE AN OBSTACLE POPPING UP */
    if (stepCount == stepsToObstacle) {
      stepsToObstacle++; // Increase the number of steps until we throw in an obstacle

      stepCount = 0;  // Reset step count - new action pending

      /* FOR TESTING PURPOSES -  NOT NEEDED IN FINAL IMPLEMENTATION */
      /* The car will update its location based on motor control and verified with sensor data */
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
        case 4:
          Serial.println("CURVE)");
          stepsToObstacle++;
          break;
        case 5:
          Serial.println("CURVE)");
          stepsToObstacle++;
          break;
      }

      /* recomputeShortestPath(byte row, byte col, byte &action, byte &actionCount)
          pre:
            (row, col) == Cars current position
            search.computeShortestPath() has been executed successfully in setup()

          post:
            if (actionCount > 0)
              action := the action needed to return to the previous node
              actionCount := the number of times that action needs to be taken to return
            else if (actionCount == 0)
              We do not need to return to the previous node, a new action will be retreived
              at the beginning of the loop()
      */
      search.recomputeShortestPath(car.row, car.col, action, actionCount);
      /* Check if we need to return to the previous node after recomputing the path */
      if (actionCount == 0) // We did move from the previous node, therefore we do not need to return
        ACTION_COMPLETE = true;
      else { // We started to move to the next node and were blocked, therefore we DO need to return to previous node
        ACTION_COMPLETE = false;
        // Reorient the car
        orientAction = getReorientAct();
        if (orientAction != 0) { // Need to reorient
          // Reorient the car
          if (orientAction == 1) {
            /* TODO - MOTOR CONTROL */
            // turn the car left
            Serial.println("\nTurning the car 90 degrees counter-clockwise");
            if (car.orientation != 0)
              car.orientation--;
            else
              car.orientation = 3;
          }
          else if (orientAction == 2) {
            /* TODO - MOTOR CONTROL */
            // turn the car right
            Serial.println("\nTurning the car 90 degrees clockwise");
            if (car.orientation != 3)
              car.orientation++;
            else
              car.orientation = 0;
          }
          else if (orientAction == 3) {
            /* TODO - MOTOR CONTROL */
            // Turn the car around (180 degrees)
            Serial.println("\nTurning the car around 180 degrees");
            if (car.orientation > 1)
              car.orientation -= 2;
            else
              car.orientation += 2;
          }
        }
      }
      return;
    }

    /* Move car based on 'action' and 'actionCount', then update car's position */

    /* Check distance sensors and center the car if needed */

    /* FOR TESTING PURPOSES -  NOT NEEDED IN FINAL IMPLEMENTATION */
    /* The car will update its location based on motor control and verified with sensor data */
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
      case 4:
        Serial.print("Following curve from N2 to N5\n");
        //stepCount = actionCount-1;
        car.row = 2;
        car.col = 7;
        car.orientation = 2;
        break;
      case 5:
        Serial.print("Following curve from N5 to N2\n");
        //stepCount = actionCount-1;
        car.row = 0;
        car.col = 3;
        car.orientation = 3;
        break;
      case 6:
        Serial.print("Returning from curve from N2 to N5\n");
        //stepCount = actionCount-1;
        car.row = 0;
        car.col = 3;
        car.orientation = 3;
        break;
      case 7:
        Serial.print("Returning from curve from N5 to N2\n");
        // stepCount = actionCount-1;
        car.row = 2;
        car.col = 7;
        car.orientation = 2;
        break;
    }

    /* If we have finished performing the action - Are we at the goal? */
    /* Goal position (row,col): (9, 9) */
    if (car.row == 9 && car.col == 9) {
      GOAL = true;
      Serial.println("Goal!");
    }

    digitalWrite(ledPin, LOW);
    delay(1000);
  }
}
