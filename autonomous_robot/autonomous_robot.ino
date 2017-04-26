/*
   Name: autonomous_robot.ino
   Created: 4/25/2017
*/

#include "dstar.h"

#define ledPin 13

DStar search;

struct Vehicle {
  byte row;         // 0-9
  byte col;         // 0-9
  byte orientation; // 0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT
};

byte action = BYTE_INF;       // action contains the next action to take (0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT, 255 = BYTE_INF = UNABLE TO REACH GOAL)
byte actionCount = BYTE_INF;  // the number of times the action must be taken (e.g., action == UP && actionCount == 3: Move 3 squares up or move up 3 ft.)
byte orientAction = 0;        // action needed to correct the car's orientation (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)

bool FAIL = false;            // is the goal unreachable?
bool GOAL = false;            // have we reached the goal?
bool OBSTACLE = false;        // Did we detect an obstacle?
bool ACTION_COMPLETE = true;  // Are we ready for a new action?

struct Vehicle car;

byte getReorientAct();

void setup() {
  /* Path Init */
  search.init();
  search.computeShortestPath();

  /* Virtual Car Stat Init */
  car.row = 0;
  car.col = 0;
  car.orientation = 1; // RIGHT

  /* Sensor & Other Init */
}

void loop() {
  if (GOAL) { // GOAL REACHED
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
  else if (FAIL) { // CANNOT REACH GOAL
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
  }
  else {
    /* MAIN */

    /* IF READY FOR NEXT ACTION */
    /* USE TIME and/or IMU data to determine if we've moved one square */
    // If so then update car's position
    /* USE TIME and/or IMU data to determine if we've moved a total of 'actionCount' squares */
    // If so then ACTION_COMPLETE = true

    if (ACTION_COMPLETE) {
      /* If we have finished performing the action - Are we at the goal? */
      /* Goal position (row,col): (9, 9) */
      if (car.row == 9 && car.col == 9) {
        GOAL = true;
        Serial.println("Goal!");
        return;
      }

      search.getNextAction(action, actionCount); // Get the next action and the number of times to take that action
      if (action == BYTE_INF || actionCount > 10) { // BYTE_INF is a custom globally defined variable part of my pathfinding code: BYTE_INF == 255
        FAIL = true;
        return;
      }
    }
    /* REORIENT CAR FOR ACTION - MOTOR CONTROL - TODO */
    orientAction = getReorientAct();
    if (orientAction != 0) { // Need to reorient
      // Reorient the car
      if (orientAction == 1) {
        /* TODO - MOTOR CONTROL */
        // turn the car 90 degrees counter-clockwise

        /* Update Virtual car orientation */
        if (car.orientation != 0)
          car.orientation--;
        else
          car.orientation = 3;
      }
      else if (orientAction == 2) {
        /* TODO - MOTOR CONTROL */
        // turn the car 90 degrees clockwise

        /* Update Virtual car orientation */
        if (car.orientation != 3)
          car.orientation++;
        else
          car.orientation = 0;
      }
      else if (orientAction == 3) {
        /* TODO - MOTOR CONTROL */
        // Turn the car around (180 degrees)

        /* Update Virtual car orientation */
        if (car.orientation > 1)
          car.orientation -= 2;
        else
          car.orientation += 2;
      }
      /* IMU - GYRO */
      // Verify orientation action with Gyro data
    }
  }

  /* MOTOR CONTROL - Control motors to start performing the action */
  /* GATHER SENSOR DATA */
  /* CHECK FOR OBSTACLES */
  /* CHECK IF WE NEED TO RECENTER CAR - MOTOR CONTROL */

  /* Obstacle detected? */
  if (OBSTACLE) {
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
          // turn the car 90 degrees counter-clockwise

          /* Update Virtual car orientation */
          if (car.orientation != 0)
            car.orientation--;
          else
            car.orientation = 3;
        }
        else if (orientAction == 2) {
          /* TODO - MOTOR CONTROL */
          // turn the car 90 degrees clockwise

          /* Update Virtual car orientation */
          if (car.orientation != 3)
            car.orientation++;
          else
            car.orientation = 0;
        }
        else if (orientAction == 3) {
          /* TODO - MOTOR CONTROL */
          // Turn the car around (180 degrees)

          /* Update Virtual car orientation */
          if (car.orientation > 1)
            car.orientation -= 2;
          else
            car.orientation += 2;
        }

        /* IMU - GYRO */
        // Verify orientation action with Gyro data
      }
    }
    return;
  }
  /* END OF LOOP */
}

/* Get reorientation action for the car, based on current action
 * (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
 */
byte getReorientAct() {
  byte orientation = car.orientation;
  if (orientation == action)
    return 0; // Already in the correct orientation
  if (actionCount > 10)
    return 0; // Can't reach goal - why reorient

  switch (orientation) {
    case 0: // car -> UP
      switch (action) {
        case 1: // action -> RIGHT
          return 2; // Turn right
        case 2: // action -> DOWN
          return 3; // Turn the car around (180 degrees)
        case 3: // action -> LEFT
          return 1; // Turn the car left
        case 4: // action -> RIGHT
          return 2; // Turn right
      }
      break;
    case 1: // car -> RIGHT
      switch (action) {
        case 0: // action -> UP
          return 1; // Turn the car left
        case 2: // action -> DOWN
          return 2; // Turn right
        case 3: // action -> LEFT
          return 3; // Turn the car around (180 degrees)
        case 5: // action -> UP
          return 1; // Turn the car left
      }
      break;
    case 2:
      switch (action) {
        case 0: // action -> UP
          return 3; // Turn the car around (180 degrees)
        case 1: // action -> RIGHT
          return 1; // Turn the car left
        case 3: // action -> LEFT
          return 2; // Turn right
        case 4: // action -> RIGHT
          return 1; // Turn the car left
        case 5: // action -> UP
          return 3; // Turn the car around (180 degrees)
      }
      break;
    case 3:
      switch (action) {
        case 0: // action -> UP
          return 2; // Turn right
        case 1: // action -> RIGHT
          return 3; // Turn the car around (180 degrees)
        case 2: // action -> DOWN
          return 1; // Turn the car left
        case 4: // action -> RIGHT
          return 3; // Turn the car around (180 degrees)
        case 5: // action -> UP
          return 2; // Turn right
      }
      break;
  }
  return 0;
}
