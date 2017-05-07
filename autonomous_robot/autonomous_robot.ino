/**
   Sketch for the CSCI 4287 team project, April 2017,
   a self driving car project.

   Team members:
      Will Rooney
      Herman Colato
      Soo Kim
      Howard Van Dam

   Hardware components:
      Arduino Mega
      Arduino Motor Sheild
      Adafruit VL6180x
      Adafruit VL5130X
      Adafruit HC-SR04
      Sparkfun MPU9250
*/

#include <Wire.h>
#include <NewPing.h>

extern "C" {
#include "utility/twi.h"
}

//#include "Adafruit_VL53L0X.h"
#include <VL53L0X.h>
#include "Adafruit_VL6180X.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "ArduinoMotorShieldR3.h"
#include "dstar.h"


/********************************************************
  TEST_COMPONENTS:   1 to run the sensor/motor checks
                     0 to skip
*********************************************************/
#define TEST_COMPONENTS 0

/* LED STATUS PINS */
#define failPin 48
#define goalPin 50
#define bluePin 52

/* MOTOR CONTROL DELAY TIMES (MS) */
#define TURN_DELAY 450
#define TURN_AROUND_DELAY 900
#define FWD_DELAY 600

/* MOTOR SHIELD */
ArduinoMotorShieldR3 md;

/* MOTOR SPEEDS */
#define MTRSPD_HIGH 350
#define MTRSPD_MED 200
#define MTRSPD_LOW 100

/* Obstacle detection thresholds */
#define OBSTC_ULTRA_DIST 30 // cm
#define OBSTC_LASER_DIST 30 // mm
#define WALL_THRESHOLD 45 // mm

/* MULTIPLEXER DEFINITIONS FOR LASER SENSORS */
#define TCAADDR  0x70     // multiplexer 
#define FRONT    2        // front laser address
#define RIGHT    3        // right laser address
#define LEFT     7        // left laser address
#define MAX_I2C_INPUTS 7  // Max multiplexer I2C addresses

/* LASER SENSORS */
//Adafruit_VL53L0X front = Adafruit_VL53L0X();
VL53L0X front;
Adafruit_VL6180X leftside = Adafruit_VL6180X();
Adafruit_VL6180X rightside = Adafruit_VL6180X();

/* ULTRASONIC DEFINITIONS */
#define TRIGGER_PIN    5
#define ECHO_PIN       6
#define MAX_DISTANCE   200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);   // NewPing setup of pins and maximum distance.


/* IMU */
#define IMU_ADDR 0x90
MPU9250 theIMU;

/* Virtual Car Status - STRUCT */
struct Vehicle {
  byte row;         // 0-9
  byte col;         // 0-9
  byte orientation; // 0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT
};
bool availActions[] = { false, false, false, false };

/* Pathfinding variables and constants */
DStar search;
byte action = BYTE_INF;       // action contains the next action to take (0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT, 255 = BYTE_INF = UNABLE TO REACH GOAL)
byte actionCount = BYTE_INF;  // the number of times the action must be taken (e.g., action == UP && actionCount == 3: Move 3 squares up or move up 3 ft.)
bool FAIL = false;            // is the goal unreachable?
bool GOAL = false;            // have we reached the goal?
bool OBSTACLE = false;        // Did we detect an obstacle?
bool ACTION_COMPLETE = true;  // Are we ready for a new action?
struct Vehicle car;           // Keep track of perceived orientation and position

/**/
byte getReorientAct();
void setMotorDir();
void setMotorDirRev();
void reorientAct();
uint8_t getLaserRngLeft();
uint8_t getLaserRngRight();
//uint8_t getLaserRngFront();
uint16_t getLaserRngFront();
String actionToStr();
void performAct();
void tcaselect(uint8_t i);
void initIMU();

/* TESTING FUNCTIONS */
void testUltrasonic();
void testLasers();
void testMotors();
void testIMU();
void testTurns();

void setup() {
  Wire.begin();
  Serial1.begin(9600); // Bluetooth Communication

  /* Path Init */
  search.init();
  search.computeShortestPath();
  Serial1.println("Path Init complete");

  /* Virtual Car Stat Init */
  car.row = 0;
  car.col = 0;
  car.orientation = 2; // DOWN

  /* Init Ultrasonic pins */
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, OUTPUT);

  /* Initialize the laser sensors */
  tcaselect(FRONT);
  Serial1.println("Tcase select 2");
  /*if (!front.begin()) {
    Serial.println("VL53L0X Not Detected");
    }*/
  if (!front.init()) {
    Serial1.println("VL53LOX Failed To Init");
  }
  front.setTimeout(500);
  front.setMeasurementTimingBudget(20000); // set measurement time budget to 20 ms

  tcaselect(RIGHT);
  Serial1.println("Tcase select 3");
  if (!rightside.begin()) {
    Serial1.println("Right VL6180X Not Detected");
  }

  tcaselect(LEFT);
  Serial1.println("Tcase select 7");
  if (!leftside.begin()) {
    Serial1.println("Left VL6180X Not Detected");
  }

  Serial1.println("Sensor Init complete");

  /* Initialize the Motor Shield */
  md.init();

  /* LED Init */
  pinMode(goalPin, OUTPUT);
  pinMode(failPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(goalPin, LOW);
  digitalWrite(failPin, LOW);
  digitalWrite(bluePin, LOW);

  /* IMU Init */
  //initIMU();
}

void loop() {
  //delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  if ( TEST_COMPONENTS ) {
    //Serial1.println("Test Components");
    //testTurns();
    //Serial1.println("Test Curve");
    //testCurve();
    //Serial1.println("Test Forward Action");
    //testFwdAct();
    //testUltrasonic();
    //delay(500);
    //testLasers();
    //delay(500);
    //testMotors();
    //delay(500);
    //testIMU();
    //delay(500);
  }
  else if (GOAL) { // GOAL REACHED
    md.setM1Speed(0); // LEFT
    md.setM2Speed(0); // RIGHT
    digitalWrite(bluePin, LOW);
    digitalWrite(goalPin, HIGH);
    delay(1000);
    digitalWrite(goalPin, LOW);
    delay(1000);
  }
  else if (FAIL) { // CANNOT REACH GOAL
    md.setM1Speed(0); // LEFT
    md.setM2Speed(0); // RIGHT
    digitalWrite(bluePin, LOW);
    digitalWrite(failPin, HIGH);
    delay(50);
    digitalWrite(failPin, LOW);
    delay(50);
  }
  else {
    digitalWrite(bluePin, HIGH);
    /* MAIN */
    if (ACTION_COMPLETE) {
      Serial.print("Virtual Car Stats: \n\trow = ");
      Serial.print(car.row);
      Serial.print("\n\tcol = ");
      Serial.print(car.col);
      Serial.print("\n\torientation = ");
      Serial.println(car.orientation);

      /* If we have finished performing the action - Are we at the goal? */
      /* Goal position (row,col): (9, 9) */
      if (car.row == 9 && car.col == 9) {
        GOAL = true;
        Serial1.println("Goal!");
        return;
      }
      search.getNextAction(action, actionCount); // Get the next action and the number of times to take that action
      search.getActions(availActions);
      if (action == BYTE_INF || actionCount > 10) { // BYTE_INF is a custom globally defined variable part of my pathfinding code: BYTE_INF == 255
        FAIL = true;
        Serial1.print("Cannot reach goal!");
        return;
      }
      ACTION_COMPLETE = false;
      Serial1.print("\nNext Action: ");
      Serial1.print(actionToStr());
      Serial1.print(" | Action count: ");
      Serial1.print(actionCount);
      Serial1.print("\n\n");
    }

    /* REORIENT CAR FOR ACTION */
    //Serial1.println("Reorientation commense");
    reorientAct();

    /* CHECK FOR OBSTACLES */
    md.setM1Speed(0); // LEFT
    md.setM2Speed(0); // RIGHT
    delay(250);
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    long ultra_inches = duration / 74 / 2;
    uint16_t front_range = getLaserRngFront();
    uint8_t node_dist = 12 * (actionCount - 1); // Distance to next node in inches, minus a foot
    if (front_range <= 100 || ultra_inches < node_dist) {
      OBSTACLE = true;
    }

    if (!OBSTACLE) { // No obstacle: move forward
      performAct(); // May still find an obstacle
    }
    if (OBSTACLE) { // Obstacle detected
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
      OBSTACLE = false;
      Serial1.print("Obstacle detected while at (");
      Serial1.print(car.row);
      Serial1.print(", ");
      Serial1.print(car.col);
      Serial1.print(") while facing ");
      Serial1.println(actionToStr());
      search.recomputeShortestPath(car.row, car.col, action, actionCount);
      if (action == BYTE_INF || actionCount > 10) { // BYTE_INF is a custom globally defined variable part of my pathfinding code: BYTE_INF == 255
        FAIL = true;
        Serial1.print("Cannot reach goal!");
        return;
      }
      action = 8;
      /* Check if we need to return to the previous node after recomputing the path */
      //if (actionCount == 0) // We did move from the previous node, therefore we do not need to return
      //ACTION_COMPLETE = true;
      //else { // We started to move to the next node and were blocked, therefore we DO need to return to previous node
      ACTION_COMPLETE = false;
      // Reorient the car
      //Serial1.println("Reorientation commense");
      //reorientAct();
      //}
      return;
    }
  }
  /* END OF LOOP */
}

/* Get reorientation action for the car, based on current action
  (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
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

void setMotorDir() {
  uint8_t range_left = getLaserRngLeft();
  uint8_t range_right = getLaserRngRight();

  if (range_left < WALL_THRESHOLD || range_right < WALL_THRESHOLD) {
    float diff = range_right - range_left;
    //Serial1.println(diff);
    if (diff > 15) {
      Serial1.println("Danger of hitting left wall!");
      // slow down right motor
      md.setM1Speed(-MTRSPD_MED); // RIGHT
      md.setM2Speed(-MTRSPD_HIGH); // LEFT
    }
    else if (diff < -15) {
      Serial1.println("Danger of hitting right wall!");
      // slow down left motor
      md.setM1Speed(-MTRSPD_HIGH); // RIGHT
      md.setM2Speed(-MTRSPD_MED); // LEFT
    }
    else {
      // equalize motor speed
      md.setM1Speed(-MTRSPD_HIGH); // RIGHT
      md.setM2Speed(-MTRSPD_HIGH); // LEFT
    }
  }
  else {
    // equalize motor speed
    md.setM1Speed(-MTRSPD_HIGH); // RIGHT
    md.setM2Speed(-MTRSPD_HIGH); // LEFT
  }
  delay(30);
}

void setMotorDirRev() {
  uint8_t range_left = getLaserRngLeft();
  uint8_t range_right = getLaserRngRight();

  if (range_left < WALL_THRESHOLD || range_right < WALL_THRESHOLD) {
    float diff = range_right - range_left;
    //Serial1.println(diff);
    if (diff > 15) {
      Serial1.println("Danger of hitting left wall!");
      // slow down right motor
      md.setM1Speed(MTRSPD_MED); // RIGHT
      md.setM2Speed(MTRSPD_HIGH); // LEFT
    }
    else if (diff < -15) {
      Serial1.println("Danger of hitting right wall!");
      // slow down left motor
      md.setM1Speed(MTRSPD_HIGH); // RIGHT
      md.setM2Speed(MTRSPD_MED); // LEFT
    }
    else {
      // equalize motor speed
      md.setM1Speed(MTRSPD_HIGH); // RIGHT
      md.setM2Speed(MTRSPD_HIGH); // LEFT
    }
  }
  else {
    // equalize motor speed
    md.setM1Speed(MTRSPD_HIGH); // RIGHT
    md.setM2Speed(MTRSPD_HIGH); // LEFT
  }
  delay(30);
}

void reorientAct() {
  byte orientAction = getReorientAct(); // action needed to correct the car's orientation (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
  if (orientAction != 0) { // Need to reorient
    /* TODO */
    // record current orientation from gyro

    // Reorient the car
    if (orientAction == 1) {
      /* TODO - MOTOR CONTROL */
      // turn the car 90 degrees counter-clockwise
      md.setM1Speed(-MTRSPD_HIGH); // RIGHT
      md.setM2Speed(MTRSPD_HIGH); // LEFT
      delay(TURN_DELAY);

      /* IMU - GYRO */
      // check new orientatino from gyro agains old
      // if beyond threshold -> Control Motors accordingly to correct
      // if abs(new - old) > 90 +- threshold
      // correct by turning clockwise
      // else if abs(new - old) < 90 +- threshold
      // correct by turning counterclockwise

      /* Update Virtual car orientation */
      if (car.orientation != 0)
        car.orientation--;
      else
        car.orientation = 3;
    }
    else if (orientAction == 2) {
      /* TODO - MOTOR CONTROL */
      // turn the car 90 degrees clockwise
      md.setM1Speed(MTRSPD_HIGH); // RIGHT
      md.setM2Speed(-MTRSPD_HIGH); // LEFT
      delay(TURN_DELAY);

      /* IMU - GYRO */
      // check new orientatino from gyro agains old
      // if beyond threshold -> Control Motors accordingly to correct
      // if abs(new - old) > 90 +- threshold
      // correct by turning counterclockwise
      // else if abs(new - old) < 90 +- threshold
      // correct by turning clockwise


      /* Update Virtual car orientation */
      if (car.orientation != 3)
        car.orientation++;
      else
        car.orientation = 0;
    }
    else if (orientAction == 3) {
      // Turn the car around (180 degrees)
      tcaselect( LEFT );
      uint8_t left_range = getLaserRngLeft();
      tcaselect( RIGHT );
      uint8_t right_range = getLaserRngRight();
      float range_diff = right_range - left_range;

      Serial1.println("Turn Around");
      Serial1.print("\tRange Diff: ");
      Serial1.println(range_diff);

      if (range_diff > 20) {
        md.setM1Speed(-MTRSPD_HIGH);
        md.setM2Speed(0); // RIGHT
        delay(TURN_AROUND_DELAY / 2);
        md.setM1Speed(-MTRSPD_HIGH);
        md.setM2Speed(MTRSPD_HIGH);
        delay(TURN_AROUND_DELAY / 2);
      }
      else if (range_diff < -20) {
        md.setM1Speed(0); // LEFT
        md.setM2Speed(-MTRSPD_HIGH);
        delay(TURN_AROUND_DELAY / 2);
        md.setM1Speed(MTRSPD_HIGH);
        md.setM2Speed(-MTRSPD_HIGH);
        delay(TURN_AROUND_DELAY / 2);
      }
      else {
        md.setM1Speed(-MTRSPD_HIGH);
        md.setM2Speed(MTRSPD_HIGH);
        delay(TURN_AROUND_DELAY);
      }

      /* IMU - GYRO */
      // check new orientatino from gyro agains old
      // if beyond threshold -> Control Motors accordingly to correct
      // if abs(new - old) > 180 +- threshold
      // correct by turning counterclockwise
      // else if abs(new - old) < 180 +- threshold
      // correct by turning clockwise

      /* Update Virtual car orientation */
      if (car.orientation > 1)
        car.orientation -= 2;
      else
        car.orientation += 2;
    }
  }
}

void performAct() {
  /* MOTOR CONTROL - Control motors to start performing the action */
  // Maybe check FRONT laser as we make the move; inbetween fragments
  uint16_t front_range;
  uint8_t left_range;
  uint8_t right_range;
  uint8_t maxRange = 175;
  md.setM1Speed(-MTRSPD_HIGH); // LEFT
  md.setM2Speed(-MTRSPD_HIGH); // RIGHT
  if (action < 4) {
    //float ultra_dist_old = sonar.ping_cm();
    //float ultra_dist = ultra_dist_old;
    // move car forward one square
    unsigned int i_delay;
    bool INTERSECTION = true;
    if (car.row == 5 && car.col == 9) {
      for (i_delay = 0; i_delay < 1000; i_delay += 50) {
        setMotorDir();
        delay(20);
      }
    }
    while (car.col != 9) {
      if (availActions[0]) {
        switch (car.orientation) {
          case 1: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
          case 3:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
        }
      }
      if (availActions[1]) {
        switch (car.orientation) {
          case 2: // GOING DOWN
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
          case 0:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
        }
      }
      if (availActions[2]) {
        switch (car.orientation) {
          case 3: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
          case 1:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
        }
      }
      if (availActions[3]) {
        switch (car.orientation) {
          case 0: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
          case 2:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(50);
            }
            else {
              INTERSECTION = false;
            }
            break;
        }
      }
      if (!INTERSECTION) {
        delay(500);
        break;
      }
    }
    //for (i_delay = 0; i_delay <= FWD_DELAY; i_delay += 50) {
    while (1) {
      setMotorDir(); // 30 ms delay inisde
      //front_range = getLaserRngFront(); // Front laser uses 20 ms sample
      front_range = getLaserRngFront();
      if (car.row == 5 && car.col == 9) {
        if (front_range < 50) // action complete
          break;
      }
      if (front_range < OBSTC_LASER_DIST) {
        OBSTACLE = true;
        // equalize motor speed
        md.setM1Speed(MTRSPD_HIGH); // LEFT
        md.setM2Speed(MTRSPD_HIGH); // RIGHT
        return;
      }

      if (availActions[0]) {
        switch (car.orientation) {
          case 1: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
          case 3:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }
      if (availActions[1]) {
        switch (car.orientation) {
          case 2: // GOING DOWN
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
          case 0:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }
      if (availActions[2]) {
        switch (car.orientation) {
          case 3: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
          case 1:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }
      if (availActions[3]) {
        switch (car.orientation) {
          case 0: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
          case 2:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }

      if (INTERSECTION) {
        Serial1.println("Intersection detected");
        break;
      }

    }
    // Update car position
    switch (car.orientation) {
      case 0:
        car.row -= actionCount;
        break;
      case 1:
        car.col += actionCount;
        break;
      case 2:
        car.row += actionCount;
        break;
      case 3:
        car.col -= actionCount;
        break;
    }
    ACTION_COMPLETE = true;
  } else if (action == 4) {
    // Moving from node #2 to node #5
    // Car's current orientation is already at "RIGHT"

    // Record current orientation from gyro (degrees)
    uint8_t range_right;
    range_right = getLaserRngRight();
    while (range_right > 200) {
      delay(50);
      range_right = getLaserRngRight();
    }
    while (1) {
      //delay(250);
      setMotorDir();
      front_range = getLaserRngFront();
      if (front_range < OBSTC_LASER_DIST) {
        OBSTACLE = true;
        return;
      }
      range_right = getLaserRngRight();
      //ultra_dist = sonar.ping_cm();
      Serial1.print("Right: ");
      Serial1.print(range_right);
      Serial1.println(" mm");
      //Serial1.print(ultra_dist);
      //Serial1.println(" cm");
      //delay(500);
      // Realy need to check (Orientation Change > 90) from gyro (degrees)
      if (range_right > 200) {
        md.setM1Speed(-MTRSPD_LOW); // LEFT
        md.setM2Speed(-MTRSPD_HIGH); // RIGHT
        delay(500); // move forward a bit into node
        md.setM1Speed(-MTRSPD_HIGH); // LEFT
        md.setM2Speed(-MTRSPD_HIGH); // RIGHT
        delay(250);
        ACTION_COMPLETE = true;
        break;
      }
      //delay(50);
    }
    // update car's position
    car.row = 2;
    car.col = 7;
    car.orientation = 3; // LEFT
    ACTION_COMPLETE = true;
    return;
  } else if (action == 5) {
    // TODO motor control
    // Moving from node #5 to node #2
    // Car's current orientation is already at "UP"
    // verify action complete with sensor data
    uint8_t range_left;
    float ultra_dist;
    while (1) {
      setMotorDir();
      front_range = getLaserRngFront();
      if (front_range < OBSTC_LASER_DIST) {
        OBSTACLE = true;
        return;
      }
      range_left = getLaserRngLeft();
      ultra_dist = sonar.ping_cm();
      if (range_left >= 100 && ultra_dist > 75) {
        md.setM1Speed(-400); // LEFT
        md.setM2Speed(-400); // RIGHT
        break;
      }
    }
    // update car's position
    car.row = 0;
    car.col = 3;
    car.orientation = 3; // LEFT
    ACTION_COMPLETE = true;
    return;
  } else if (action == 6) {
    // Turn the car around (180 degrees)
    md.setM1Speed(-400); // LEFT
    md.setM2Speed(400); // RIGHT
    delay(TURN_AROUND_DELAY);

    uint8_t range_left;
    float ultra_dist;
    while (1) {
      range_left = getLaserRngLeft();
      ultra_dist = sonar.ping_cm();
      if (range_left >= 100 && ultra_dist > 75) {
        md.setM1Speed(-400); // LEFT
        md.setM2Speed(-400); // RIGHT
        break;
      }
      setMotorDir();
      front_range = getLaserRngFront();
      if (front_range < OBSTC_LASER_DIST) {
        OBSTACLE = true;
        return;
      }
    }

    // update car's position
    car.row = 0;
    car.col = 3;
    car.orientation = 3; // LEFT
    ACTION_COMPLETE = true;
    return;
  } else if (action == 7) {
    // Turn the car around (180 degrees)
    md.setM1Speed(-400); // LEFT
    md.setM2Speed(400); // RIGHT
    delay(TURN_AROUND_DELAY);

    uint8_t range_right;
    float ultra_dist;
    while (1) {
      range_right = getLaserRngRight();
      ultra_dist = sonar.ping_cm();
      if (range_right >= 100 || ultra_dist > 90) {
        md.setM1Speed(-400); // LEFT
        md.setM2Speed(-400); // RIGHT
        break;
      }
      setMotorDir();
      front_range = getLaserRngFront();
      if (front_range < OBSTC_LASER_DIST) {
        OBSTACLE = true;
        return;
      }
    }

    // update car's position
    car.row = 2;
    car.col = 7;
    car.orientation = 2; // DOWN
    ACTION_COMPLETE = true;
    return;
  }
  else if (action == 8) {
    bool INTERSECTION = false;
    md.setM1Speed(MTRSPD_HIGH); // LEFT
    md.setM2Speed(MTRSPD_HIGH); // RIGHT

    //for (i_delay = 0; i_delay <= FWD_DELAY; i_delay += 50) {
    while (1) {
      setMotorDirRev(); // 30 ms delay inisde

      if (availActions[0]) {
        switch (car.orientation) {
          case 1: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDirRev();
              //delay(200);
              INTERSECTION = true;
            }
            break;
          case 3:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDirRev();
              //delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }
      if (availActions[1]) {
        switch (car.orientation) {
          case 2: // GOING DOWN
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDirRev();
              //delay(200);
              INTERSECTION = true;
            }
            break;
          case 0:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDir();
              //delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }
      if (availActions[2]) {
        switch (car.orientation) {
          case 3: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDirRev();
              //delay(200);
              INTERSECTION = true;
            }
            break;
          case 1:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDirRev();
              //delay(200);
              INTERSECTION = true;
            }
            break;
        }
      }
      if (availActions[3]) {
        switch (car.orientation) {
          case 0: // GOING RIGHT
            left_range = getLaserRngLeft();
            if (left_range > maxRange) {
              setMotorDirRev();
              //delay(50);
              INTERSECTION = true;
            }
            break;
          case 2:
            right_range = getLaserRngRight();
            if (right_range > maxRange) {
              setMotorDirRev();
              //delay(50);
              INTERSECTION = true;
            }
            break;
        }
      }

      if (INTERSECTION) {
        Serial1.println("Intersection detected");
        delay(100);
        break;
      }
    }
    ACTION_COMPLETE = true;
  }
}

uint8_t getLaserRngLeft() {
  tcaselect( LEFT );
  uint8_t range = leftside.readRange();
  uint8_t status = leftside.readRangeStatus();
  if (status == VL6180X_ERROR_NONE) {
    //Serial1.print("Left Range: ");
    //Serial1.println(range);
    return range;
  }
  else {
    //Serial1.println("Out of range");
    return 250;
  }
}

uint8_t getLaserRngRight() {
  tcaselect( RIGHT );
  uint8_t range = rightside.readRange();
  uint8_t status = rightside.readRangeStatus();
  if (status == VL6180X_ERROR_NONE) {
    //Serial1.print("Right Range: ");
    //Serial1.println(range);
    return range;
  }
  else {
    //Serial1.println("Out of range");
    return 250;
  }
}

uint16_t getLaserRngFront() {
  tcaselect( FRONT );
  uint16_t range = front.readRangeSingleMillimeters();
  return range;
}

String actionToStr() {
  switch (action) {
    case 0:
      return "Up";
    case 1:
      return "Right";
    case 2:
      return "Down";
    case 3:
      return "Left";
    case 4:
      return "Curve: 2 -> 5";
    case 5:
      return "Curve: 5 -> 2";
    case 6:
      return "Curve & Obstc: Return -> 2";
    case 7:
      return "Curve & Obstc: Return -> 5";
  }
}

/**
  Multiplexer Addressing function
*/
void tcaselect(uint8_t i) {
  if (i > MAX_I2C_INPUTS) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

/**
  Test functions
*/

void testTurns() {
  Serial1.println("Testing 90 degree turn left");
  md.setM1Speed(-MTRSPD_HIGH); // RIGHT
  md.setM2Speed(MTRSPD_HIGH); // LEFT
  delay(TURN_DELAY);
  md.setM1Speed(0); // RIGHT
  md.setM2Speed(0); // LEFT
  delay(2000);

  Serial1.println("Testing 90 degree turn right");
  md.setM1Speed(MTRSPD_HIGH); // RIGHT
  md.setM2Speed(-MTRSPD_HIGH); // LEFT
  delay(TURN_DELAY);
  md.setM1Speed(0); // RIGHT
  md.setM2Speed(0); // LEFT
  delay(2000);

  Serial1.println("Testing 180 degree turn right");
  md.setM1Speed(MTRSPD_HIGH); // RIGHT
  md.setM2Speed(-MTRSPD_HIGH); // LEFT
  delay(TURN_AROUND_DELAY);
  md.setM1Speed(0); // RIGHT
  md.setM2Speed(0); // LEFT
  delay(2000);
}

void testCurve() {
  ACTION_COMPLETE = false;
  while (1) {
    if (ACTION_COMPLETE) {
      md.setM1Speed(0); // LEFT
      md.setM2Speed(0); // RIGHT
      digitalWrite(goalPin, HIGH);
      delay(50);
      digitalWrite(goalPin, LOW);
      delay(50);
    }
    else if (!OBSTACLE) {
      action = 4;
      performAct();
    }
    else if (OBSTACLE) {
      md.setM1Speed(0); // LEFT
      md.setM2Speed(0); // RIGHT
      digitalWrite(failPin, HIGH);
      delay(50);
      digitalWrite(failPin, LOW);
      delay(50);
    }
  }
}

void testFwdAct() {
  ACTION_COMPLETE = false;
  while (1) {
    if (ACTION_COMPLETE) {
      md.setM1Speed(0); // LEFT
      md.setM2Speed(0); // RIGHT
      digitalWrite(goalPin, HIGH);
      delay(50);
      digitalWrite(goalPin, LOW);
      delay(50);
    }
    else if (!OBSTACLE) {
      action = 1;
      performAct();
      if (!OBSTACLE)
        ACTION_COMPLETE = true;
    }
    else if (OBSTACLE) {
      md.setM1Speed(0); // LEFT
      md.setM2Speed(0); // RIGHT
      digitalWrite(failPin, HIGH);
      delay(50);
      digitalWrite(failPin, LOW);
      delay(50);
    }
  }
}

void testUltrasonic() {

  for ( int i = 1; i < 20; i++ )
  {
    delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial1.print("Ultrasonic: ");
    Serial1.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial1.println("cm");
    delay(100);
  }
}

void testLasers() {

  //VL53L0X_RangingMeasurementData_t measure;
  //float luxleft;
  //float luxright;
  int i = 0;

  /*tcaselect( FRONT );
    for ( int i = 1; i < 20; i++ )
    {
    front.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
    Serial1.print("Front laser Distance (mm): ");
    Serial1.println(measure.RangeMilliMeter);
    }
    else {
    Serial1.println("Out Of Range");
    }
    delay(50);
    }  // end for*/
  tcaselect( FRONT );
  Serial1.println("Testing Lasers");
  for (i = 0; i < 20; ++i) {
    Serial1.print("Front Laser: ");
    Serial1.print(front.readRangeSingleMillimeters());
    if (front.timeoutOccurred()) {
      Serial.print(" TIMEOUT");
    }
    //uint16_t front_range = front.readRangeSingleMillimeters();
    //Serial1.print(getLaserRngFront());
    Serial1.println(" mm");
    delay(50);
  }

  tcaselect( LEFT );
  for ( int i = 1; i < 20; i++ )
  {
    //luxleft = leftside.readLux(VL6180X_ALS_GAIN_5);
    //Serial1.print("Left Lux: ");
    //Serial1.println(luxleft);
    uint8_t range1 = leftside.readRange();
    uint8_t status1 = leftside.readRangeStatus();
    if (status1 == VL6180X_ERROR_NONE) {
      Serial1.print("Left Range: ");
      Serial1.println(range1);
    }
    else
      Serial1.println("Left Laser Error!");
    delay(50);
  }  // end for

  tcaselect( RIGHT );
  for ( int i = 1; i < 20; i++ )
  {
    //luxright = rightside.readLux(VL6180X_ALS_GAIN_5);
    //Serial.print("Right Lux: ");
    //Serial.println(luxright);
    uint8_t range2 = rightside.readRange();
    uint8_t status2 = rightside.readRangeStatus();
    if (status2 == VL6180X_ERROR_NONE) {
      Serial1.print("Right Range: ");
      Serial1.println(range2);
    }
    else
      Serial1.println("Right Laser Error!");
    delay(50);
  }  // end for

  delay(50);
}  // end testLasers

void testMotors() {

  Serial1.println("M1 Speed 100 % Forward");
  md.setM1Speed(400);
  Serial1.println("M2 Speed 100 % Forward");
  md.setM2Speed(400);
  Serial1.print("M1 current: ");
  Serial1.println(md.getM1CurrentMilliamps());
  Serial1.print("M2 current: ");
  Serial1.println(md.getM2CurrentMilliamps());
  delay(2000);

  Serial1.println("M1 Speed 100 % Backward");
  md.setM1Speed(-400);
  Serial1.println("M2 Speed 100 % Backward");
  md.setM2Speed(-400);
  Serial1.print("M1 current: ");
  Serial1.println(md.getM1CurrentMilliamps());
  Serial1.print("M2 current: ");
  Serial1.println(md.getM2CurrentMilliamps());
  delay(2000);

  Serial1.println("M1 Speed 50 % Forward");
  md.setM1Speed(200);
  Serial1.println("M2 Speed 50 % Forward");
  md.setM2Speed(200);
  Serial1.print("M1 current: ");
  Serial1.println(md.getM1CurrentMilliamps());
  Serial1.print("M2 current: ");
  Serial1.println(md.getM2CurrentMilliamps());
  delay(2000);

  Serial1.println("M1 Speed 50 % Backward");
  md.setM1Speed(-200);
  Serial1.println("M2 Speed 50 % Backward");
  md.setM2Speed(-200);
  Serial1.print("M1 current: ");
  Serial1.println(md.getM1CurrentMilliamps());
  Serial1.print("M2 current: ");
  Serial1.println(md.getM2CurrentMilliamps());
  delay(2000);

  Serial1.println("M1 Speed 0 % ");
  md.setM1Speed(0);
  Serial1.println("M2 Speed 0 % ");
  md.setM2Speed(0);
  Serial1.print("M1 current: ");
  Serial1.println(md.getM1CurrentMilliamps());
  Serial1.print("M2 current: ");
  Serial1.println(md.getM2CurrentMilliamps());
  delay(2000);

}  // end testMotors

void initIMU() {
  byte c = theIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial1.print("MPU9250 "); Serial1.print("I AM "); Serial1.print(c, HEX);
  Serial1.print(" I should be "); Serial1.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial1.println("MPU9250 is online...");
  } // if (c == 0x71)
  else
  {
    Serial1.print("Could not connect to MPU9250: 0x");
    Serial1.println(c, HEX);
  }
}
void testIMU() {

  byte c = theIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial1.print("MPU9250 "); Serial1.print("I AM "); Serial1.print(c, HEX);
  Serial1.print(" I should be "); Serial1.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial1.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    theIMU.MPU9250SelfTest(theIMU.SelfTest);
    Serial1.print("x - axis self test: acceleration trim within : ");
    Serial1.print(theIMU.SelfTest[0], 1); Serial1.println(" % of factory value");
    Serial1.print("y - axis self test: acceleration trim within : ");
    Serial1.print(theIMU.SelfTest[1], 1); Serial1.println(" % of factory value");
    Serial1.print("z - axis self test: acceleration trim within : ");
    Serial1.print(theIMU.SelfTest[2], 1); Serial1.println(" % of factory value");
    Serial1.print("x - axis self test: gyration trim within : ");
    Serial1.print(theIMU.SelfTest[3], 1); Serial1.println(" % of factory value");
    Serial1.print("y - axis self test: gyration trim within : ");
    Serial1.print(theIMU.SelfTest[4], 1); Serial1.println(" % of factory value");
    Serial1.print("z - axis self test: gyration trim within : ");
    Serial1.print(theIMU.SelfTest[5], 1); Serial1.println(" % of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    theIMU.calibrateMPU9250(theIMU.gyroBias, theIMU.accelBias);

    theIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial1.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = theIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial1.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial1.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    theIMU.initAK8963(theIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial1.println("AK8963 initialized for active data mode....");

    //  Serial.println("Calibration values : ");
    Serial1.print("X - Axis sensitivity adjustment value ");
    Serial1.println(theIMU.magCalibration[0], 2);
    Serial1.print("Y - Axis sensitivity adjustment value ");
    Serial1.println(theIMU.magCalibration[1], 2);
    Serial1.print("Z - Axis sensitivity adjustment value ");
    Serial1.println(theIMU.magCalibration[2], 2);

  } // if (c == 0x71)
  else
  {
    Serial1.print("Could not connect to MPU9250 : 0x");
    Serial1.println(c, HEX);
  }
}  // end test IMU
