/**
 * Sketch for the CSCI 4287 team project, April 2017,
 * a self driving car project.
 * 
 * Team members:
 *    Will Rooney
 *    Herman Colato
 *    Soo Kim
 *    Howard Van Dam
 *    
 * Hardware components:
 *    Arduino Mega
 *    Arduino Motor Sheild
 *    Adafruit VL6180x
 *    Adafruit VL5130X
 *    Adafruit HC-SR04
 *    Sparkfun MPU9250
 */
 
#include <Wire.h>
#include <NewPing.h>

extern "C" {
  #include "utility/twi.h"
}

#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL6180X.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "ArduinoMotorShieldR3.h"
#include "dstar.h"

/** 
 *  Set this define to 1 to run the sensor/motor checks
 *                     0 to skip
 */
#define TEST_COMPONENTS 1

/** 
 *  Constants for sensor addresses and pin assignments
 */

#define ledPin 13
#define TCAADDR  0x70    //  multiplexer 
#define FRONT    0x40
#define LEFT     0x00
#define RIGHT    0x20
#define IMU_ADDR 0x90

#define MAX_I2C_INPUTS 7
#define TRIGGER_PIN    12  
#define ECHO_PIN       11  
#define MAX_DISTANCE   200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Adafruit_VL53L0X front = Adafruit_VL53L0X();
Adafruit_VL6180X leftside = Adafruit_VL6180X();
Adafruit_VL6180X rightside = Adafruit_VL6180X();
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);   // NewPing setup of pins and maximum distance.
MPU9250 theIMU;
ArduinoMotorShieldR3 md;


/**
 *   Pathfinding variables and constants
 */
// SIMULATION STRUCT
struct tempCar {
  byte row;         // 0-9
  byte col;         // 0-9
  byte orientation; // 0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT
};
struct tempCar car;           // SIMULATION 
 
DStar search;                 // Create a new pathfinding object called 'search'

byte action = BYTE_INF;       // action contains the next action to take (0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT, 255 = BYTE_INF = UNABLE TO REACH GOAL)
byte actionCount = BYTE_INF;  // the number of times the action must be taken (e.g., action == UP && actionCount == 3: Move 3 squares up or move up 3 ft.)
byte orientAction = 0;        // action needed to correct the car's orientation (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
byte stepCount = 0;           // SIMULATION - for keeping track of the cars position
byte stepsToObstacle = 2;     // SIMULATION - for adding in obstacles to the simulation

bool ACTION_COMPLETE = true;  // has the car finished taking the given action? INITIALLY TRUE - action is initialization
bool FAIL = false;            // is the goal unreachable?
bool GOAL = false;            // have we reached the goal?

/**
 * 
 */
void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.

  // Initialize the laser sensors
  tcaselect(FRONT);
  if(!front.begin()){
    Serial.println("VL53L0X Not Detected");
  }
  tcaselect(LEFT);
  if(!leftside.begin()){
    Serial.println("Left VL6180X Not Detected");
  }
  tcaselect(RIGHT);
  if(!rightside.begin()){
    Serial.println("Right VL6180X Not Detected");
  }
  //  Initialize the Motor Shield
  md.init();

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
}  // end setup

void loop() {
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

  if( TEST_COMPONENTS ) {
    Serial.print("Testing Sensors: ");
    testUltrasonic();
    testLasers();
    testMotors();
    //   testIMU();
  }
  else  // run the car
  {
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
      if (action == BYTE_INF) { // BYTE_INF is a custom globally defined variable part of my pathfinding code: BYTE_INF == 255
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
        }
        else if (orientAction == 2) {
          /* TODO - MOTOR CONTROL */
          // turn the car right
        }
        else if (orientAction == 3) {
          /* TODO - MOTOR CONTROL */
          // Turn the car around (180 degrees)
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
          }
          else if (orientAction == 2) {
            /* TODO - MOTOR CONTROL */
            // turn the car right
          }
          else if (orientAction == 3) {
            /* TODO - MOTOR CONTROL */
            // Turn the car around (180 degrees)
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
   
  }   // end else
  
}  // end loop

/**
 * Pathfinding support functions
 */
// Get reorientation action for the car, based on current action
// (0 == NO ACTION NEEDED, 1 = TURN LEFT, 2 = TURN RIGHT, 3 = TURN AROUND)
byte getReorientAct() {
  byte orientation = car.orientation;
  if (orientation == action)
    return 0; // Already in the correct orientation
  if (action == BYTE_INF)
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
  }
  else
    return 0; // Car's stored orientation is invalid
}

/** 
 *  Multiplexer Addressing function
 */
void tcaselect(uint8_t i){
  if(i > MAX_I2C_INPUTS) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

/**
 * Sensor test functions
 */
void testUltrasonic() {
  
  for( int i = 1; i < 20; i++ )
  {
     delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
     Serial.print("Ping: ");
     Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
     Serial.println("cm");
     delay(100);
  }
}

void testLasers() {

  VL53L0X_RangingMeasurementData_t measure;
  float luxleft;
  float luxright;
  int i = 0;

  tcaselect( FRONT );
  for( int i = 1; i < 20; i++ )
  {
     front.rangingTest(&measure, false);
     if(measure.RangeStatus != 4){
       Serial.print("Distance (mm): ");
       Serial.println(measure.RangeMilliMeter);
     }
     else{
       Serial.println("Out Of Range");
     }
     delay(50);
  }  // end for
  
  tcaselect( LEFT );
  for( int i = 1; i < 20; i++ )
  {
     luxleft = leftside.readLux(VL6180X_ALS_GAIN_5);
     Serial.print("Left Lux: ");
     Serial.println(luxleft);
     uint8_t range1 = leftside.readRange();
     uint8_t status1 = leftside.readRangeStatus();
     if(status1 == VL6180X_ERROR_NONE){
       Serial.print("Left Range: ");
       Serial.println(range1);
     }
     delay(50);
  }  // end for 
  
  tcaselect( RIGHT );
  for( int i = 1; i < 20; i++ )
  {
     luxright = rightside.readLux(VL6180X_ALS_GAIN_5);
     Serial.print("Right Lux: ");
     Serial.println(luxright);
     uint8_t range2 = rightside.readRange();
     uint8_t status2 = rightside.readRangeStatus();
     if(status2 == VL6180X_ERROR_NONE){
       Serial.print("Right Range: ");
       Serial.println(range2);
     }
     delay(50);
  }  // end for
   
  delay(50);
}  // end testLasers

void testMotors(){

   Serial.println("M1 Speed 100% Forward");
  md.setM1Speed(400);
  Serial.println("M2 Speed 100% Forward");
  md.setM2Speed(400);
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  delay(2000);

  Serial.println("M1 Speed 100% Backward");
  md.setM1Speed(-400);
  Serial.println("M2 Speed 100% Backward");
  md.setM2Speed(-400);
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  delay(2000);
  
  Serial.println("M1 Speed 50% Forward");
  md.setM1Speed(200);
  Serial.println("M2 Speed 50% Forward");
  md.setM2Speed(200);
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  delay(2000);

  Serial.println("M1 Speed 50% Backward");
  md.setM1Speed(-200);
  Serial.println("M2 Speed 50% Backward");
  md.setM2Speed(-200);
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  delay(2000);
  
  Serial.println("M1 Speed 0%");
  md.setM1Speed(0);
  Serial.println("M2 Speed 0%");
  md.setM2Speed(0);
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  delay(2000);

}  // end testMotors


void testIMU() {
  
   byte c = theIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
   Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
   Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    theIMU.MPU9250SelfTest(theIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(theIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(theIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(theIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(theIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(theIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(theIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    theIMU.calibrateMPU9250(theIMU.gyroBias, theIMU.accelBias);

    theIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = theIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    theIMU.initAK8963(theIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
   
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(theIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(theIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(theIMU.magCalibration[2], 2);

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
  }
}  // end test IMU

