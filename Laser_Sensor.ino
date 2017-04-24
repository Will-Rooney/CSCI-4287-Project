#include <Wire.h>
extern "C" {
  #include "utility/twi.h"
}
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL6180X.h"

#define TCAADDR 0x70

Adafruit_VL53L0X front = Adafruit_VL53L0X();
Adafruit_VL6180X leftside = Adafruit_VL6180X();
Adafruit_VL6180X rightside = Adafruit_VL6180X();

void tcaselect(uint8_t i){
  if(i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  Serial.println("Ready");

  //Initializing front sensor (VL53L0X)
  //On 4
  tcaselect(4);
  if(!front.begin()){
    Serial.println("VL53L0X Not Detected");
    while(1);
  }
  //On 0
  tcaselect(0);
  if(!leftside.begin()){
    Serial.println("Left VL6180X Not Detected");
    while(1);
  }
  //On 2
  tcaselect(2);
  if(!rightside.begin()){
    Serial.println("Right VL6180X Not Detected");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  VL53L0X_RangingMeasurementData_t measure;
  float luxleft;
  float luxright;

  tcaselect(4);
  front.rangingTest(&measure, false);
  if(measure.RangeStatus != 4){
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  }
  else{
    Serial.println("Out Of Range");
  }

  tcaselect(0);
  luxleft = leftside.readLux(VL6180X_ALS_GAIN_5);
  Serial.print("Left Lux: ");
  Serial.println(luxleft);
  uint8_t range1 = leftside.readRange();
  uint8_t status1 = leftside.readRangeStatus();
  if(status1 == VL6180X_ERROR_NONE){
    Serial.print("Left Range: ");
    Serial.println(range1);
  }

  tcaselect(2);
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
}

