#include "Wire.h"
extern "C"{
  #include "utility/twi.h"
}

#define TCAADDR 0X70

void tcaselect(uint8_t i){
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  while (!Serial);
  delay(1000);

  Wire.begin();

  Serial.begin(115200);
  Serial.println("Scanner Ready");

  for (uint8_t t=0; t<8; t++){
    tcaselect(t);
    Serial.print("TCA Port #");
    Serial.println(t);

    for(uint8_t addr=0; addr<=127; addr++){
      if(addr == TCAADDR) continue;

      uint8_t data;
      if (!twi_writeTo(addr. &data, 0, 1, 1)){
        Serial.print("Found I2C 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("Done");
}

void loop() {
}
