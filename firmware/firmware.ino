#include "CanSatNeXT.h"

void setup() {
  Serial.begin(115200);
  CanSatInit(42);
  Serial.println("TRANSMISSION START");

}

void loop() {
  float temperature = readTemperature();
  sendData((String)temperature);
  delay(500);
}
