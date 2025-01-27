#include "CanSatNeXT.h"

void setup() {
  Serial.begin(115200);
  CanSatInit(42);
  Serial.println("TRANSMISSION START");

}

void loop() {
  float temperature = readTemperature();
  float pressure = readPressure();
  float ldrVoltage = analogRead(LDR);
  float ax, ay, az;
  readAcceleration(ax, ay, az);
  sendData((String)temperature + "," + (String)pressure + "," + (String)ldrVoltage + "," + (String)ax + "," + (String)ay + "," + (String)az);
  delay(500);
}
