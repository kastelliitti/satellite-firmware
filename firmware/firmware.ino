#include "CanSatNeXT.h"
#include <Wire.h>
#include "ADS1X15.h"

ADS1115 ADS(0x48);

int MODE = 0;
int INTERVAL = 200;

void setup() {
  Serial.begin(115200);
  CanSatInit(42);
  if (!ADS.begin()) {
    Serial.println("ADS1115 not found on I2C bus!");
  } else {
    Serial.println("ADS1115 initialized successfully.");
  }
  ADS.setGain(0);
  Serial.println("READY");
}

void loop() {
  if (MODE == 0) {
    prelaunchMode();
  } else if (MODE == 1) {
    missionMode();
  } else if (MODE == 2) {
    postmissionMode();
  } else {
    standbyMode();
  }
}

void prelaunchMode() {
  Serial.println("Waiting for launch...");
  uint8_t package = 42;
  sendData(&package, 1);
  delay(200);
}

void standbyMode() {
  uint8_t package = 28;
  sendData(&package, 1);
  delay(200);
}

void missionMode() {
  float temperature = readTemperature();
  float pressure = readPressure();
  int16_t adcValue = ADS.readADC(0);
  float ldrVoltage = analogRead(LDR);
  float voltageMeter = -100.0;
  if (adcValue != -101) {
    voltageMeter = ADS.toVoltage(adcValue);
  }
  float acceleration[3];
  readAcceleration(acceleration[0], acceleration[1], acceleration[2]);

  uint8_t package[28];
  memcpy(&package[0], &temperature, 4);
  memcpy(&package[4], &pressure, 4);
  memcpy(&package[8], &ldrVoltage, 4);
  memcpy(&package[12], &voltageMeter, 4);
  for (int i = 0; i < 3; i++) {
    memcpy(&package[16 + (4 * i)], &acceleration[i], 4);
  }
  sendData(&package, 28);
  delay(INTERVAL);
}

void postmissionMode() {
  standbyMode();
}

void onBinaryDataReceived(const uint8_t *data, uint16_t len){
  if (len == 1) {
    int newMode = 0;
    memcpy(&newMode, data, len);
    if (newMode >= 0 && newMode < 3) {
      MODE = newMode;
    }
  } else if (len == 2) {
    unsigned short int newInterval;
    memcpy(&newInterval, data, len);
    if (newInterval >= 50 && newInterval <= 10000) {
      INTERVAL = newInterval;
    }
  }
}
