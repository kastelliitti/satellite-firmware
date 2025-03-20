#include "CanSatNeXT.h"
#include <Wire.h>
#include "ADS1X15.h"

ADS1115 ADS(0x48);

// GET FROM THERMISTOR DATASHEET
int THERMISTOR_B = 3950;
int THERMISTOR_T0 = 298.15;
int THERMISTOR_R0 = 10000;

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
  float thermistorV = analogReadVoltage(NTC);
  float rTherm = THERMISTOR_R0*(thermistorV/(3.3-thermistorV));
  float temperatureK = THERMISTOR_B/(log(rTherm/THERMISTOR_R0)+(THERMISTOR_B/THERMISTOR_T0));
  float temperature = temperatureK - 273.15;

  float pressure = readPressure();
  int16_t adcValue = ADS.readADC(0);
  float ldrVoltage = analogRead(LDR);
  float voltageMeter = -100.0;
  if (adcValue != -101) {
    voltageMeter = ADS.toVoltage(adcValue);
  }
  float acceleration[3];
  readAcceleration(acceleration[0], acceleration[1], acceleration[2]);

  float gyro[3];
  readGyro(gyro[0], gyro[1], gyro[2]);

  float batteryVoltage = analogReadVoltage(BATT);

  uint8_t package[44];
  memcpy(&package[0], &temperature, 4);    // 0-3
  memcpy(&package[4], &pressure, 4);       // 4-7
  memcpy(&package[8], &ldrVoltage, 4);     // 8-11
  memcpy(&package[12], &voltageMeter, 4);  // 12-15
  for (int i = 0; i < 3; i++) {
    memcpy(&package[16 + (4 * i)], &acceleration[i], 4);  // 16-19, 20-23, 24-27
  }
  for (int i = 0; i < 3; i++) {
    memcpy(&package[28 + (4 * i)], &gyro[i], 4);  // 28-31, 32-35, 36-39
  }
  memcpy(&package[40], &batteryVoltage, 4);  // 40-43
  sendData(&package, 44);
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
