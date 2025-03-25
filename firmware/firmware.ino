#include "CanSatNeXT.h"
#include <Wire.h>
#include "ADS1X15.h"

ADS1115 ADS(0x48);

// GET FROM THERMISTOR DATASHEET
const float THERMISTOR_B = 3977.0f;
const float THERMISTOR_T0 = 298.15f;
const float THERMISTOR_R0 = 10000.0f;

// SD CARD SETTINGS
bool SD_PRESENT = false;
String FILENAME = "/savefile.csv";

int MODE = 0;
int INTERVAL = 200;

void setup() {
  Serial.begin(115200);
  int saveFileId = 0;
  bool saveFileIdFound = false;
  while (!saveFileIdFound) {
    String fileNameCandidate = "/savefile" + saveFileId + ".csv";
    if (!fileExists(fileNameCandidate)) {
      FILENAME = fileNameCandidate;
      saveFileIdFound = true;
    }
    saveFileId++;
  }
  setRadioChannel(7);
  CanSatInit(42);
  if (!ADS.begin()) {

    Serial.println("ADS1115 not found on I2C bus!");
  } else {
    Serial.println("ADS1115 initialized successfully.");
  }
  SD_PRESENT = SDCardPresent();
  if (SD_PRESENT) {
    appendFile(FILENAME, "timestamp, signal_strength, temp(C), r_thermistor(ohm), pressure(mbar), ldr(V), voltage(V), ax(G), ay(G), az(G), gx(mrad/s), gy(mrad/s), gz(mrad/s), battery_voltage(V)\n");
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
  float rTherm = THERMISTOR_R0 * ( 1 / ( (3.3 / thermistorV) - 1) );
  float temperatureK = THERMISTOR_B / ( log(rTherm / THERMISTOR_R0) + (THERMISTOR_B / THERMISTOR_T0) );
  float temperature = temperatureK - 273.15;
  //float temperature = rTherm;

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

  appendFile(FILENAME, String(millis()) + "," + String(temperature, 16) + "," + String(rTherm, 16) + "," + String(pressure, 16) + ","
    + String(ldrVoltage, 16) + "," + String(voltageMeter, 16) + "," + String(acceleration[0], 16) + ","
    + String(acceleration[1], 16) + "," + String(acceleration[2], 16) + "," + String(gyro[0], 16) + ","
    + String(gyro[1], 16) + "," + String(gyro[2], 16) + "," + String(batteryVoltage, 16) + "\n");
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
