#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Sensors
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DSOX.h>

// SD
#include "SdFat.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM6DSO32_ADDR_1 0x6A
#define LSM6DSO32_ADDR_2 0x6B

const unsigned long LOG_INTERVAL_MS = 20;   // ~50 Hz
const unsigned long FLUSH_INTERVAL_MS = 1000;
const unsigned long LOG_DURATION_MS = 10000;

SdFs sd;
FsFile logFile;
#define SD_CONFIG SdioConfig(FIFO_SDIO)

Adafruit_ADXL375 accel(12345, &Wire);  // I2C mode
Adafruit_BMP3XX bmp;
Adafruit_MPR121 cap;
LSM6DSOXClass imu1(Wire, LSM6DSO32_ADDR_1);
LSM6DSOXClass imu2(Wire, LSM6DSO32_ADDR_2);

bool adxlOk = false;
bool imu1Ok = false;
bool imu2Ok = false;
bool capOk = false;
bool bmpOk = false;

unsigned long startTime = 0;
unsigned long lastLogMs = 0;
unsigned long lastFlushMs = 0;

void initSensors() {
  adxlOk = accel.begin();
  imu1Ok = imu1.begin();
  imu2Ok = imu2.begin();
  capOk = cap.begin();

  if (bmp.begin_I2C()) {
    bmpOk = true;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  } else {
    bmpOk = false;
  }
}

void collectAndLogRow(unsigned long nowMs) {
  float adxlX = NAN, adxlY = NAN, adxlZ = NAN;
  float imu1Ax = NAN, imu1Ay = NAN, imu1Az = NAN;
  float imu1Gx = NAN, imu1Gy = NAN, imu1Gz = NAN;
  float imu2Ax = NAN, imu2Ay = NAN, imu2Az = NAN;
  float imu2Gx = NAN, imu2Gy = NAN, imu2Gz = NAN;
  float tempC = NAN, pressHpa = NAN, altM = NAN;
  uint32_t sensorStatus = 0;

  if (adxlOk) {
    sensors_event_t event;
    accel.getEvent(&event);
    adxlX = event.acceleration.x;
    adxlY = event.acceleration.y;
    adxlZ = event.acceleration.z;
    sensorStatus |= (1u << 0);
  }

  if (imu1Ok) {
    if (imu1.accelerationAvailable()) {
      imu1.readAcceleration(imu1Ax, imu1Ay, imu1Az);
    }
    if (imu1.gyroscopeAvailable()) {
      imu1.readGyroscope(imu1Gx, imu1Gy, imu1Gz);
    }
    sensorStatus |= (1u << 1);
  }

  if (imu2Ok) {
    if (imu2.accelerationAvailable()) {
      imu2.readAcceleration(imu2Ax, imu2Ay, imu2Az);
    }
    if (imu2.gyroscopeAvailable()) {
      imu2.readGyroscope(imu2Gx, imu2Gy, imu2Gz);
    }
    sensorStatus |= (1u << 2);
  }

  if (bmpOk && bmp.performReading()) {
    tempC = bmp.temperature;
    pressHpa = bmp.pressure / 100.0;
    altM = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    sensorStatus |= (1u << 3);
  }

  if (capOk) {
    sensorStatus |= (1u << 4);
  }

  logFile.printf(
      "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu\n",
      nowMs, adxlX, adxlY, adxlZ, imu1Ax, imu1Ay, imu1Az, imu1Gx, imu1Gy,
      imu1Gz, imu2Ax, imu2Ay, imu2Az, imu2Gx, imu2Gy, imu2Gz, tempC,
      pressHpa, altM, (unsigned long)sensorStatus);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  Wire.begin();

  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD init failed");
    while (1) {
    }
  }

  logFile = sd.open("log000.csv", O_WRITE | O_CREAT | O_TRUNC);
  if (!logFile) {
    Serial.println("Could not open log file");
    while (1) {
    }
  }

  logFile.println("ms,adxl_x,adxl_y,adxl_z,imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,temp_c,press_hpa,alt_m,sensor_status");
  logFile.flush();

  initSensors();

  startTime = millis();
  lastLogMs = startTime;
  lastFlushMs = startTime;
}

void loop() {
  unsigned long nowMs = millis();

  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    collectAndLogRow(nowMs);
  }

  if (nowMs - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = nowMs;
    logFile.flush();
  }

  if (nowMs - startTime >= LOG_DURATION_MS) {
    logFile.flush();
    logFile.close();
    while (1) {
    }
  }
}
