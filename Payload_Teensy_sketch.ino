#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Sensors
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DSOX.h>

// RF
#include <LoRa.h>

// SD
#include "SdFat.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// LoRa pins/frequency (adjust to your wiring)
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915E6
const uint8_t LORA_LOCAL_ADDRESS = 0xDD;  // payload
const uint8_t LORA_GROUND_ADDRESS = 0xCC; // home station
const uint8_t LORA_BROADCAST = 0xFF;

// Power switch control pin (Teensy GPIO -> TPS1H200A IN)
// Update this to the exact pin used on your PCB.
#define SENSOR_PWR_EN_PIN 6
#define VALVE_CTRL_PIN 27
#define FLOW_SENSOR_PIN 7
#define VALVE_ACTIVE_HIGH true
const bool USE_SENSOR_POWER_SWITCH = false; // debug: bypass rail toggle
const bool ENABLE_FLOW_SENSOR = true;   // set true when flow sensor is wired
const bool ENABLE_VALVE_CONTROL = true; // set true when valve driver is wired

// IMU I2C addresses
#define LSM6DSO32_ADDR_1 0x6A
#define LSM6DSO32_ADDR_2 0x6B

// Logging timing
const unsigned long LOG_INTERVAL_MS = 20; // ~50 Hz
const unsigned long FLUSH_INTERVAL_MS = 1000;
const unsigned long FLOW_UPDATE_MS = 1000;
const unsigned long LOG_FILE_DURATION_MS = 20000;  // rotate file every 20 s
const unsigned long MAX_LOG_DURATION_MS = 1800000; // auto-stop after 30 min
const bool AUTO_START_ON_BOOT = false;             // set true for bench tests
const bool TEST_ADXL_ONLY = false; // true = init/log ADXL only, skip others

// Valve
const bool ENABLE_AUTO_VALVE_FLIGHT_LOGIC = true;
const float GRAVITY_MS2 = 9.80665f;
const bool FORCE_LAUNCH_DETECTED_FOR_TEST = false;
const bool OPEN_VALVE_ON_ACCEL_FOR_TEST = false;
const float TEST_OPEN_ACCEL_THRESHOLD_MS2 = 14.0f;
const unsigned long TEST_OPEN_CONFIRM_MS = 60;
const float LAUNCH_ACCEL_THRESHOLD_MS2 = 50.0f; // ~1.3 g
const unsigned long LAUNCH_CONFIRM_MS = 120;
const float MICROGRAVITY_ENTER_MAX_MS2 = 8.8f; // enter microgravity
const float MICROGRAVITY_EXIT_MIN_MS2 = 3.0f;   // exit microgravity
const unsigned long MICROGRAVITY_CONFIRM_MS = 200;
const float ACCEL_MAG_FILTER_ALPHA = 0.15f;
const unsigned long FLIGHT_LOGIC_ARM_DELAY_MS = 300;
const bool STOP_LOGGING_ON_MICROGRAVITY_EXIT = true;

// Flow meter spec: F(Hz) = 98 * Q(L/min) => pulses/liter = 98*60 = 5880.
const float FLOW_PULSES_PER_LITER = 5880.0f;
const bool ENABLE_SENSOR_TARE = true;
const unsigned int SENSOR_TARE_SAMPLES = 32;
const unsigned long SENSOR_TARE_DELAY_MS = 2;

SdFs sd;
FsFile logFile;
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Sensor objects
Adafruit_ADXL375 accel(12345, &Wire); // I2C mode
Adafruit_BMP3XX bmp;
Adafruit_MPR121 cap;
LSM6DSOXClass imu1(Wire, LSM6DSO32_ADDR_1);
LSM6DSOXClass imu2(Wire, LSM6DSO32_ADDR_2);

// Runtime state
bool systemPowered = false;
bool loggingActive = false;
bool loraOk = false;
unsigned long startTime = 0;
unsigned long currentFileStartMs = 0;
unsigned long lastLogMs = 0;
unsigned long lastFlushMs = 0;

// Sensor health flags
bool adxlOk = false;
bool imu1Ok = false;
bool imu2Ok = false;
bool capOk = false;
bool bmpOk = false;
bool flowOk = false;
bool valveOpen = false;
bool launchDetected = false;
bool microgravityNow = false;
byte loraMsgCount = 0;

volatile uint32_t flowPulseCount = 0;
float flowHz = 0.0f;
float flowLpm = 0.0f;
unsigned long lastFlowUpdateMs = 0;
unsigned long launchConditionStartMs = 0;
unsigned long microConditionStartMs = 0;
unsigned long testOpenConditionStartMs = 0;
unsigned long flightLogicReadyMs = 0;
bool sensorTareComplete = false;
float filteredAccelMagMs2 = NAN;
bool stopLoggingRequested = false;
float adxlXTareMs2 = 0.0f;
float adxlYTareMs2 = 0.0f;
float adxlZTareMs2 = 0.0f;
float imu1AxTareG = 0.0f;
float imu1AyTareG = 0.0f;
float imu1AzTareG = 0.0f;
float imu1GxTareDps = 0.0f;
float imu1GyTareDps = 0.0f;
float imu1GzTareDps = 0.0f;
float imu2AxTareG = 0.0f;
float imu2AyTareG = 0.0f;
float imu2AzTareG = 0.0f;
float imu2GxTareDps = 0.0f;
float imu2GyTareDps = 0.0f;
float imu2GzTareDps = 0.0f;
float bmpTempTareC = 0.0f;
float bmpPressTareHpa = 0.0f;
float bmpAltTareM = 0.0f;

void flowPulseISR() { flowPulseCount++; }

bool createNextLogFile() {
  char filename[16];

  for (int i = 0; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "log%03d.csv", i);
    if (!sd.exists(filename)) {
      logFile = sd.open(filename, O_WRITE | O_CREAT | O_TRUNC);
      if (!logFile) {
        return false;
      }

      logFile.println("ms,adxl_x,adxl_y,adxl_z,imu1_ax,imu1_ay,imu1_az,imu1_gx,"
                      "imu1_gy,imu1_gz,imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,"
                      "imu2_gz,temp_c,press_hpa,alt_m,accel_mag_ms2,flow_hz,"
                      "flow_lpm,valve,sensor_status");
      logFile.flush();
      Serial.print("Opened ");
      Serial.println(filename);
      return true;
    }
  }

  return false;
}

void powerSensorsOn() {
  if (!USE_SENSOR_POWER_SWITCH) {
    systemPowered = true;
    Serial.println("Sensor rail toggle bypassed (debug)");
    return;
  }

  if (systemPowered) {
    return;
  }

  digitalWrite(SENSOR_PWR_EN_PIN, HIGH);
  delay(50); // rail settle
  systemPowered = true;
  Serial.println("Sensor rail ON");
}

void powerSensorsOff() {
  if (!USE_SENSOR_POWER_SWITCH) {
    systemPowered = false;
    return;
  }

  if (!systemPowered) {
    return;
  }

  digitalWrite(SENSOR_PWR_EN_PIN, LOW);
  systemPowered = false;
  Serial.println("Sensor rail OFF");
}

void setValve(bool open) {
  if (!ENABLE_VALVE_CONTROL) {
    valveOpen = false;
    return;
  }

  valveOpen = open;

  if (open) {
    Serial.print("valve on");
  }
  // High current load path is external (MOSFET/smart switch). This only drives
  // the control input.
  if (VALVE_ACTIVE_HIGH) {
    if (open) {
      digitalWrite(VALVE_CTRL_PIN, HIGH);
    } else {
      digitalWrite(VALVE_CTRL_PIN, LOW);
    }
  } else {
    if (open) {
      digitalWrite(VALVE_CTRL_PIN, LOW);
    } else {
      digitalWrite(VALVE_CTRL_PIN, HIGH);
    }
  }
}

void updateFlowStats(unsigned long nowMs) {
  if (!ENABLE_FLOW_SENSOR) {
    flowHz = 0.0f;
    flowLpm = 0.0f;
    return;
  }

  if (nowMs - lastFlowUpdateMs < FLOW_UPDATE_MS) {
    return;
  }

  uint32_t pulses = 0;
  noInterrupts();
  pulses = flowPulseCount;
  flowPulseCount = 0;
  interrupts();

  unsigned long elapsedMs = nowMs - lastFlowUpdateMs;
  if (elapsedMs == 0) {
    return;
  }

  flowHz = (1000.0f * pulses) / (float)elapsedMs;
  flowLpm = (flowHz * 60.0f) / FLOW_PULSES_PER_LITER;
  lastFlowUpdateMs = nowMs;
}

bool sustainedFor(bool condition, unsigned long nowMs, unsigned long &sinceMs,
                  unsigned long durationMs) {
  if (!condition) {
    sinceMs = 0;
    return false;
  }
  if (sinceMs == 0) {
    sinceMs = nowMs;
  }
  return (nowMs - sinceMs) >= durationMs;
}

float vectorMagnitude3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

bool tryGetAccelMagnitudeMs2(float adxlX, float adxlY, float adxlZ,
                             float imu1Ax, float imu1Ay, float imu1Az,
                             float imu2Ax, float imu2Ay, float imu2Az,
                             float &accelMagMs2) {
  if (isfinite(adxlX) && isfinite(adxlY) && isfinite(adxlZ)) {
    accelMagMs2 = vectorMagnitude3(adxlX, adxlY, adxlZ);
    return true;
  }

  float sum = 0.0f;
  int count = 0;
  if (isfinite(imu1Ax) && isfinite(imu1Ay) && isfinite(imu1Az)) {
    sum += vectorMagnitude3(imu1Ax, imu1Ay, imu1Az) * GRAVITY_MS2;
    count++;
  }
  if (isfinite(imu2Ax) && isfinite(imu2Ay) && isfinite(imu2Az)) {
    sum += vectorMagnitude3(imu2Ax, imu2Ay, imu2Az) * GRAVITY_MS2;
    count++;
  }
  if (count == 0) {
    return false;
  }

  accelMagMs2 = sum / (float)count;
  return true;
}

void resetFlightDetectionState() {
  launchDetected = false;
  microgravityNow = false;
  launchConditionStartMs = 0;
  microConditionStartMs = 0;
  testOpenConditionStartMs = 0;
  flightLogicReadyMs = 0;
  filteredAccelMagMs2 = NAN;
  stopLoggingRequested = false;
}

void updateFlightValveLogic(unsigned long nowMs, float adxlX, float adxlY,
                            float adxlZ, float imu1Ax, float imu1Ay,
                            float imu1Az, float imu2Ax, float imu2Ay,
                            float imu2Az) {
  if (!ENABLE_AUTO_VALVE_FLIGHT_LOGIC) {
    return;
  }

  if (!sensorTareComplete || nowMs < flightLogicReadyMs) {
    microgravityNow = false;
    setValve(false);
    return;
  }

  float accelMagMs2 = NAN;
  if (!tryGetAccelMagnitudeMs2(adxlX, adxlY, adxlZ, imu1Ax, imu1Ay, imu1Az,
                               imu2Ax, imu2Ay, imu2Az, accelMagMs2)) {
    microgravityNow = false;
    setValve(false);
    return;
  }

  if (!isfinite(filteredAccelMagMs2)) {
    filteredAccelMagMs2 = accelMagMs2;
  } else {
    filteredAccelMagMs2 +=
        ACCEL_MAG_FILTER_ALPHA * (accelMagMs2 - filteredAccelMagMs2);
  }

  if (!launchDetected) {
    bool launchCondition = filteredAccelMagMs2 >= LAUNCH_ACCEL_THRESHOLD_MS2;
    if (sustainedFor(launchCondition, nowMs, launchConditionStartMs,
                     LAUNCH_CONFIRM_MS)) {
      launchDetected = true;
      Serial.println("Flight state: launch detected");
    }
  }

  if (FORCE_LAUNCH_DETECTED_FOR_TEST) {
    launchDetected = true;
  }

  if (OPEN_VALVE_ON_ACCEL_FOR_TEST) {
    bool openCondition = filteredAccelMagMs2 >= TEST_OPEN_ACCEL_THRESHOLD_MS2;
    bool valveShouldOpen = sustainedFor(
        openCondition, nowMs, testOpenConditionStartMs, TEST_OPEN_CONFIRM_MS);
    microgravityNow = false;
    Serial.print("ms: ");
    Serial.print(nowMs);
    Serial.print("valve state: ");
    Serial.print(valveShouldOpen);
    Serial.println(" ");
    setValve(valveShouldOpen);
    return;
  }

  bool prevMicrogravityNow = microgravityNow;
  if (launchDetected) {
    bool microCondition = false;
    if (!microgravityNow) {
      microCondition = filteredAccelMagMs2 <= MICROGRAVITY_ENTER_MAX_MS2;
    } else {
      microCondition = filteredAccelMagMs2 <= MICROGRAVITY_EXIT_MIN_MS2;
    }
    microgravityNow = sustainedFor(microCondition, nowMs, microConditionStartMs,
                                   MICROGRAVITY_CONFIRM_MS);
  } else {
    microgravityNow = false;
    microConditionStartMs = 0;
  }

  if (microgravityNow != prevMicrogravityNow) {
    Serial.print("Flight state: microgravity ");
    Serial.println(microgravityNow ? "entered" : "exited");
    if (STOP_LOGGING_ON_MICROGRAVITY_EXIT && prevMicrogravityNow &&
        !microgravityNow) {
      stopLoggingRequested = true;
    }
  }

  if (launchDetected) {
    if (microgravityNow) {
      setValve(true);
    } else {
      setValve(false);
    }
  } else {
    setValve(false);
  }
}

void scanI2CBus() {
  Serial.println("I2C scan start");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("  found 0x");
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
      found++;
    } else if (err == 4) {
      Serial.print("  unknown error at 0x");
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
    }
  }

  if (found == 0) {
    Serial.println("  no I2C devices found");
  }
  Serial.println("I2C scan done");
}

void initSensors() {
  // Reset flags each start attempt.
  adxlOk = false;
  imu1Ok = false;
  imu2Ok = false;
  capOk = false;
  bmpOk = false;

  if (accel.begin()) {
    adxlOk = true;
    Serial.println("ADXL375 detected");
  } else {
    Serial.println("ADXL375 not detected");
  }

  if (TEST_ADXL_ONLY) {
    Serial.println("TEST_ADXL_ONLY enabled: skipping IMU/MPR121/BMP/flow");
    imu1Ok = false;
    imu2Ok = false;
    capOk = false;
    bmpOk = false;
    flowOk = false;
    return;
  }

  flowOk = ENABLE_FLOW_SENSOR;

  if (imu1.begin()) {
    imu1Ok = true;
    Serial.println("LSM6DSO32 #1 detected at 0x6A");
  } else {
    Serial.println("LSM6DSO32 #1 not detected at 0x6A");
  }

  if (imu2.begin()) {
    imu2Ok = true;
    Serial.println("LSM6DSO32 #2 detected at 0x6B");
  } else {
    Serial.println("LSM6DSO32 #2 not detected at 0x6B");
  }

  if (cap.begin()) {
    capOk = true;
    Serial.println("MPR121 detected");
  } else {
    Serial.println("MPR121 not detected");
  }

  if (bmp.begin_I2C()) {
    bmpOk = true;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("BMP388 detected");
  } else {
    Serial.println("BMP388 not detected");
  }
}

void calibrateSensorTares() {
  adxlXTareMs2 = 0.0f;
  adxlYTareMs2 = 0.0f;
  adxlZTareMs2 = 0.0f;
  imu1AxTareG = 0.0f;
  imu1AyTareG = 0.0f;
  imu1AzTareG = 0.0f;
  imu1GxTareDps = 0.0f;
  imu1GyTareDps = 0.0f;
  imu1GzTareDps = 0.0f;
  imu2AxTareG = 0.0f;
  imu2AyTareG = 0.0f;
  imu2AzTareG = 0.0f;
  imu2GxTareDps = 0.0f;
  imu2GyTareDps = 0.0f;
  imu2GzTareDps = 0.0f;
  bmpTempTareC = 0.0f;
  bmpPressTareHpa = 0.0f;
  bmpAltTareM = 0.0f;

  if (!ENABLE_SENSOR_TARE) {
    sensorTareComplete = true;
    return;
  }

  float adxlXSum = 0.0f, adxlYSum = 0.0f, adxlZSum = 0.0f;
  unsigned int adxlCount = 0;
  float imu1AxSum = 0.0f, imu1AySum = 0.0f, imu1AzSum = 0.0f;
  float imu1GxSum = 0.0f, imu1GySum = 0.0f, imu1GzSum = 0.0f;
  unsigned int imu1AccCount = 0, imu1GyroCount = 0;
  float imu2AxSum = 0.0f, imu2AySum = 0.0f, imu2AzSum = 0.0f;
  float imu2GxSum = 0.0f, imu2GySum = 0.0f, imu2GzSum = 0.0f;
  unsigned int imu2AccCount = 0, imu2GyroCount = 0;
  float bmpTempSum = 0.0f, bmpPressSum = 0.0f, bmpAltSum = 0.0f;
  unsigned int bmpCount = 0;

  for (unsigned int i = 0; i < SENSOR_TARE_SAMPLES; i++) {
    if (adxlOk) {
      sensors_event_t event;
      accel.getEvent(&event);
      if (isfinite(event.acceleration.x) && isfinite(event.acceleration.y) &&
          isfinite(event.acceleration.z)) {
        adxlXSum += event.acceleration.x;
        adxlYSum += event.acceleration.y;
        adxlZSum += event.acceleration.z;
        adxlCount++;
      }
    }

    if (imu1Ok) {
      float ax = 0.0f, ay = 0.0f, az = 0.0f;
      if (imu1.accelerationAvailable()) {
        imu1.readAcceleration(ax, ay, az);
        if (isfinite(ax) && isfinite(ay) && isfinite(az)) {
          imu1AxSum += ax;
          imu1AySum += ay;
          imu1AzSum += az;
          imu1AccCount++;
        }
      }

      float gx = 0.0f, gy = 0.0f, gz = 0.0f;
      if (imu1.gyroscopeAvailable()) {
        imu1.readGyroscope(gx, gy, gz);
        if (isfinite(gx) && isfinite(gy) && isfinite(gz)) {
          imu1GxSum += gx;
          imu1GySum += gy;
          imu1GzSum += gz;
          imu1GyroCount++;
        }
      }
    }

    if (imu2Ok) {
      float ax = 0.0f, ay = 0.0f, az = 0.0f;
      if (imu2.accelerationAvailable()) {
        imu2.readAcceleration(ax, ay, az);
        if (isfinite(ax) && isfinite(ay) && isfinite(az)) {
          imu2AxSum += ax;
          imu2AySum += ay;
          imu2AzSum += az;
          imu2AccCount++;
        }
      }

      float gx = 0.0f, gy = 0.0f, gz = 0.0f;
      if (imu2.gyroscopeAvailable()) {
        imu2.readGyroscope(gx, gy, gz);
        if (isfinite(gx) && isfinite(gy) && isfinite(gz)) {
          imu2GxSum += gx;
          imu2GySum += gy;
          imu2GzSum += gz;
          imu2GyroCount++;
        }
      }
    }

    if (bmpOk && bmp.performReading()) {
      float tempC = bmp.temperature;
      float pressHpa = bmp.pressure / 100.0f;
      float altM = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      if (isfinite(tempC) && isfinite(pressHpa) && isfinite(altM)) {
        bmpTempSum += tempC;
        bmpPressSum += pressHpa;
        bmpAltSum += altM;
        bmpCount++;
      }
    }

    delay(SENSOR_TARE_DELAY_MS);
  }

  if (adxlCount > 0) {
    adxlXTareMs2 = adxlXSum / (float)adxlCount;
    adxlYTareMs2 = adxlYSum / (float)adxlCount;
    adxlZTareMs2 = adxlZSum / (float)adxlCount;
  }
  if (imu1AccCount > 0) {
    imu1AxTareG = imu1AxSum / (float)imu1AccCount;
    imu1AyTareG = imu1AySum / (float)imu1AccCount;
    imu1AzTareG = imu1AzSum / (float)imu1AccCount;
  }
  if (imu1GyroCount > 0) {
    imu1GxTareDps = imu1GxSum / (float)imu1GyroCount;
    imu1GyTareDps = imu1GySum / (float)imu1GyroCount;
    imu1GzTareDps = imu1GzSum / (float)imu1GyroCount;
  }
  if (imu2AccCount > 0) {
    imu2AxTareG = imu2AxSum / (float)imu2AccCount;
    imu2AyTareG = imu2AySum / (float)imu2AccCount;
    imu2AzTareG = imu2AzSum / (float)imu2AccCount;
  }
  if (imu2GyroCount > 0) {
    imu2GxTareDps = imu2GxSum / (float)imu2GyroCount;
    imu2GyTareDps = imu2GySum / (float)imu2GyroCount;
    imu2GzTareDps = imu2GzSum / (float)imu2GyroCount;
  }
  if (bmpCount > 0) {
    bmpTempTareC = bmpTempSum / (float)bmpCount;
    bmpPressTareHpa = bmpPressSum / (float)bmpCount;
    bmpAltTareM = bmpAltSum / (float)bmpCount;
  }

  Serial.print("ADXL X tare (m/s^2): ");
  Serial.println(adxlXTareMs2, 4);
  Serial.print("ADXL Y tare (m/s^2): ");
  Serial.println(adxlYTareMs2, 4);
  Serial.print("ADXL Z tare (m/s^2): ");
  Serial.println(adxlZTareMs2, 4);
  Serial.print("IMU1 accel tare (g): ");
  Serial.print(imu1AxTareG, 4);
  Serial.print(", ");
  Serial.print(imu1AyTareG, 4);
  Serial.print(", ");
  Serial.println(imu1AzTareG, 4);
  Serial.print("IMU1 gyro tare (dps): ");
  Serial.print(imu1GxTareDps, 4);
  Serial.print(", ");
  Serial.print(imu1GyTareDps, 4);
  Serial.print(", ");
  Serial.println(imu1GzTareDps, 4);
  Serial.print("IMU2 accel tare (g): ");
  Serial.print(imu2AxTareG, 4);
  Serial.print(", ");
  Serial.print(imu2AyTareG, 4);
  Serial.print(", ");
  Serial.println(imu2AzTareG, 4);
  Serial.print("IMU2 gyro tare (dps): ");
  Serial.print(imu2GxTareDps, 4);
  Serial.print(", ");
  Serial.print(imu2GyTareDps, 4);
  Serial.print(", ");
  Serial.println(imu2GzTareDps, 4);
  Serial.print("BMP tare (C,hPa,m): ");
  Serial.print(bmpTempTareC, 4);
  Serial.print(", ");
  Serial.print(bmpPressTareHpa, 4);
  Serial.print(", ");
  Serial.println(bmpAltTareM, 4);
  sensorTareComplete = true;
}

void stopLogging(const char *reason) {
  if (loggingActive && logFile) {
    logFile.flush();
    logFile.close();
    Serial.print("Logging stopped: ");
    Serial.println(reason);
  }

  loggingActive = false;
  sensorTareComplete = false;
  setValve(false);
  resetFlightDetectionState();
  powerSensorsOff();
}

void sendLoRaMessage(uint8_t destination, const String &outgoing) {
  if (!loraOk)
    return;

  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write(loraMsgCount++);
  LoRa.write((uint8_t)outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
}

void startLogging() {
  if (loggingActive) {
    return;
  }

  sensorTareComplete = false;
  powerSensorsOn();
  initSensors();
  calibrateSensorTares();

  if (!createNextLogFile()) {
    Serial.println("Could not create log file");
    powerSensorsOff();
    return;
  }

  startTime = millis();
  currentFileStartMs = startTime;
  lastLogMs = startTime;
  lastFlushMs = startTime;
  lastFlowUpdateMs = startTime;
  flowHz = 0.0f;
  flowLpm = 0.0f;
  noInterrupts();
  flowPulseCount = 0;
  interrupts();
  resetFlightDetectionState();
  flightLogicReadyMs = startTime + FLIGHT_LOGIC_ARM_DELAY_MS;
  loggingActive = true;
  Serial.println("Logging started");
}

bool rotateLogFile(unsigned long nowMs) {
  if (!loggingActive || !logFile) {
    return false;
  }

  logFile.flush();
  logFile.close();

  if (!createNextLogFile()) {
    Serial.println("Could not rotate log file");
    stopLogging("log rotate failed");
    return false;
  }

  currentFileStartMs = nowMs;
  Serial.println("Log file rotated");
  return true;
}

void executeCommand(String cmd, const char *source,
                    uint8_t sender = LORA_GROUND_ADDRESS) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0)
    return;

  Serial.print(source);
  Serial.print(" cmd: ");
  Serial.println(cmd);

  if (cmd == "START" || cmd == "BUTTON PRESSED") {
    startLogging();
  } else if (cmd == "STOP") {
    stopLogging("STOP command");
  } else if (cmd == "SCAN") {
    scanI2CBus();
  } else if (cmd == "OPEN" || cmd == "VALVE_ON") {
    setValve(true);
  } else if (cmd == "CLOSE" || cmd == "VALVE_OFF") {
    setValve(false);
  } else if (cmd == "PING" || cmd == "HB") {
    sendLoRaMessage(sender, "PONG");
  }
}

void handleLoRaCommands() {
  if (!loraOk) {
    return;
  }

  int packetSize = LoRa.parsePacket();
  if (!packetSize) {
    return;
  }

  uint8_t rx[256];
  size_t n = 0;
  while (LoRa.available() && n < sizeof(rx)) {
    rx[n++] = (uint8_t)LoRa.read();
  }
  if (n == 0)
    return;

  // Preferred mode: framed packet [dest, src, msgId, len, payload...]
  if (n >= 4 && rx[3] == (uint8_t)(n - 4)) {
    uint8_t recipient = rx[0];
    uint8_t sender = rx[1];
    if (recipient != LORA_LOCAL_ADDRESS && recipient != LORA_BROADCAST) {
      return;
    }

    String payload;
    for (size_t i = 4; i < n; i++)
      payload += (char)rx[i];
    executeCommand(payload, "LoRa", sender);
    return;
  }

  // Compatibility mode: plain-text packet.
  String plain;
  for (size_t i = 0; i < n; i++)
    plain += (char)rx[i];
  executeCommand(plain, "LoRa");
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  executeCommand(cmd, "Serial");
}

void collectAndLogRow(unsigned long nowMs) {
  float adxlX = 0.0f, adxlY = 0.0f, adxlZ = 0.0f;
  float imu1Ax = 0.0f, imu1Ay = 0.0f, imu1Az = 0.0f;
  float imu1Gx = 0.0f, imu1Gy = 0.0f, imu1Gz = 0.0f;
  float imu2Ax = 0.0f, imu2Ay = 0.0f, imu2Az = 0.0f;
  float imu2Gx = 0.0f, imu2Gy = 0.0f, imu2Gz = 0.0f;
  float tempC = 0.0f, pressHpa = 0.0f, altM = 0.0f, accelMagMs2 = 0.0f;
  uint32_t sensorStatus = 0;

  if (adxlOk) {
    sensors_event_t event;
    accel.getEvent(&event);
    adxlX = event.acceleration.x - adxlXTareMs2;
    adxlY = event.acceleration.y - adxlYTareMs2;
    adxlZ = (event.acceleration.z - adxlZTareMs2) + GRAVITY_MS2;
    sensorStatus |= (1u << 0);
  }

  if (imu1Ok) {
    if (imu1.accelerationAvailable()) {
      imu1.readAcceleration(imu1Ax, imu1Ay, imu1Az);
      imu1Ax -= imu1AxTareG;
      imu1Ay -= imu1AyTareG;
      imu1Az = (imu1Az - imu1AzTareG) + 1.0f;
    }
    if (imu1.gyroscopeAvailable()) {
      imu1.readGyroscope(imu1Gx, imu1Gy, imu1Gz);
      imu1Gx -= imu1GxTareDps;
      imu1Gy -= imu1GyTareDps;
      imu1Gz -= imu1GzTareDps;
    }
    sensorStatus |= (1u << 1);
  }

  if (imu2Ok) {
    if (imu2.accelerationAvailable()) {
      imu2.readAcceleration(imu2Ax, imu2Ay, imu2Az);
      imu2Ax -= imu2AxTareG;
      imu2Ay -= imu2AyTareG;
      imu2Az = (imu2Az - imu2AzTareG) + 1.0f;
    }
    if (imu2.gyroscopeAvailable()) {
      imu2.readGyroscope(imu2Gx, imu2Gy, imu2Gz);
      imu2Gx -= imu2GxTareDps;
      imu2Gy -= imu2GyTareDps;
      imu2Gz -= imu2GzTareDps;
    }
    sensorStatus |= (1u << 2);
  }

  if (bmpOk && bmp.performReading()) {
    tempC = bmp.temperature - bmpTempTareC;
    pressHpa = (bmp.pressure / 100.0f) - bmpPressTareHpa;
    altM = bmp.readAltitude(SEALEVELPRESSURE_HPA) - bmpAltTareM;
    sensorStatus |= (1u << 3);
  }

  if (capOk) {
    sensorStatus |= (1u << 4);
  }
  if (flowOk) {
    sensorStatus |= (1u << 5);
  }

  tryGetAccelMagnitudeMs2(adxlX, adxlY, adxlZ, imu1Ax, imu1Ay, imu1Az, imu2Ax,
                          imu2Ay, imu2Az, accelMagMs2);

  updateFlightValveLogic(nowMs, adxlX, adxlY, adxlZ, imu1Ax, imu1Ay, imu1Az,
                         imu2Ax, imu2Ay, imu2Az);

  if (valveOpen) {
    sensorStatus |= (1u << 6);
  }
  if (launchDetected) {
    sensorStatus |= (1u << 7);
  }
  if (microgravityNow) {
    sensorStatus |= (1u << 8);
  }

  logFile.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%."
                 "4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%lu\n",
                 nowMs, adxlX, adxlY, adxlZ, imu1Ax, imu1Ay, imu1Az, imu1Gx,
                 imu1Gy, imu1Gz, imu2Ax, imu2Ay, imu2Az, imu2Gx, imu2Gy, imu2Gz,
                 tempC, pressHpa, altM, accelMagMs2, flowHz, flowLpm,
                 valveOpen ? 1u : 0u, (unsigned long)sensorStatus);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  pinMode(SENSOR_PWR_EN_PIN, OUTPUT);
  digitalWrite(SENSOR_PWR_EN_PIN, LOW); // keep sensors/load rail OFF at boot
  if (ENABLE_VALVE_CONTROL) {
    pinMode(VALVE_CTRL_PIN, OUTPUT);
    setValve(false);
  }
  if (ENABLE_FLOW_SENSOR) {
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseISR,
                    FALLING);
    flowOk = true;
  } else {
    flowOk = false;
  }

  Wire.begin();

  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD init failed");
    while (1) {
    }
  }

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
  if (!LoRa.begin(RF95_FREQ)) {
    Serial.println("LoRa init failed, continuing without RF control");
    loraOk = false;
  } else {
    loraOk = true;
    Serial.println("LoRa ready; waiting for START/STOP commands");
  }

  if (AUTO_START_ON_BOOT) {
    startLogging();
  }
}

void loop() {
  unsigned long nowMs = millis();

  handleLoRaCommands();
  handleSerialCommands();

  if (!loggingActive) {
    return;
  }

  // START may have been processed above; refresh so time deltas are computed
  // against the same or newer timestamp than startTime/lastLogMs.
  nowMs = millis();

  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    updateFlowStats(nowMs);
    collectAndLogRow(nowMs);
  }

  if (stopLoggingRequested) {
    stopLoggingRequested = false;
    stopLogging("microgravity exited");
    return;
  }

  if (nowMs - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = nowMs;
    logFile.flush();
    Serial.println("SD flush");
  }

  if (nowMs - currentFileStartMs >= LOG_FILE_DURATION_MS) {
    if (!rotateLogFile(nowMs)) {
      return;
    }
    lastFlushMs = nowMs;
  }

  if (nowMs - startTime >= MAX_LOG_DURATION_MS) {
    stopLogging("max duration reached");
  }
}
