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
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 915E6

// Power switch control pin (Teensy GPIO -> TPS1H200A IN)
// Update this to the exact pin used on your PCB.
#define SENSOR_PWR_EN_PIN 6
#define VALVE_CTRL_PIN 5
#define FLOW_SENSOR_PIN 4
#define VALVE_ACTIVE_HIGH true
const bool USE_SENSOR_POWER_SWITCH = false; // debug: bypass rail toggle
const bool ENABLE_FLOW_SENSOR = false;   // set true when flow sensor is wired
const bool ENABLE_VALVE_CONTROL = false; // set true when valve driver is wired

// IMU I2C addresses
#define LSM6DSO32_ADDR_1 0x6A
#define LSM6DSO32_ADDR_2 0x6B

// Logging timing
const unsigned long LOG_INTERVAL_MS = 20; // ~50 Hz
const unsigned long FLUSH_INTERVAL_MS = 1000;
const unsigned long FLOW_UPDATE_MS = 1000;
const unsigned long MAX_LOG_DURATION_MS = 20000; // auto-stop after START (20 s)
const bool AUTO_START_ON_BOOT = false;           // set true for bench tests
const bool TEST_ADXL_ONLY = false; // true = init/log ADXL only, skip others
// Calibrate this for your exact flow sensor model.
const float FLOW_PULSES_PER_LITER = 450.0f;

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

volatile uint32_t flowPulseCount = 0;
float flowHz = 0.0f;
float flowLpm = 0.0f;
unsigned long lastFlowUpdateMs = 0;

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
                      "imu2_gz,temp_c,press_hpa,alt_m,flow_hz,flow_lpm,valve,"
                      "sensor_status");
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
  bool pinState = VALVE_ACTIVE_HIGH ? open : !open;
  // High current load path is external (MOSFET/smart switch). This only drives
  // the control input.
  digitalWrite(VALVE_CTRL_PIN, pinState ? HIGH : LOW);
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

void stopLogging(const char *reason) {
  if (loggingActive && logFile) {
    logFile.flush();
    logFile.close();
    Serial.print("Logging stopped: ");
    Serial.println(reason);
  }

  loggingActive = false;
  setValve(false);
  powerSensorsOff();
}

void startLogging() {
  if (loggingActive) {
    return;
  }

  powerSensorsOn();
  initSensors();

  if (!createNextLogFile()) {
    Serial.println("Could not create log file");
    powerSensorsOff();
    return;
  }

  startTime = millis();
  lastLogMs = startTime;
  lastFlushMs = startTime;
  lastFlowUpdateMs = startTime;
  flowHz = 0.0f;
  flowLpm = 0.0f;
  noInterrupts();
  flowPulseCount = 0;
  interrupts();
  loggingActive = true;
  Serial.println("Logging started");
}

void handleLoRaCommands() {
  if (!loraOk) {
    return;
  }

  int packetSize = LoRa.parsePacket();
  if (!packetSize) {
    return;
  }

  String cmd;
  while (LoRa.available()) {
    cmd += (char)LoRa.read();
  }
  cmd.trim();
  cmd.toUpperCase();

  Serial.print("LoRa cmd: ");
  Serial.println(cmd);

  if (cmd == "START") {
    startLogging();
  } else if (cmd == "STOP") {
    stopLogging("RF STOP command");
  } else if (cmd == "OPEN" || cmd == "VALVE_ON") {
    setValve(true);
  } else if (cmd == "CLOSE" || cmd == "VALVE_OFF") {
    setValve(false);
  } else if (cmd == "PING") {
    LoRa.beginPacket();
    LoRa.print("PONG");
    LoRa.endPacket();
  }
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) {
    return;
  }

  Serial.print("Serial cmd: ");
  Serial.println(cmd);

  if (cmd == "START") {
    startLogging();
  } else if (cmd == "STOP") {
    stopLogging("Serial STOP command");
  } else if (cmd == "OPEN" || cmd == "VALVE_ON") {
    setValve(true);
  } else if (cmd == "CLOSE" || cmd == "VALVE_OFF") {
    setValve(false);
  } else if (cmd == "PING") {
    Serial.println("PONG");
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
  if (flowOk) {
    sensorStatus |= (1u << 5);
  }
  if (valveOpen) {
    sensorStatus |= (1u << 6);
  }

  logFile.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%."
                 "4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%lu\n",
                 nowMs, adxlX, adxlY, adxlZ, imu1Ax, imu1Ay, imu1Az, imu1Gx,
                 imu1Gy, imu1Gz, imu2Ax, imu2Ay, imu2Az, imu2Gx, imu2Gy, imu2Gz,
                 tempC, pressHpa, altM, flowHz, flowLpm, valveOpen ? 1u : 0u,
                 (unsigned long)sensorStatus);
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

  if (nowMs - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = nowMs;
    logFile.flush();
    Serial.println("SD flush");
  }

  if (nowMs - startTime >= MAX_LOG_DURATION_MS) {
    stopLogging("max duration reached");
  }
}
