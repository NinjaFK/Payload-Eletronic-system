#include <Arduino.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Wire.h>

// Sensors
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_Sensor.h>

// RF
#include <LoRa.h>

// SD
#include "SdFat.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// LoRa pins/frequency (adjust to your wiring)
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 433E6
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
const bool ENABLE_FLOW_SENSOR = false;  // set true when flow sensor is wired
const bool ENABLE_VALVE_CONTROL = true; // set true when valve driver is wired

// IMU I2C addresses
#define LSM6DSO32_ADDR_1 0x6A
#define LSM6DSO32_ADDR_2 0x6B

// Logging timing
const unsigned long LOG_INTERVAL_MS = 20; // ~50 Hz
const unsigned long FLUSH_INTERVAL_MS = 1000;
const unsigned long FLOW_UPDATE_MS = 100;
const unsigned long LIVE_TIME_REPORT_MS = 500;
const unsigned long RF_TIME_REPORT_MS = 500;
const unsigned long LOG_FILE_DURATION_MS = 20000;   // rotate file every 20 s
const unsigned long MAX_LOG_DURATION_MS = 14000000; // auto-stop after 4 hours
const bool AUTO_START_ON_BOOT = false;              // set true for bench tests
const bool TEST_ADXL_ONLY = false; // true = init/log ADXL only, skip others
const bool ENABLE_LIVE_TIME_REPORT = true;
const bool ENABLE_RF_TIME_REPORT = true;

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
const float MICROGRAVITY_EXIT_MIN_MS2 = 3.0f;  // exit microgravity
const unsigned long MICROGRAVITY_CONFIRM_MS = 200;
const float ACCEL_MAG_FILTER_ALPHA = 0.15f;
const unsigned long FLIGHT_LOGIC_ARM_DELAY_MS = 300;
const bool STOP_LOGGING_ON_MICROGRAVITY_EXIT = true;
const uint8_t MPR121_WATER_ELECTRODE = 0;
// Capacitive calibration from recent bench logs:
// dry/mostly-air run ~697 raw baseline, full-water run ~724.5 raw.
const float CAP_CAL_AIR_RAW = 697.0f;
const float CAP_CAL_WATER_RAW = 724.5f;
const float DIELECTRIC_AIR = 1.0006f;
const float DIELECTRIC_WATER = 80.0f;
const float CAP_RAW_FILTER_ALPHA = 0.12f; // raw count smoothing
const float CAP_WATER_PCT_ALPHA = 0.20f;  // EMA filter to reduce chatter
const bool ENABLE_CAP_STARTUP_TARE = true;
const bool CAP_TARE_ALIGN_TO_AIR = true;
const float CAP_TARE_OFFSET_APPLY_THRESHOLD_RAW = 20.0f;
const float CAP_TARE_MAX_ABS_OFFSET_RAW = 300.0f;
const bool ENABLE_CAP_ESTIMATED_FLOW = true;
// Fluid volume represented by the capacitive sensing zone (liters).
const float CAP_SENSOR_VOLUME_L = 0.010f;
// Use absolute fill change so mixed slug/bubble flow still reports activity.
const bool CAP_ESTIMATED_FLOW_USE_ABS_DELTA = true;
const float CAP_FLOW_FRAC_DEADBAND = 0.01f; // ignore <1% sample-to-sample fill
const float CAP_ESTIMATED_FLOW_MAX_LPM = 2.0f;

// Flow meter spec: F(Hz) = 98 * Q(L/min) => pulses/liter = 98*60 = 5880.
const float FLOW_PULSES_PER_LITER = 5880.0f;
const bool ENABLE_SENSOR_TARE = true;
const unsigned int SENSOR_TARE_SAMPLES = 32;
const unsigned long SENSOR_TARE_DELAY_MS = 2;

SdFs sd;
FsFile logFile;
#if defined(FIFO_SDIO)
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif defined(BUILTIN_SDCARD)
#define SD_CONFIG SdioConfig()
#else
#define SD_CONFIG SdSpiConfig(SS, SHARED_SPI)
#endif

#if defined(CORE_TEENSY)
#define HAS_TEENSY3_RTC 1
#else
#define HAS_TEENSY3_RTC 0
#endif

// Sensor objects
Adafruit_ADXL375 accel(12345, &Wire); // I2C mode
Adafruit_BMP3XX bmp;
Adafruit_MPR121 cap;
Adafruit_LSM6DSO32 imu1;
Adafruit_LSM6DSO32 imu2;

// Runtime state
bool systemPowered = false;
bool loggingActive = false;
bool loraOk = false;
unsigned long startTime = 0;
unsigned long currentFileStartMs = 0;
unsigned long lastLogMs = 0;
unsigned long lastFlushMs = 0;
unsigned long lastLiveTimeReportMs = 0;
unsigned long lastRfTimeReportMs = 0;

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
float imu1AxTareMs2 = 0.0f;
float imu1AyTareMs2 = 0.0f;
float imu1AzTareMs2 = 0.0f;
float imu1GxTareRadS = 0.0f;
float imu1GyTareRadS = 0.0f;
float imu1GzTareRadS = 0.0f;
float imu2AxTareMs2 = 0.0f;
float imu2AyTareMs2 = 0.0f;
float imu2AzTareMs2 = 0.0f;
float imu2GxTareRadS = 0.0f;
float imu2GyTareRadS = 0.0f;
float imu2GzTareRadS = 0.0f;
float bmpTempTareC = 0.0f;
float bmpPressTareHpa = 0.0f;
float bmpAltTareM = 0.0f;
float filteredWaterPct = NAN;
float filteredCapRaw = NAN;
float lastWaterFracForFlow = NAN;
unsigned long lastCapEstimateMs = 0;
float capStartupBaselineRaw = NAN;
float capRawAlignOffset = 0.0f;
bool capStartupTareValid = false;

/**
 * @brief TimeLib sync provider callback using Teensy RTC.
 *
 * @return Current RTC epoch time.
 */
time_t getRtcTime() {
#if HAS_TEENSY3_RTC
  return Teensy3Clock.get();
#else
  return 0;
#endif
}

/**
 * @brief Flow sensor interrupt service routine.
 *
 * Increments the shared pulse counter on each detected flow pulse.
 */
void flowPulseISR() { flowPulseCount++; }

/**
 * @brief Create and open the next available rotating CSV log file.
 *
 * Scans for the first free filename in the form `logNNN.csv`, opens it, and
 * writes the CSV header row.
 *
 * @return true if a new log file was created and opened successfully.
 * @return false if no file could be created/opened.
 */
bool createNextLogFile() {
  char filename[16];

  for (int i = 0; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "log%03d.csv", i);
    if (!sd.exists(filename)) {
      logFile = sd.open(filename, O_WRITE | O_CREAT | O_TRUNC);
      if (!logFile) {
        return false;
      }

      logFile.println(
          "datetime_local,ms,adxl_x,adxl_y,adxl_z,"
          "imu1_ax_ms2,imu1_ay_ms2,imu1_az_ms2,imu1_gx_rads,imu1_gy_rads,"
          "imu1_gz_rads,imu1_accel_norm_g,"
          "imu2_ax_ms2,imu2_ay_ms2,imu2_az_ms2,imu2_gx_rads,imu2_gy_rads,"
          "imu2_gz_rads,imu2_accel_norm_g,"
          "temp_c,press_hpa,alt_m,accel_mag_ms2,flow_hz,flow_lpm,"
          "cap_raw,water_pct,valve,sensor_status");
      logFile.flush();
      Serial.print("Opened ");
      Serial.println(filename);
      return true;
    }
  }

  return false;
}

/**
 * @brief Enable sensor power rail.
 *
 * Turns on the switched sensor rail when enabled, or marks the system powered
 * in debug bypass mode.
 */
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

/**
 * @brief Disable sensor power rail.
 *
 * Turns off the switched sensor rail when enabled. In debug bypass mode this
 * only updates software state.
 */
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

/**
 * @brief Command valve open/closed state.
 *
 * Applies the requested valve state to the control pin using configured active
 * polarity and updates valve state bookkeeping.
 *
 * @param open `true` to command valve open, `false` to command valve closed.
 */
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

/**
 * @brief Update computed flow metrics from ISR pulse counts.
 *
 * Every `FLOW_UPDATE_MS`, atomically snapshots pulse count, computes flow
 * frequency in Hz and flow rate in liters/minute, then resets the counter.
 *
 * @param nowMs Current `millis()` timestamp.
 */
void updateFlowStats(unsigned long nowMs) {
  // Behavior: flow sensor disabled -> publish zero flow values and exit.
  if (!ENABLE_FLOW_SENSOR) {
    flowHz = 0.0f;
    flowLpm = 0.0f;
    return;
  }

  // Behavior: wait until next configured flow integration window.
  if (nowMs - lastFlowUpdateMs < FLOW_UPDATE_MS) {
    return;
  }

  // Behavior: atomically snapshot and clear ISR-updated pulse count.
  uint32_t pulses = 0;
  noInterrupts();
  pulses = flowPulseCount;
  flowPulseCount = 0;
  interrupts();

  unsigned long elapsedMs = nowMs - lastFlowUpdateMs;
  // Behavior: guard divide-by-zero if timestamps happen to match.
  if (elapsedMs == 0) {
    return;
  }

  // Behavior: convert pulse frequency to flow engineering units.
  flowHz = (1000.0f * pulses) / (float)elapsedMs;
  flowLpm = (flowHz * 60.0f) / FLOW_PULSES_PER_LITER;
  lastFlowUpdateMs = nowMs;
}

/**
 * @brief Check whether a condition has remained true for a minimum duration.
 *
 * Uses `sinceMs` as state: resets it when condition is false, sets it on first
 * true sample, and returns true once the elapsed time reaches `durationMs`.
 *
 * @param condition Condition to evaluate.
 * @param nowMs Current `millis()` timestamp.
 * @param sinceMs Timestamp storage for when the condition first became true.
 * @param durationMs Required continuous true duration.
 * @return true when the condition has been continuously true long enough.
 * @return false otherwise.
 */
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

/**
 * @brief Compute 3D vector magnitude.
 *
 * @param x X-axis component.
 * @param y Y-axis component.
 * @param z Z-axis component.
 * @return Euclidean norm `sqrt(x^2 + y^2 + z^2)`.
 */
float vectorMagnitude3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

/**
 * @brief Clamp a scalar to [0, 1].
 */
float clamp01(float v) {
  if (v < 0.0f) {
    return 0.0f;
  }
  if (v > 1.0f) {
    return 1.0f;
  }
  return v;
}

/**
 * @brief Convert MPR121 filtered count into estimated water fraction.
 *
 * Uses a 2-point calibration between known air and known full-water states.
 * Because capacitance scales with dielectric constant, this ratio is a proxy
 * for water fill in the sensing field.
 *
 * @param capRaw MPR121 filteredData raw count.
 * @return Estimated water fraction [0..1], or NAN if invalid calibration.
 */
float estimateWaterFractionFromCapRaw(float capRaw) {
  if (!isfinite(capRaw)) {
    return NAN;
  }
  float span = CAP_CAL_WATER_RAW - CAP_CAL_AIR_RAW;
  if (fabsf(span) < 1e-6f) {
    return NAN;
  }
  float ratio = (capRaw - CAP_CAL_AIR_RAW) / span;
  return clamp01(ratio);
}

/**
 * @brief Compute average acceleration magnitude across valid IMUs.
 *
 * Uses whichever IMU acceleration triplets are finite, computes each magnitude,
 * and returns their average.
 *
 * @param imu1AxMs2 IMU1 acceleration X (m/s^2).
 * @param imu1AyMs2 IMU1 acceleration Y (m/s^2).
 * @param imu1AzMs2 IMU1 acceleration Z (m/s^2).
 * @param imu2AxMs2 IMU2 acceleration X (m/s^2).
 * @param imu2AyMs2 IMU2 acceleration Y (m/s^2).
 * @param imu2AzMs2 IMU2 acceleration Z (m/s^2).
 * @param[out] accelMagMs2 Output average magnitude (m/s^2) when available.
 * @return true if at least one IMU provided a valid magnitude.
 * @return false if no valid IMU acceleration sample was available.
 */
bool tryGetAccelMagnitudeMs2(float imu1AxMs2, float imu1AyMs2, float imu1AzMs2,
                             float imu2AxMs2, float imu2AyMs2, float imu2AzMs2,
                             float &accelMagMs2) {
  float sum = 0.0f;
  int count = 0;
  if (isfinite(imu1AxMs2) && isfinite(imu1AyMs2) && isfinite(imu1AzMs2)) {
    sum += vectorMagnitude3(imu1AxMs2, imu1AyMs2, imu1AzMs2);
    count++;
  }
  if (isfinite(imu2AxMs2) && isfinite(imu2AyMs2) && isfinite(imu2AzMs2)) {
    sum += vectorMagnitude3(imu2AxMs2, imu2AyMs2, imu2AzMs2);
    count++;
  }
  if (count == 0) {
    return false;
  }

  accelMagMs2 = sum / (float)count;
  return true;
}

/**
 * @brief Reset launch/microgravity detection state and timers.
 */
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

/**
 * @brief Evaluate flight state machine and update valve command.
 *
 * Computes filtered acceleration magnitude, detects launch and microgravity
 * entry/exit using sustained threshold checks, and commands valve state.
 *
 * @param nowMs Current `millis()` timestamp.
 * @param imu1AxMs2 IMU1 acceleration X (m/s^2).
 * @param imu1AyMs2 IMU1 acceleration Y (m/s^2).
 * @param imu1AzMs2 IMU1 acceleration Z (m/s^2).
 * @param imu2AxMs2 IMU2 acceleration X (m/s^2).
 * @param imu2AyMs2 IMU2 acceleration Y (m/s^2).
 * @param imu2AzMs2 IMU2 acceleration Z (m/s^2).
 */
void updateFlightValveLogic(unsigned long nowMs, float imu1AxMs2,
                            float imu1AyMs2, float imu1AzMs2, float imu2AxMs2,
                            float imu2AyMs2, float imu2AzMs2) {
  // Behavior: master flight logic disable -> do not alter valve automatically.
  if (!ENABLE_AUTO_VALVE_FLIGHT_LOGIC) {
    return;
  }

  // Behavior: hold safe state until tare is complete and arm delay has elapsed.
  if (!sensorTareComplete || nowMs < flightLogicReadyMs) {
    microgravityNow = false;
    setValve(false);
    return;
  }

  float accelMagMs2 = NAN;
  // Behavior: no valid acceleration estimate -> fail safe with valve closed.
  if (!tryGetAccelMagnitudeMs2(imu1AxMs2, imu1AyMs2, imu1AzMs2, imu2AxMs2,
                               imu2AyMs2, imu2AzMs2, accelMagMs2)) {
    microgravityNow = false;
    setValve(false);
    return;
  }

  // EMA filter reduces single-sample spikes before threshold checks.
  if (!isfinite(filteredAccelMagMs2)) {
    filteredAccelMagMs2 = accelMagMs2;
  } else {
    filteredAccelMagMs2 +=
        ACCEL_MAG_FILTER_ALPHA * (accelMagMs2 - filteredAccelMagMs2);
  }

  // Behavior: once launch is confirmed, latch `launchDetected`.
  if (!launchDetected) {
    bool launchCondition = filteredAccelMagMs2 >= LAUNCH_ACCEL_THRESHOLD_MS2;
    if (sustainedFor(launchCondition, nowMs, launchConditionStartMs,
                     LAUNCH_CONFIRM_MS)) {
      launchDetected = true;
      Serial.println("Flight state: launch detected");
    }
  }

  // Behavior: test override to force launch state without threshold check.
  if (FORCE_LAUNCH_DETECTED_FOR_TEST) {
    launchDetected = true;
  }

  // Behavior: bench-test mode opens valve from accel threshold only.
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

  // Hysteresis: lower threshold to enter microgravity, higher to exit.
  bool prevMicrogravityNow = microgravityNow;
  // Behavior: microgravity detection only runs after launch latch.
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
    // Behavior: valve follows current microgravity state during flight.
    if (microgravityNow) {
      setValve(true);
    } else {
      setValve(false);
    }
  } else {
    // Behavior: never open valve pre-launch in normal flight logic.
    setValve(false);
  }
}

/**
 * @brief Scan and print all responding I2C addresses.
 *
 * Useful for wiring and address diagnostics during bring-up.
 */
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

/**
 * @brief Initialize onboard sensors and update health flags.
 *
 * Configures available sensors and records per-device status for logging and
 * runtime logic.
 */
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

  if (imu1.begin_I2C(LSM6DSO32_ADDR_1, &Wire)) {
    imu1Ok = true;
    imu1.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
    imu1.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu1.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu1.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("LSM6DSO32 #1 detected at 0x6A");
  } else {
    Serial.println("LSM6DSO32 #1 not detected at 0x6A");
  }

  if (imu2.begin_I2C(LSM6DSO32_ADDR_2, &Wire)) {
    imu2Ok = true;
    imu2.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
    imu2.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu2.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu2.setGyroDataRate(LSM6DS_RATE_104_HZ);
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

  if (capOk) {
    Serial.println("Running auto configuration.");
    cap.setAutoconfig(true);
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

/**
 * @brief Measure and store tare offsets for enabled sensors.
 *
 * Collects multiple samples to estimate baseline offsets used to zero data
 * during logging.
 */
void calibrateSensorTares() {
  // Behavior: start from known zero offsets each calibration cycle.
  adxlXTareMs2 = 0.0f;
  adxlYTareMs2 = 0.0f;
  adxlZTareMs2 = 0.0f;
  imu1AxTareMs2 = 0.0f;
  imu1AyTareMs2 = 0.0f;
  imu1AzTareMs2 = 0.0f;
  imu1GxTareRadS = 0.0f;
  imu1GyTareRadS = 0.0f;
  imu1GzTareRadS = 0.0f;
  imu2AxTareMs2 = 0.0f;
  imu2AyTareMs2 = 0.0f;
  imu2AzTareMs2 = 0.0f;
  imu2GxTareRadS = 0.0f;
  imu2GyTareRadS = 0.0f;
  imu2GzTareRadS = 0.0f;
  bmpTempTareC = 0.0f;
  bmpPressTareHpa = 0.0f;
  bmpAltTareM = 0.0f;
  capStartupBaselineRaw = NAN;
  capRawAlignOffset = 0.0f;
  capStartupTareValid = false;

  // Behavior: optional bypass for quicker startup/testing.
  if (!ENABLE_SENSOR_TARE) {
    sensorTareComplete = true;
    return;
  }

  // Accumulators and valid-sample counters for each sensor stream.
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
  float capRawSum = 0.0f;
  unsigned int capCount = 0;

  for (unsigned int i = 0; i < SENSOR_TARE_SAMPLES; i++) {
    // Behavior: collect ADXL tare sample when available and finite.
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

    // Behavior: collect IMU1 accel/gyro tare samples independently.
    if (imu1Ok) {
      sensors_event_t imu1AccelEvent;
      sensors_event_t imu1GyroEvent;
      sensors_event_t imu1TempEvent;
      imu1.getEvent(&imu1AccelEvent, &imu1GyroEvent, &imu1TempEvent);

      if (isfinite(imu1AccelEvent.acceleration.x) &&
          isfinite(imu1AccelEvent.acceleration.y) &&
          isfinite(imu1AccelEvent.acceleration.z)) {
        imu1AxSum += imu1AccelEvent.acceleration.x;
        imu1AySum += imu1AccelEvent.acceleration.y;
        imu1AzSum += imu1AccelEvent.acceleration.z;
        imu1AccCount++;
      }
      if (isfinite(imu1GyroEvent.gyro.x) && isfinite(imu1GyroEvent.gyro.y) &&
          isfinite(imu1GyroEvent.gyro.z)) {
        imu1GxSum += imu1GyroEvent.gyro.x;
        imu1GySum += imu1GyroEvent.gyro.y;
        imu1GzSum += imu1GyroEvent.gyro.z;
        imu1GyroCount++;
      }
    }

    // Behavior: collect IMU2 accel/gyro tare samples independently.
    if (imu2Ok) {
      sensors_event_t imu2AccelEvent;
      sensors_event_t imu2GyroEvent;
      sensors_event_t imu2TempEvent;
      imu2.getEvent(&imu2AccelEvent, &imu2GyroEvent, &imu2TempEvent);

      if (isfinite(imu2AccelEvent.acceleration.x) &&
          isfinite(imu2AccelEvent.acceleration.y) &&
          isfinite(imu2AccelEvent.acceleration.z)) {
        imu2AxSum += imu2AccelEvent.acceleration.x;
        imu2AySum += imu2AccelEvent.acceleration.y;
        imu2AzSum += imu2AccelEvent.acceleration.z;
        imu2AccCount++;
      }
      if (isfinite(imu2GyroEvent.gyro.x) && isfinite(imu2GyroEvent.gyro.y) &&
          isfinite(imu2GyroEvent.gyro.z)) {
        imu2GxSum += imu2GyroEvent.gyro.x;
        imu2GySum += imu2GyroEvent.gyro.y;
        imu2GzSum += imu2GyroEvent.gyro.z;
        imu2GyroCount++;
      }
    }

    // Behavior: collect BMP temperature/pressure/altitude baseline.
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

    // Behavior: collect startup capacitance baseline for session alignment.
    if (capOk && ENABLE_CAP_STARTUP_TARE) {
      float capSample = (float)cap.filteredData(MPR121_WATER_ELECTRODE);
      if (isfinite(capSample)) {
        capRawSum += capSample;
        capCount++;
      }
    }

    delay(SENSOR_TARE_DELAY_MS);
  }

  // Behavior: commit average tare only when at least one valid sample exists.
  if (adxlCount > 0) {
    adxlXTareMs2 = adxlXSum / (float)adxlCount;
    adxlYTareMs2 = adxlYSum / (float)adxlCount;
    adxlZTareMs2 = adxlZSum / (float)adxlCount;
  }
  if (imu1AccCount > 0) {
    imu1AxTareMs2 = imu1AxSum / (float)imu1AccCount;
    imu1AyTareMs2 = imu1AySum / (float)imu1AccCount;
    imu1AzTareMs2 = imu1AzSum / (float)imu1AccCount;
  }
  if (imu1GyroCount > 0) {
    imu1GxTareRadS = imu1GxSum / (float)imu1GyroCount;
    imu1GyTareRadS = imu1GySum / (float)imu1GyroCount;
    imu1GzTareRadS = imu1GzSum / (float)imu1GyroCount;
  }
  if (imu2AccCount > 0) {
    imu2AxTareMs2 = imu2AxSum / (float)imu2AccCount;
    imu2AyTareMs2 = imu2AySum / (float)imu2AccCount;
    imu2AzTareMs2 = imu2AzSum / (float)imu2AccCount;
  }
  if (imu2GyroCount > 0) {
    imu2GxTareRadS = imu2GxSum / (float)imu2GyroCount;
    imu2GyTareRadS = imu2GySum / (float)imu2GyroCount;
    imu2GzTareRadS = imu2GzSum / (float)imu2GyroCount;
  }
  if (bmpCount > 0) {
    bmpTempTareC = bmpTempSum / (float)bmpCount;
    bmpPressTareHpa = bmpPressSum / (float)bmpCount;
    bmpAltTareM = bmpAltSum / (float)bmpCount;
  }
  if (capCount > 0) {
    capStartupBaselineRaw = capRawSum / (float)capCount;
    capStartupTareValid = true;

    if (CAP_TARE_ALIGN_TO_AIR) {
      float requestedOffset = CAP_CAL_AIR_RAW - capStartupBaselineRaw;
      float absRequestedOffset = fabsf(requestedOffset);
      if (absRequestedOffset >= CAP_TARE_OFFSET_APPLY_THRESHOLD_RAW &&
          absRequestedOffset <= CAP_TARE_MAX_ABS_OFFSET_RAW) {
        capRawAlignOffset = requestedOffset;
      } else {
        capRawAlignOffset = 0.0f;
      }
    }
  }

  // Behavior: print final tare values for bench verification/debugging.
  Serial.print("ADXL X tare (m/s^2): ");
  Serial.println(adxlXTareMs2, 4);
  Serial.print("ADXL Y tare (m/s^2): ");
  Serial.println(adxlYTareMs2, 4);
  Serial.print("ADXL Z tare (m/s^2): ");
  Serial.println(adxlZTareMs2, 4);
  Serial.print("IMU1 accel tare (m/s^2): ");
  Serial.print(imu1AxTareMs2, 4);
  Serial.print(", ");
  Serial.print(imu1AyTareMs2, 4);
  Serial.print(", ");
  Serial.println(imu1AzTareMs2, 4);
  Serial.print("IMU1 gyro tare (rad/s): ");
  Serial.print(imu1GxTareRadS, 4);
  Serial.print(", ");
  Serial.print(imu1GyTareRadS, 4);
  Serial.print(", ");
  Serial.println(imu1GzTareRadS, 4);
  Serial.print("IMU2 accel tare (m/s^2): ");
  Serial.print(imu2AxTareMs2, 4);
  Serial.print(", ");
  Serial.print(imu2AyTareMs2, 4);
  Serial.print(", ");
  Serial.println(imu2AzTareMs2, 4);
  Serial.print("IMU2 gyro tare (rad/s): ");
  Serial.print(imu2GxTareRadS, 4);
  Serial.print(", ");
  Serial.print(imu2GyTareRadS, 4);
  Serial.print(", ");
  Serial.println(imu2GzTareRadS, 4);
  Serial.print("BMP tare (C,hPa,m): ");
  Serial.print(bmpTempTareC, 4);
  Serial.print(", ");
  Serial.print(bmpPressTareHpa, 4);
  Serial.print(", ");
  Serial.println(bmpAltTareM, 4);
  Serial.print("CAP startup baseline raw: ");
  Serial.println(capStartupBaselineRaw, 2);
  Serial.print("CAP raw alignment offset: ");
  Serial.println(capRawAlignOffset, 2);
  sensorTareComplete = true;
}

/**
 * @brief Stop logging and return system to idle/safe state.
 *
 * Flushes and closes the log file, disables outputs as needed, and powers down
 * sensor rail.
 *
 * @param reason Human-readable reason printed to serial console.
 */
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

/**
 * @brief Send a framed LoRa command/telemetry packet.
 *
 * Packet format: destination, source, message ID, payload length, payload.
 *
 * @param destination LoRa recipient address.
 * @param outgoing Payload string to transmit.
 */
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

/**
 * @brief Start a new logging session.
 *
 * Powers sensors, initializes devices, performs tare calibration, creates a log
 * file, and resets runtime timers/state.
 */
void startLogging() {
  // Behavior: ignore duplicate START requests while already active.
  if (loggingActive) {
    return;
  }

  // Behavior: bring hardware online and calibrate before opening log file.
  sensorTareComplete = false;
  powerSensorsOn();
  initSensors();
  calibrateSensorTares();

  // Behavior: abort startup safely if log file cannot be created.
  if (!createNextLogFile()) {
    Serial.println("Could not create log file");
    powerSensorsOff();
    return;
  }

  // Behavior: initialize session timing/state baselines.
  startTime = millis();
  currentFileStartMs = startTime;
  lastLogMs = startTime;
  lastFlushMs = startTime;
  lastLiveTimeReportMs = startTime;
  lastRfTimeReportMs = startTime;
  lastFlowUpdateMs = startTime;
  flowHz = 0.0f;
  flowLpm = 0.0f;
  filteredWaterPct = NAN;
  filteredCapRaw = NAN;
  lastWaterFracForFlow = NAN;
  lastCapEstimateMs = 0;
  if (!capStartupTareValid) {
    capStartupBaselineRaw = NAN;
    capRawAlignOffset = 0.0f;
  }
  // Behavior: clear flow pulse counter atomically before sampling begins.
  noInterrupts();
  flowPulseCount = 0;
  interrupts();
  resetFlightDetectionState();
  flightLogicReadyMs = startTime + FLIGHT_LOGIC_ARM_DELAY_MS;
  loggingActive = true;
  Serial.println("Logging started");
}

/**
 * @brief Rotate to a new log file segment.
 *
 * Closes current file and creates the next `logNNN.csv` file.
 *
 * @param nowMs Current `millis()` timestamp used to reset segment start time.
 * @return true if rotation succeeded.
 * @return false if rotation failed and logging was stopped.
 */
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

/**
 * @brief Format elapsed milliseconds as `hh:mm:ss.mmm`.
 *
 * @param elapsedMs Elapsed session time in milliseconds.
 * @param out Destination buffer.
 * @param outLen Destination buffer length.
 */
void formatElapsedTime(unsigned long elapsedMs, char *out, size_t outLen) {
  unsigned long hours = elapsedMs / 3600000UL;
  unsigned long minutes = (elapsedMs % 3600000UL) / 60000UL;
  unsigned long seconds = (elapsedMs % 60000UL) / 1000UL;
  unsigned long millisPart = elapsedMs % 1000UL;
  snprintf(out, outLen, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds,
           millisPart);
}

/**
 * @brief Format current local RTC time as `M/D/YYYY h:mm:ss AM/PM`.
 *
 * Writes `"NA"` when the RTC has not been set.
 *
 * @param out Destination buffer.
 * @param outLen Destination buffer length.
 */
void formatCurrentDateTime(char *out, size_t outLen) {
  if (timeStatus() == timeNotSet) {
    snprintf(out, outLen, "NA");
    return;
  }

  time_t nowTime = now();
  int hour24 = hour(nowTime);
  bool isPm = (hour24 >= 12);
  int hour12 = hour24 % 12;
  if (hour12 == 0) {
    hour12 = 12;
  }

  snprintf(out, outLen, "%d/%d/%d %d:%02d:%02d %s", month(nowTime),
           day(nowTime), year(nowTime), hour12, minute(nowTime),
           second(nowTime), isPm ? "PM" : "AM");
}

/**
 * @brief Periodically print live elapsed logging time to serial.
 *
 * Prints elapsed session time as `hh:mm:ss.mmm` and raw milliseconds while
 * logging is active.
 *
 * @param nowMs Current `millis()` timestamp.
 */
void reportLiveTime(unsigned long nowMs) {
  if (!ENABLE_LIVE_TIME_REPORT || !loggingActive) {
    return;
  }
  if (nowMs - lastLiveTimeReportMs < LIVE_TIME_REPORT_MS) {
    return;
  }

  lastLiveTimeReportMs = nowMs;
  unsigned long elapsedMs = nowMs - startTime;
  char elapsedText[16];
  char wallClockText[24];
  formatElapsedTime(elapsedMs, elapsedText, sizeof(elapsedText));
  formatCurrentDateTime(wallClockText, sizeof(wallClockText));

  Serial.print("Time ");
  Serial.print(wallClockText);
  Serial.print(" +");
  Serial.print(elapsedText);
  Serial.print(" (");
  Serial.print(elapsedMs);
  Serial.println(" ms)");
}

/**
 * @brief Periodically transmit elapsed logging time over LoRa.
 *
 * @param nowMs Current `millis()` timestamp.
 */
void reportLiveTimeRf(unsigned long nowMs) {
  if (!ENABLE_RF_TIME_REPORT || !loggingActive) {
    return;
  }
  if (nowMs - lastRfTimeReportMs < RF_TIME_REPORT_MS) {
    return;
  }

  lastRfTimeReportMs = nowMs;
  unsigned long elapsedMs = nowMs - startTime;
  char elapsedText[16];
  char wallClockText[24];
  char outgoing[80];
  formatElapsedTime(elapsedMs, elapsedText, sizeof(elapsedText));
  formatCurrentDateTime(wallClockText, sizeof(wallClockText));
  snprintf(outgoing, sizeof(outgoing), "TIME=%s,elapsed=%s,ms=%lu",
           wallClockText, elapsedText, elapsedMs);
  sendLoRaMessage(LORA_GROUND_ADDRESS, outgoing);
}

/**
 * @brief Parse and execute a control command.
 *
 * Supports start/stop/scan/valve/ping command set from serial or LoRa sources.
 *
 * @param cmd Command text to parse (normalized in-place).
 * @param source Command source label for diagnostics (e.g. "Serial", "LoRa").
 * @param sender Sender LoRa address used for replies (default: ground station).
 */
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

/**
 * @brief Handle inbound LoRa command packets.
 *
 * Supports framed packet format and backward-compatible plain-text packets.
 */
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

/**
 * @brief Handle one pending serial command line.
 *
 * Reads until newline and dispatches through the common command parser.
 */
void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  executeCommand(cmd, "Serial");
}

/**
 * @brief Acquire sensors and append one CSV log row.
 *
 * Reads enabled sensors, applies tare offsets, updates flight/valve logic, and
 * writes one formatted sample row to the current log file.
 *
 * @param nowMs Current `millis()` timestamp for this sample.
 */
void collectAndLogRow(unsigned long nowMs) {
  char wallClockText[24];
  formatCurrentDateTime(wallClockText, sizeof(wallClockText));

  // Default each channel to NAN so missing sensors are explicit in CSV.
  float adxlX = NAN, adxlY = NAN, adxlZ = NAN;
  float imu1AxMs2 = NAN, imu1AyMs2 = NAN, imu1AzMs2 = NAN;
  float imu1GxRadS = NAN, imu1GyRadS = NAN, imu1GzRadS = NAN;
  float imu1AccelNormG = NAN;
  float imu2AxMs2 = NAN, imu2AyMs2 = NAN, imu2AzMs2 = NAN;
  float imu2GxRadS = NAN, imu2GyRadS = NAN, imu2GzRadS = NAN;
  float imu2AccelNormG = NAN;
  float tempC = NAN, pressHpa = NAN, altM = NAN, accelMagMs2 = NAN;
  float capRaw = NAN;
  float waterPct = NAN;
  uint32_t sensorStatus = 0;

  // Bitfield packs sensor availability + flight state into one CSV column.
  // Behavior: ADXL read + tare correction.
  if (adxlOk) {
    sensors_event_t event;
    accel.getEvent(&event);
    adxlX = event.acceleration.x - adxlXTareMs2;
    adxlY = event.acceleration.y - adxlYTareMs2;
    adxlZ = (event.acceleration.z - adxlZTareMs2) + GRAVITY_MS2;
    sensorStatus |= (1u << 0);
  }

  // Behavior: IMU1 read, compute accel norm, and apply gyro tare.
  if (imu1Ok) {
    sensors_event_t imu1AccelEvent;
    sensors_event_t imu1GyroEvent;
    sensors_event_t imu1TempEvent;
    imu1.getEvent(&imu1AccelEvent, &imu1GyroEvent, &imu1TempEvent);

    if (isfinite(imu1AccelEvent.acceleration.x) &&
        isfinite(imu1AccelEvent.acceleration.y) &&
        isfinite(imu1AccelEvent.acceleration.z)) {
      imu1AxMs2 = imu1AccelEvent.acceleration.x;
      imu1AyMs2 = imu1AccelEvent.acceleration.y;
      imu1AzMs2 = imu1AccelEvent.acceleration.z;
      imu1AccelNormG =
          vectorMagnitude3(imu1AxMs2, imu1AyMs2, imu1AzMs2) / GRAVITY_MS2;
    }
    if (isfinite(imu1GyroEvent.gyro.x) && isfinite(imu1GyroEvent.gyro.y) &&
        isfinite(imu1GyroEvent.gyro.z)) {
      imu1GxRadS = imu1GyroEvent.gyro.x - imu1GxTareRadS;
      imu1GyRadS = imu1GyroEvent.gyro.y - imu1GyTareRadS;
      imu1GzRadS = imu1GyroEvent.gyro.z - imu1GzTareRadS;
    }
    sensorStatus |= (1u << 1);
  }

  // Behavior: IMU2 read, compute accel norm, and apply gyro tare.
  if (imu2Ok) {
    sensors_event_t imu2AccelEvent;
    sensors_event_t imu2GyroEvent;
    sensors_event_t imu2TempEvent;
    imu2.getEvent(&imu2AccelEvent, &imu2GyroEvent, &imu2TempEvent);

    if (isfinite(imu2AccelEvent.acceleration.x) &&
        isfinite(imu2AccelEvent.acceleration.y) &&
        isfinite(imu2AccelEvent.acceleration.z)) {
      imu2AxMs2 = imu2AccelEvent.acceleration.x;
      imu2AyMs2 = imu2AccelEvent.acceleration.y;
      imu2AzMs2 = imu2AccelEvent.acceleration.z;
      imu2AccelNormG =
          vectorMagnitude3(imu2AxMs2, imu2AyMs2, imu2AzMs2) / GRAVITY_MS2;
    }
    if (isfinite(imu2GyroEvent.gyro.x) && isfinite(imu2GyroEvent.gyro.y) &&
        isfinite(imu2GyroEvent.gyro.z)) {
      imu2GxRadS = imu2GyroEvent.gyro.x - imu2GxTareRadS;
      imu2GyRadS = imu2GyroEvent.gyro.y - imu2GyTareRadS;
      imu2GzRadS = imu2GyroEvent.gyro.z - imu2GzTareRadS;
    }
    sensorStatus |= (1u << 2);
  }

  // Behavior: BMP read + tare correction when fresh sample is available.
  if (bmpOk && bmp.performReading()) {
    tempC = bmp.temperature - bmpTempTareC;
    pressHpa = (bmp.pressure / 100.0f) - bmpPressTareHpa;
    altM = bmp.readAltitude(SEALEVELPRESSURE_HPA) - bmpAltTareM;
    sensorStatus |= (1u << 3);
  }

  // Behavior: read capacitive water channel and estimate linear water percent.
  if (capOk) {
    capRaw = (float)cap.filteredData(MPR121_WATER_ELECTRODE);
    float capRawAdjusted = capRaw + capRawAlignOffset;
    if (!isfinite(filteredCapRaw)) {
      filteredCapRaw = capRawAdjusted;
    } else {
      filteredCapRaw +=
          CAP_RAW_FILTER_ALPHA * (capRawAdjusted - filteredCapRaw);
    }

    float waterFrac = estimateWaterFractionFromCapRaw(filteredCapRaw);
    if (isfinite(waterFrac)) {
      float rawWaterPct = waterFrac * 100.0f;
      if (!isfinite(filteredWaterPct)) {
        filteredWaterPct = rawWaterPct;
      } else {
        filteredWaterPct +=
            CAP_WATER_PCT_ALPHA * (rawWaterPct - filteredWaterPct);
      }
      waterPct = filteredWaterPct;

      // Effective dielectric estimate can be useful while tuning calibration.
      float effectivePermittivity =
          DIELECTRIC_AIR + waterFrac * (DIELECTRIC_WATER - DIELECTRIC_AIR);
      (void)effectivePermittivity;

      // If the pulse flow meter is not connected, publish a capacitive
      // activity-based flow estimate in flow_lpm.
      if (!ENABLE_FLOW_SENSOR && ENABLE_CAP_ESTIMATED_FLOW) {
        if (lastCapEstimateMs != 0 && isfinite(lastWaterFracForFlow)) {
          unsigned long dtMs = nowMs - lastCapEstimateMs;
          if (dtMs > 0) {
            float waterFracForFlow = filteredWaterPct * 0.01f;
            float deltaFrac = waterFracForFlow - lastWaterFracForFlow;
            if (CAP_ESTIMATED_FLOW_USE_ABS_DELTA) {
              deltaFrac = fabsf(deltaFrac);
            } else if (deltaFrac < 0.0f) {
              deltaFrac = 0.0f;
            }
            if (deltaFrac < CAP_FLOW_FRAC_DEADBAND) {
              deltaFrac = 0.0f;
            }
            flowHz = deltaFrac * (1000.0f / (float)dtMs);
            flowLpm =
                deltaFrac * CAP_SENSOR_VOLUME_L * (60000.0f / (float)dtMs);
            if (flowLpm > CAP_ESTIMATED_FLOW_MAX_LPM) {
              flowLpm = CAP_ESTIMATED_FLOW_MAX_LPM;
            }
          }
        }
        lastWaterFracForFlow = filteredWaterPct * 0.01f;
        lastCapEstimateMs = nowMs;
      }
    }
    sensorStatus |= (1u << 4);
  }
  // Behavior: mark presence/status bits for non-sampled subsystems.
  if (flowOk) {
    sensorStatus |= (1u << 5);
  }

  // Behavior: derived acceleration magnitude used by flight logic and logs.
  tryGetAccelMagnitudeMs2(imu1AxMs2, imu1AyMs2, imu1AzMs2, imu2AxMs2, imu2AyMs2,
                          imu2AzMs2, accelMagMs2);

  // Behavior: update launch/microgravity/valve state for this sample.
  updateFlightValveLogic(nowMs, imu1AxMs2, imu1AyMs2, imu1AzMs2, imu2AxMs2,
                         imu2AyMs2, imu2AzMs2);

  // Behavior: encode current flight-state flags into status bitfield.
  if (valveOpen) {
    sensorStatus |= (1u << 6);
  }
  if (launchDetected) {
    sensorStatus |= (1u << 7);
  }
  if (microgravityNow) {
    sensorStatus |= (1u << 8);
  }

  // Behavior: append one complete, fixed-column CSV row.
  logFile.printf(
      "%s,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,"
      "%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%u,%"
      "lu\n",
      wallClockText, nowMs, adxlX, adxlY, adxlZ, imu1AxMs2, imu1AyMs2,
      imu1AzMs2, imu1GxRadS, imu1GyRadS, imu1GzRadS, imu1AccelNormG, imu2AxMs2,
      imu2AyMs2, imu2AzMs2, imu2GxRadS, imu2GyRadS, imu2GzRadS, imu2AccelNormG,
      tempC, pressHpa, altM, accelMagMs2, flowHz, flowLpm, capRaw, waterPct,
      valveOpen ? 1u : 0u, (unsigned long)sensorStatus);
}

/**
 * @brief Arduino setup routine.
 *
 * Initializes pins, buses, storage, radio, and optionally starts logging.
 */
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
  setSyncProvider(getRtcTime);
  setSyncInterval(60);
  if (timeStatus() == timeSet) {
    Serial.println("RTC time synced");
  } else {
    Serial.println("RTC time not set");
  }

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

/**
 * @brief Arduino main loop.
 *
 * Processes incoming commands, executes periodic logging/flow updates, and
 * handles flush, file rotation, and auto-stop conditions.
 */
void loop() {
  unsigned long nowMs = millis();

  // Behavior: command channels are always serviced, even when not logging.
  handleLoRaCommands();
  handleSerialCommands();

  // Behavior: idle fast when logging is inactive.
  if (!loggingActive) {
    return;
  }

  // START may have been processed above; refresh so time deltas are computed
  // against the same or newer timestamp than startTime/lastLogMs.
  nowMs = millis();
  reportLiveTime(nowMs);
  reportLiveTimeRf(nowMs);

  // Behavior: run periodic sampling/logging at configured interval.
  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    updateFlowStats(nowMs);
    collectAndLogRow(nowMs);
  }

  // Behavior: stop request from flight logic is handled immediately.
  if (stopLoggingRequested) {
    stopLoggingRequested = false;
    stopLogging("microgravity exited");
    return;
  }

  // Behavior: periodic SD flush to reduce data loss on sudden power loss.
  if (nowMs - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = nowMs;
    logFile.flush();
    Serial.println("SD flush");
  }

  // Behavior: rotate file by elapsed segment duration.
  if (nowMs - currentFileStartMs >= LOG_FILE_DURATION_MS) {
    if (!rotateLogFile(nowMs)) {
      return;
    }
    lastFlushMs = nowMs;
  }

  // Behavior: hard cap total logging duration as safety bound.
  if (nowMs - startTime >= MAX_LOG_DURATION_MS) {
    stopLogging("max duration reached");
  }
}
