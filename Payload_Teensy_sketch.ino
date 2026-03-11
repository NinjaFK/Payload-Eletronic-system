#include <Arduino.h>
#include <SD.h>
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

#define ADXL375_CS 10
#define SD_CONFIG SdioConfig(FIFO_SDIO)

float x, y, z;

// Sensor objects
Adafruit_ADXL375 accel = Adafruit_ADXL375(ADXL375_CS);
Adafruit_BMP3XX bmp;
Adafruit_MPR121 cap = Adafruit_MPR121();

// BMP
#define SEALEVELPRESSURE_HPA (1013.25)

// SD
#include "SdFat.h"
SdFs sd;
FsFile logFile;
#define SD_CONFIG SdioConfig(FIFO_SDIO)
unsigned long startTime;
const unsigned long LOG_DURATION_MS = 10000;

void displayDataRate(void) {
  Serial.print("Data Rate:    ");

  switch (accel.getDataRate()) {
  case ADXL343_DATARATE_3200_HZ:
    Serial.print("3200 ");
    break;
  case ADXL343_DATARATE_1600_HZ:
    Serial.print("1600 ");
    break;
  case ADXL343_DATARATE_800_HZ:
    Serial.print("800 ");
    break;
  case ADXL343_DATARATE_400_HZ:
    Serial.print("400 ");
    break;
  case ADXL343_DATARATE_200_HZ:
    Serial.print("200 ");
    break;
  case ADXL343_DATARATE_100_HZ:
    Serial.print("100 ");
    break;
  case ADXL343_DATARATE_50_HZ:
    Serial.print("50 ");
    break;
  case ADXL343_DATARATE_25_HZ:
    Serial.print("25 ");
    break;
  case ADXL343_DATARATE_12_5_HZ:
    Serial.print("12.5 ");
    break;
  case ADXL343_DATARATE_6_25HZ:
    Serial.print("6.25 ");
    break;
  case ADXL343_DATARATE_3_13_HZ:
    Serial.print("3.13 ");
    break;
  case ADXL343_DATARATE_1_56_HZ:
    Serial.print("1.56 ");
    break;
  case ADXL343_DATARATE_0_78_HZ:
    Serial.print("0.78 ");
    break;
  case ADXL343_DATARATE_0_39_HZ:
    Serial.print("0.39 ");
    break;
  case ADXL343_DATARATE_0_20_HZ:
    Serial.print("0.20 ");
    break;
  case ADXL343_DATARATE_0_10_HZ:
    Serial.print("0.10 ");
    break;
  default:
    Serial.print("???? ");
    break;
  }
  Serial.println(" Hz");
}

void setup() {
  // Serial (output)
  Serial.begin(115200);

  // Waits for serial to connect
  while (!Serial && millis() < 3000)
    ;

  // SD card
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD init failed");
    while (1)
      ;
  }

  if (sd.exists("log000.txt")) {
    Serial.println("Deleting old log000.txt");
    sd.remove("log000.txt");
  }

  FsFile f = sd.open("log000.txt", O_WRITE | O_CREAT | O_TRUNC);

  if (!f) {
    Serial.println("Open/create failed");
    while (1)
      ;
  }

  f.close();

  logFile = sd.open("log000.txt", FILE_WRITE);

  if (!logFile) {
    Serial.println("Could not open log file");
    while (1)
      ;
    Serial.println("Could not open log file");
  }
  logFile.println("millis,ax,ay,az");
  logFile.flush();

  Wire.begin(); // defaults to SDA=18, SCL=19 on Teensy 4.1

  // Serial.println("ADXL375 Accelerometer Test");
  // Serial.println("");

  /*
  ADXL375 Accelerometer Test

  Normal
  ------------------------------------
  Sensor:       ADXL375
  Type:         Acceleration (m/s2)
  Driver Ver:   1
  Unique ID:    10
  Min Value:    -1961.33
  Max Value:    1961.33
  Resolution:   0.48
  ------------------------------------

  Data Rate:    100  Hz
  */
  if (!accel.begin()) { // use the I2C interface over STEMMA QT
    Serial.println("ADXL375 not detected!");
    while (0)
      ;
  } else {
    Serial.println("ADXL375 detected!");
  }

  // accel.printSensorDetails();
  // displayDataRate();
  // Serial.println("");

  // Accel LSM6DSO32
  if (!IMU.begin()) { // use the I2C interface over STEMMA QT
    Serial.println("LSM6DSO32 not detected!");
    while (0)
      ;
  } else {
    Serial.println("LSM6DSO32 detected!");
  }

  if (!cap.begin()) {
    Serial.println("MPR121 not detected!");
    while (0)
      ;
  } else {
    Serial.println("MPR121 detected!");
  }

  /*ADDED CODE FOR BMP388
  if (!bmp.begin_I2C())
  {
    Serial.println("BMP388 not detected!");
    while (0);
  }

  // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  /**/

  /*NOTE: im not sure what oversampling or filter rate would
  be best so below is a list of all the possible settings, although
  i would assume higher oversampling and filtering would be better
  as it would not have to run for long

  BMP3_NO_OVERSAMPLING
  BMP3_OVERSAMPLING_2X
  BMP3_OVERSAMPLING_4X
  BMP3_OVERSAMPLING_8X
  BMP3_OVERSAMPLING_16X
  BMP3_OVERSAMPLING_32X

  BMP3_IIR_FILTER_DISABLE (no filtering)
  BMP3_IIR_FILTER_COEFF_1
  BMP3_IIR_FILTER_COEFF_3
  BMP3_IIR_FILTER_COEFF_7
  BMP3_IIR_FILTER_COEFF_15
  BMP3_IIR_FILTER_COEFF_31
  BMP3_IIR_FILTER_COEFF_63
  BMP3_IIR_FILTER_COEFF_127

  BMP3_ODR_200_HZ, BMP3_ODR_100_HZ, BMP3_ODR_50_HZ, BMP3_ODR_25_HZ,
  BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ, BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ,
  BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ,BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ,
  BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ, BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ,
  BMP3_ODR_0_003_HZ, or BMP3_ODR_0_001_HZ
  */
}

// Adxl375 collecting
void collectAdxlData() {
  sensors_event_t event;

  accel.getEvent(&event);

  // Serial.print("X: ");
  // Serial.print(event.acceleration.x);
  // Serial.print("  ");
  // Serial.print("Y: ");
  // Serial.print(event.acceleration.y);
  // Serial.print("  ");
  // Serial.print("Z: ");
  // Serial.print(event.acceleration.z);
  // Serial.print("  ");
  // Serial.println("m/s^2 ");
  Serial.printf("%lu,%.4f,%.4f,%.4f\n", t, event.acceleration.x,
                event.acceleration.y, event.acceleration.z);

  logFile.printf("%lu,%.4f,%.4f,%.4f\n", t, event.acceleration.x,
                 event.acceleration.y, event.acceleration.z);
}

void loop() {
  unsigned long t = millis();

  if (t % 1000 > 500) {
    Serial.println("Wrote to SD");
    logFile.flush();
  }

  // ADXL375
  collectAdxlData();
  // ####################

  // LSM6DSO32
  // ####################

  // if (IMU.accelerationAvailable())
  // {
  //   IMU.readAcceleration(x, y, z);

  //   Serial.print(t);
  //   Serial.print(",");
  //   Serial.print(x, 4);
  //   Serial.print(",");
  //   Serial.print(y, 4);
  //   Serial.print(",");
  //   Serial.println(z, 4);

  //   logFile.printf("%lu,%.4f,%.4f,%.4f\n", t, x, y, z);

  //   if (t % 1000 > 500)
  //   {
  //     Serial.println("Wrote to SD");
  //     logFile.flush();
  //   }
  // }

  if (t - startTime > LOG_DURATION_MS) {
    logFile.flush();
    logFile.close();
    Serial.println("Logging complete — file closed.");
    while (1)
      ;
  }
  // ####################

  // ####################

  /*FURTHER ADDED CODE FOR BMP388
  if (! bmp.performReading()) {
    Serial.println("Failed to read MBP388");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  */

  delay(500);
}
