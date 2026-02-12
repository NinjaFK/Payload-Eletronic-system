#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Sensors
#include <Adafruit_Sensor.h>
// #include <Adafruit_LSM6DSO32.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPR121.h>

// RF
#include <LoRa.h>

#define ADXL375_CS 10

// Sensor objects
// Adafruit_LSM6DSO32 imu;
Adafruit_ADXL375 imu = Adafruit_ADXL375(ADXL375_CS);
Adafruit_BMP3XX bmp;
Adafruit_MPR121 cap = Adafruit_MPR121();

// SD

SdFs sd;
FsFile logFile;
#define SD_CONFIG SdioConfig(FIFO_SDIO)

void setup()
{
  delay(5000);
  // Serial (output)
  Serial.begin(115200);

  // Waits for serial to connect
  while (!Serial && millis() < 3000)
    ;

  // SD card
  if (!sd.begin(SD_CONFIG))
  {
    Serial.println("SD init failed");
    while (1)
      ;
  }
  logFile = sd.open("log000.txt", FILE_WRITE);

  Wire.begin(); // defaults to SDA=18, SCL=19 on Teensy 4.1

  if (!imu.begin())
  { // use the I2C interface over STEMMA QT
    Serial.println("ADXL375 not detected!");
    while (1)
      ;
  }

  Serial.println("Normal");
}

void loop()
{
  sensors_event_t event;
  imu.getEvent(&event);
}
