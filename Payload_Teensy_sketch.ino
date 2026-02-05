#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Sensors
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPR121.h>

// RF
#include <LoRa.h>

// Sensor objects
Adafruit_LSM6DSO32 imu;
Adafruit_ADXL375  adxl = Adafruit_ADXL375();
Adafruit_BMP3XX   bmp;
Adafruit_MPR121   cap = Adafruit_MPR121();

// SD

SdFs sd;
FsFile logFile;
#define SD_CONFIG SdioConfig(FIFO_SDIO)

void setup() {
  // Serial (output)
  Serial.begin(115200);

  // Waits for serial to connect
  while (!Serial);

  // SD card
  if(!sd.begin(SD_CONFIG)) {
      Serial.println("SD init failed"); while(1);
  }
  logFile = sd.open("log000.txt", FILE_WRITE);



}

void loop() {
  // put your main code here, to run repeatedly:

}
