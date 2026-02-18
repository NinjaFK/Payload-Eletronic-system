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
Adafruit_ADXL375 accel = Adafruit_ADXL375(ADXL375_CS);
Adafruit_BMP3XX bmp;
Adafruit_MPR121 cap = Adafruit_MPR121();

// SD

SdFs sd;
FsFile logFile;
#define SD_CONFIG SdioConfig(FIFO_SDIO)

void displayDataRate(void)
{
  Serial.print("Data Rate:    ");

  switch (accel.getDataRate())
  {
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

void setup()
{
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

  Serial.println("ADXL375 Accelerometer Test");
  Serial.println("");

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
  if (!accel.begin())
  { // use the I2C interface over STEMMA QT
    Serial.println("ADXL375 not detected!");
    while (0)
      ;
  }

  // accel.printSensorDetails();
  // displayDataRate();
  // Serial.println("");

  if (!cap.begin())
  {
    Serial.println("MPR121 not detected!");
    while (0)
      ;
  }
}

void loop()
{
  sensors_event_t event;
  // accel.getEvent(&event);

  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(event.acceleration.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(event.acceleration.z);
  Serial.print("  ");
  Serial.println("m/s^2 ");
  delay(500);
}
