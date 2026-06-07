#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// ESP32 DevKit V1 SPI pins
// If your SD CS is wired to GPIO5 instead, change SD_CS_PIN to 5.
static const int SD_SCK_PIN = 18;  // SD CLK
static const int SD_MISO_PIN = 19; // SD SO (MISO)
static const int SD_MOSI_PIN = 23; // SD SI (MOSI)
static const int SD_CS_PIN = 16;   // SD CS (safer than strap pin GPIO5)

static const uint32_t SD_SPI_HZ = 10000000U; // start conservative at 10 MHz
static const char *TEST_FILE = "/sd_test.txt";

bool initSdCard() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, SPI, SD_SPI_HZ)) {
    Serial.println("SD.begin failed");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card detected");
    return false;
  }

  const char *typeName = "UNKNOWN";
  if (cardType == CARD_MMC) {
    typeName = "MMC";
  } else if (cardType == CARD_SD) {
    typeName = "SDSC";
  } else if (cardType == CARD_SDHC) {
    typeName = "SDHC/SDXC";
  }

  uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
  uint64_t usedMB = SD.usedBytes() / (1024ULL * 1024ULL);
  uint64_t totalMB = SD.totalBytes() / (1024ULL * 1024ULL);

  Serial.printf("SD mounted. Type=%s CardSize=%lluMB FS=%llu/%lluMB used\n",
                typeName, cardSizeMB, usedMB, totalMB);
  return true;
}

void listRoot() {
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open root directory");
    return;
  }

  Serial.println("Root directory:");
  File entry = root.openNextFile();
  while (entry) {
    if (entry.isDirectory()) {
      Serial.printf("  <DIR>  %s\n", entry.name());
    } else {
      Serial.printf("  <FILE> %s  (%u bytes)\n", entry.name(),
                    (unsigned)entry.size());
    }
    entry = root.openNextFile();
  }
}

bool runSelfTest() {
  const uint32_t now = millis();
  char line[96];
  snprintf(line, sizeof(line), "ESP32 SD self-test at %lu ms\n",
           (unsigned long)now);

  File out = SD.open(TEST_FILE, FILE_WRITE);
  if (!out) {
    Serial.println("Failed to open test file for write");
    return false;
  }
  size_t written = out.print(line);
  out.close();

  if (written != strlen(line)) {
    Serial.printf("Write failed (%u/%u bytes)\n", (unsigned)written,
                  (unsigned)strlen(line));
    return false;
  }

  File in = SD.open(TEST_FILE, FILE_READ);
  if (!in) {
    Serial.println("Failed to open test file for read");
    return false;
  }

  String content = in.readString();
  in.close();

  if (content.length() == 0) {
    Serial.println("Read back empty content");
    return false;
  }

  Serial.println("Self-test PASS");
  Serial.printf("Read: %s", content.c_str());
  return true;
}

void deleteTestFile() {
  if (SD.exists(TEST_FILE)) {
    if (SD.remove(TEST_FILE)) {
      Serial.printf("Deleted %s\n", TEST_FILE);
    } else {
      Serial.printf("Failed to delete %s\n", TEST_FILE);
    }
  } else {
    Serial.printf("%s does not exist\n", TEST_FILE);
  }
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  t = run write/read self-test");
  Serial.println("  l = list root directory");
  Serial.println("  d = delete /sd_test.txt");
  Serial.println("  h = print help");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32 SD card test");
  Serial.printf("Pins: SCK=%d MISO=%d MOSI=%d CS=%d\n", SD_SCK_PIN, SD_MISO_PIN,
                SD_MOSI_PIN, SD_CS_PIN);

  if (!initSdCard()) {
    Serial.println("SD init failed. Check wiring and power (3.3V only).");
    while (true) {
      delay(1000);
    }
  }

  runSelfTest();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == 't' || cmd == 'T') {
      runSelfTest();
    } else if (cmd == 'l' || cmd == 'L') {
      listRoot();
    } else if (cmd == 'd' || cmd == 'D') {
      deleteTestFile();
    } else if (cmd == 'h' || cmd == 'H') {
      printHelp();
    }
  }
}
