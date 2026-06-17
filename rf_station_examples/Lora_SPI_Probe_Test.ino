#include <LoRa.h>
#include <SPI.h>

// Match sender sketch pin definitions (initial/default).
const int csPin = 4;
const int resetPin = 2;
const int irqPin = 3;

// Set to your radio enable pin if used; keep -1 if EN is hard-wired high.
const int enPin = -1;

const long rfFrequencyHz = 421480000;
const int csCandidates[] = {4, 10, 9, 8, 7, 6, 5};
const size_t csCandidateCount = sizeof(csCandidates) / sizeof(csCandidates[0]);

uint8_t spiReadRegWithCs(int activeCsPin, uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeCsPin, LOW);
  SPI.transfer(reg & 0x7F); // read
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(activeCsPin, HIGH);
  SPI.endTransaction();
  return value;
}

uint8_t spiReadReg(uint8_t reg) { return spiReadRegWithCs(csPin, reg); }

void hardResetRadio() {
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
  delay(10);
  digitalWrite(resetPin, LOW);
  delay(10);
  digitalWrite(resetPin, HIGH);
  delay(10);
}

void printHexReg(const char *name, uint8_t reg) {
  uint8_t value = spiReadReg(reg);
  Serial.print(name);
  Serial.print(" (0x");
  Serial.print(reg, HEX);
  Serial.print(") = 0x");
  if (value < 0x10)
    Serial.print('0');
  Serial.println(value, HEX);
}

void scanChipSelectPins() {
  Serial.println("Scanning CS candidates for RegVersion=0x12...");
  int matches = 0;

  for (size_t i = 0; i < csCandidateCount; i++) {
    int testCs = csCandidates[i];
    pinMode(testCs, OUTPUT);
    digitalWrite(testCs, HIGH);
    delay(1);

    uint8_t version = spiReadRegWithCs(testCs, 0x42);
    Serial.print("  CS D");
    Serial.print(testCs);
    Serial.print(" -> RegVersion 0x");
    if (version < 0x10)
      Serial.print('0');
    Serial.println(version, HEX);

    if (version == 0x12) {
      matches++;
    }
  }

  if (matches == 0) {
    Serial.println("No candidate returned 0x12.");
  } else {
    Serial.println("One or more candidates returned 0x12. Use that CS pin.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== LoRa SPI Probe Test ===");
  Serial.print("Pins: CS=");
  Serial.print(csPin);
  Serial.print(" RST=");
  Serial.print(resetPin);
  Serial.print(" IRQ=");
  Serial.println(irqPin);

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  if (enPin >= 0) {
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, HIGH);
    Serial.print("EN pin forced HIGH on D");
    Serial.println(enPin);
  } else {
    Serial.println("EN pin not controlled in this sketch");
  }

  SPI.begin();
  hardResetRadio();

  // SX127x expected RegVersion is 0x12.
  uint8_t version = spiReadReg(0x42);
  Serial.print("RegVersion (0x42) = 0x");
  if (version < 0x10)
    Serial.print('0');
  Serial.println(version, HEX);

  printHexReg("RegOpMode", 0x01);
  printHexReg("RegPaConfig", 0x09);

  if (version == 0x12) {
    Serial.println("SPI probe PASS: SX127x detected.");
  } else {
    Serial.println("SPI probe FAIL: expected 0x12.");
    Serial.println("Likely issue: CS pin, power/EN, reset, or wrong module/chip.");
    scanChipSelectPins();
  }

  LoRa.setPins(csPin, resetPin, irqPin);
  LoRa.setSPIFrequency(1000000);

  Serial.print("LoRa.begin(");
  Serial.print(rfFrequencyHz);
  Serial.println(") ...");

  if (LoRa.begin(rfFrequencyHz)) {
    Serial.println("LoRa.begin PASS");
  } else {
    Serial.println("LoRa.begin FAIL");
  }
}

void loop() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs >= 2000) {
    lastMs = millis();
    uint8_t version = spiReadReg(0x42);
    Serial.print("heartbeat RegVersion=0x");
    if (version < 0x10)
      Serial.print('0');
    Serial.println(version, HEX);
  }
}
