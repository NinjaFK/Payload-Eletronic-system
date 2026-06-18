#include <LiquidCrystal_I2C.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>

// LoRa pins
const int csPin = 4;
const int resetPin = 2;
const int irqPin = 3;
const long loraBaseFrequencyHz = 421480000L;
const long loraFrequencyStepHz = 125000L;
const long loraMinFrequencyHz = 420600000L;
const long loraMaxFrequencyHz = 438000000L;

// LEDs
const int redLEDPin = 5;
const int blueLEDPin = 6;

// Button
const int buttonPin = 8;
int sendButtonState;
int lastButtonState = HIGH;
const bool enableButtonSend = false; // set true to allow button->START packets
unsigned long lastButtonEdgeMs = 0;
const unsigned long buttonDebounceMs = 40;

// Messages
String buttonPress = "button pressed";
String heartbeatMsg = "hb";

byte msgCount = 0;

// LCD Screen
LiquidCrystal_I2C lcd(0x27, 20, 4);

// -------- CHANGE PER DEVICE --------
byte localAddress = 0xCC;
byte destination = 0xA1; // payload address
const byte loraSyncWord = 0x12;
// ----------------------------------

// FSM
enum SystemState { IDLE, CONNECTED, LOST };
SystemState currentState = IDLE;

// Timing
unsigned long lastReceiveTime = 0;
const long signalTimeout = 5000;

unsigned long previousMillis = 0;

const long blueBlinkInterval = 1000;
const long idleBlinkInterval = 3000;
const long lostBlinkInterval = 500;

// Heartbeat timing
unsigned long lastHeartbeatTime = 0;
const long heartbeatInterval = 3000;

bool ledState = false;
long loraCurrentFrequencyHz = loraBaseFrequencyHz;

bool parseLongArg(const String &text, long &valueOut) {
  if (text.length() == 0) {
    return false;
  }
  int index = 0;
  if (text[0] == '+' || text[0] == '-') {
    if (text.length() == 1) {
      return false;
    }
    index = 1;
  }
  for (int i = index; i < text.length(); i++) {
    if (!isDigit(text[i])) {
      return false;
    }
  }
  valueOut = text.toInt();
  return true;
}

bool applyLoRaFrequencyStep(long step) {
  int64_t nextFreq = (int64_t)loraBaseFrequencyHz +
                     ((int64_t)step * (int64_t)loraFrequencyStepHz);
  if (nextFreq < loraMinFrequencyHz || nextFreq > loraMaxFrequencyHz) {
    return false;
  }

  LoRa.idle();
  LoRa.setFrequency((long)nextFreq);
  LoRa.receive();
  loraCurrentFrequencyHz = (long)nextFreq;

  Serial.print("LoRa frequency set to ");
  Serial.print(loraCurrentFrequencyHz);
  Serial.print(" Hz (step ");
  Serial.print(step);
  Serial.println(")");
  lcdLog("FREQ " + String(step));
  return true;
}

bool isOpenEvent(const String &msg) {
  return msg == "OPEN" || msg == "VALVE_ON" || msg == "VALVE_OPEN" ||
         msg == "VALVE=1";
}

bool isCloseEvent(const String &msg) {
  return msg == "CLOSE" || msg == "VALVE_OFF" || msg == "VALVE_CLOSED" ||
         msg == "VALVE=0";
}

// ---------- LCD log buffer ----------
String lcdLines[4] = {"", "", "", ""};

void lcdLog(String msg) {
  if (msg.length() > 20)
    msg = msg.substring(0, 20);

  lcdLines[0] = lcdLines[1];
  lcdLines[1] = lcdLines[2];
  lcdLines[2] = lcdLines[3];
  lcdLines[3] = msg;

  lcd.clear();
  for (int i = 0; i < 4; i++) {
    lcd.setCursor(0, i);
    lcd.print(lcdLines[i]);
  }
}

void handleSerialCommand() {
  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0)
    return;

  if (cmd.startsWith("SETLOCAL ") || cmd.startsWith("LOCALSET ")) {
    int argStart = cmd.startsWith("SETLOCAL ") ? 9 : 9;
    String stepText = cmd.substring(argStart);
    stepText.trim();
    long step = 0;

    if (!parseLongArg(stepText, step)) {
      Serial.println("Invalid local set. Use SETLOCAL <step>");
      lcdLog("SETLOCAL invalid");
      return;
    }

    if (!applyLoRaFrequencyStep(step)) {
      Serial.println("SETLOCAL out of range");
      lcdLog("SETLOCAL range");
      return;
    }

    Serial.println("Sender tuned locally");
    lcdLog("Sender tuned");
  } else if (cmd.startsWith("SET ")) {
    String stepText = cmd.substring(4);
    stepText.trim();
    long step = 0;

    if (!parseLongArg(stepText, step)) {
      Serial.println("Invalid SET command. Use SET <step>");
      lcdLog("SET invalid");
      return;
    }

    // Payload-only set. Sender retune can be done later with SETLOCAL <step>.
    sendMessage("SET " + String(step));
    LoRa.receive();
    Serial.println("Sent command: SET " + String(step));
    lcdLog("Sent SET " + String(step));
  } else if (cmd == "START" || cmd == "STOP" || cmd == "PING" ||
             cmd == "OPEN" || cmd == "CLOSE" || cmd == "SCAN") {
    sendMessage(cmd);
    LoRa.receive();
    Serial.println("Sent command: " + cmd);
    lcdLog("Sent command: " + cmd);
    if (isOpenEvent(cmd)) {
      Serial.println("VALVE OPEN command sent");
      lcdLog("VALVE OPEN command");
    } else if (isCloseEvent(cmd)) {
      Serial.println("VALVE CLOSE command sent");
      lcdLog("VALVE CLOSE command");
    }
  } else {
    Serial.println("Unknown command. Use START/STOP/PING/OPEN/CLOSE/SCAN/SET "
                   "<step>/SETLOCAL <step>");
    lcdLog("Unknown command");
  }
}

void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);

  Serial.begin(9600);

  // Start in IDLE
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(blueLEDPin, LOW);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(loraBaseFrequencyHz)) {
    Serial.println("LoRa failed!");
    lcdLog("LoRa failed!");
    while (1)
      ;
  }
  LoRa.setSyncWord(loraSyncWord);
  LoRa.enableCrc();
  loraCurrentFrequencyHz = loraBaseFrequencyHz;

  Serial.println("LoRa FSM with Heartbeat Ready");
  lcdLog("LoRa FSM Ready");
  Serial.println("USB commands: START/STOP/PING/OPEN/CLOSE/SCAN/SET "
                 "<step>/SETLOCAL <step>");
  lcdLog("USB: START/STOP/...");
}

void loop() {

  unsigned long currentMillis = millis();
  handleSerialCommand();
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }

  // -------- BUTTON SEND --------
  sendButtonState = digitalRead(buttonPin);
  if (enableButtonSend && lastButtonState == HIGH && sendButtonState == LOW &&
      (currentMillis - lastButtonEdgeMs) >= buttonDebounceMs) {
    lastButtonEdgeMs = currentMillis;
    sendMessage(buttonPress);
    delay(200);
  }
  lastButtonState = sendButtonState;

  // -------- HEARTBEAT SEND --------
  if (currentMillis - lastHeartbeatTime >= heartbeatInterval) {
    lastHeartbeatTime = currentMillis;

    sendMessage(heartbeatMsg);

    Serial.println("Heartbeat sent");
    lcdLog("Heartbeat sent");
  }

  // -------- STATE TRANSITIONS --------
  if (currentState == CONNECTED &&
      (currentMillis - lastReceiveTime > signalTimeout)) {

    Serial.println("Connection LOST");
    lcdLog("Connection LOST");

    currentState = LOST;
    previousMillis = millis();
  }

  // -------- STATE ACTIONS --------
  switch (currentState) {

  case IDLE:
    handleBlink(currentMillis, false, idleBlinkInterval);
    break;

  case CONNECTED:
    handleBlink(currentMillis, true, blueBlinkInterval);
    break;

  case LOST:
    handleBlink(currentMillis, false, lostBlinkInterval);
    break;
  }
}

// -------- SEND FUNCTION --------
void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(msgCount);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
  msgCount++;
}

// -------- RECEIVE FUNCTION --------
void onReceive(int packetSize) {
  if (packetSize == 0)
    return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  LoRa.read(); // msg ID
  byte incomingLength = LoRa.read();

  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
    return;
  if (recipient != localAddress)
    return;

  bool isTelemetry =
      incoming.startsWith("Time ") || incoming.startsWith("LoRa cmd: ") ||
      incoming.startsWith("Serial cmd: ") || incoming == "SD flush" ||
      incoming == "STARTED" || incoming == "STOPPED" || incoming == "PONG" ||
      incoming.startsWith("FREQ ");

  if (isOpenEvent(incoming)) {
    Serial.println("VALVE OPEN");
    lcdLog("VALVE OPEN");
  } else if (isCloseEvent(incoming)) {
    Serial.println("VALVE CLOSED");
    lcdLog("VALVE CLOSED");
  }

  // ANY valid message keeps connection alive
  if (incoming.equals(buttonPress) || incoming.equals(heartbeatMsg) ||
      incoming == "START" || incoming == "STOP" || incoming == "PING" ||
      incoming == "SCAN" || isOpenEvent(incoming) || isCloseEvent(incoming) ||
      isTelemetry) {

    lastReceiveTime = millis();

    if (currentState != CONNECTED) {
      Serial.println("Connection ESTABLISHED");
      lcdLog("Connection ESTAB");
      previousMillis = millis();
    }

    currentState = CONNECTED;
  }
}

// -------- BLINK FUNCTION --------
void handleBlink(unsigned long currentMillis, bool blinkBlue, long interval) {

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    ledState = !ledState;

    if (blinkBlue) {
      digitalWrite(blueLEDPin, ledState);
      digitalWrite(redLEDPin, LOW);
    } else {
      digitalWrite(redLEDPin, ledState);
      digitalWrite(blueLEDPin, LOW);
    }
  }
}
