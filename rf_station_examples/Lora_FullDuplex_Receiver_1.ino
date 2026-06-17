#include <LiquidCrystal_I2C.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>

// LoRa pins
const int csPin = 4;
const int resetPin = 2;
const int irqPin = 3;

// LEDs
const int redLEDPin = 5;
const int blueLEDPin = 6;

// Button
const int buttonPin = 8;
int sendButtonState;

// Messages
String buttonPress = "button pressed";
String heartbeatMsg = "hb";

byte msgCount = 0;

// LCD Screen
LiquidCrystal_I2C lcd(0x27, 20, 4);

// -------- CHANGE PER DEVICE --------
byte localAddress = 0xCC;
byte destination = 0xA1; // payload address
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

bool isOpenEvent(const String &msg) {
  return msg == "OPEN" || msg == "VALVE_ON" || msg == "VALVE_OPEN" ||
         msg == "VALVE=1";
}

bool isCloseEvent(const String &msg) {
  return msg == "CLOSE" || msg == "VALVE_OFF" || msg == "VALVE_CLOSED" ||
         msg == "VALVE=0";
}

void handleSerialCommand() {
  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0)
    return;

  if (cmd == "START" || cmd == "STOP" || cmd == "PING" || cmd == "OPEN" ||
      cmd == "CLOSE" || cmd == "SCAN") {
    sendMessage(cmd);
    LoRa.receive();
    Serial.println("Sent command: " + cmd);
    if (isOpenEvent(cmd)) {
      Serial.println("VALVE OPEN command sent");
      lcdLog("VALVE OPEN command sent");
    } else if (isCloseEvent(cmd)) {
      Serial.println("VALVE CLOSE command sent");
      lcdLog("VALVE CLOSE command sent");
    }
  } else {
    Serial.println("Unknown command. Use START/STOP/PING/OPEN/CLOSE/SCAN");
    lcdLog("Unknown command. Use START/STOP/PING/OPEN/CLOSE/SCAN");
  }
}

// ---------- LCD Screen Initalization -------
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

void setup() {
  lcd.init();
  lcd.backlight();

  // lcd.setCursor(0,0);
  // lcd.print("LCD TEST");

  // lcd.setCursor(0,1);
  // lcd.print("Line 2");

  // lcd.setCursor(0,2);
  // lcd.print("Line 3");

  // lcd.setCursor(0,3);
  // lcd.print("Line 4");

  // pinMode(buttonPin, INPUT_PULLUP);
  // pinMode(redLEDPin, OUTPUT);
  // pinMode(blueLEDPin, OUTPUT);

  Serial.begin(9600);

  // Start in IDLE
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(blueLEDPin, LOW);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa failed!");
    lcdLog("LoRa failed!");
    while (1)
      ;
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("LoRa FSM with Heartbeat Ready");
  lcdLog("LoRa FSM with Heartbeat Ready");
  Serial.println("USB commands: START/STOP/PING/OPEN/CLOSE/SCAN");
  lcdLog("USB commands: START/STOP/PING/OPEN/CLOSE/SCAN");

  // -------- LCD DISPLAY FUNCTION  --------
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.print("LoRa Starting...");
}

void loop() {

  unsigned long currentMillis = millis();
  handleSerialCommand();

  // -------- BUTTON SEND --------
  sendButtonState = digitalRead(buttonPin);

  if (sendButtonState == LOW) {
    sendMessage(buttonPress);
    delay(200);
    LoRa.receive();
  }

  // -------- HEARTBEAT SEND --------
  if (currentMillis - lastHeartbeatTime >= heartbeatInterval) {
    lastHeartbeatTime = currentMillis;

    sendMessage(heartbeatMsg);
    LoRa.receive();

    Serial.println("Heartbeat sent");
    lcdLog("Heartbeat sent");
  }

  // -------- STATE TRANSITIONS --------
  if (currentState == CONNECTED &&
      (currentMillis - lastReceiveTime > signalTimeout)) {

    Serial.println("Connection LOST");
    lcdLog("onnection LOST");

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

  Serial.println("Received: " + incoming);
  lcdLog("Received: " + incoming);

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
      incoming == "SCAN" || isOpenEvent(incoming) || isCloseEvent(incoming)) {

    lastReceiveTime = millis();

    if (currentState != CONNECTED) {
      Serial.println("Connection ESTABLISHED");
      lcdLog("Connection ESTABLISHED");
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
