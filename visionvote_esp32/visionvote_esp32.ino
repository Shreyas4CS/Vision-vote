/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║              VISIONVOTE — ESP32 FIRMWARE                    ║
 * ║              by CognoSpace                                  ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Hardware:                                                   ║
 * ║   • ESP32                                                     ║
 * ║   • SG90 Servo Motor                                          ║
 * ║   • Green LED                                                 ║
 * ║   • Red LED                                                   ║
 * ║   • Active Buzzer                                             ║
 * ║   • 0.96" I2C OLED SSD1306 128x64                            ║
 * ║   • 3 Push Buttons                                            ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  PIN CONNECTIONS                                              ║
 * ║   Servo       → GPIO 25                                      ║
 * ║   Green LED   → GPIO 18                                      ║
 * ║   Red LED     → GPIO 19                                      ║
 * ║   Buzzer      → GPIO 4                                       ║
 * ║   Button 1    → GPIO 23                                      ║
 * ║   Button 2    → GPIO 12                                      ║
 * ║   Button 3    → GPIO 15                                      ║
 * ║   OLED SDA    → GPIO 21                                      ║
 * ║   OLED SCL    → GPIO 22                                      ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

// ================================================================
// LIBRARIES
// ================================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
// PIN DEFINITIONS
// ================================================================

#define PIN_SERVO       25
#define PIN_LED_GREEN   18
#define PIN_LED_RED     19
#define PIN_BUZZER      4

#define PIN_BTN1        23
#define PIN_BTN2        12
#define PIN_BTN3        15

#define OLED_SDA        21
#define OLED_SCL        22

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// ================================================================
// BLE UUIDs
// These must match the website
// ================================================================

#define SERVICE_UUID    "12345678-1234-1234-1234-123456789012"
#define CHAR_UUID       "12345678-1234-1234-1234-123456789013"
#define NOTIFY_UUID     "12345678-1234-1234-1234-123456789014"

// ================================================================
// SERVO SETTINGS
// ================================================================

int SERVO_OPEN_ANGLE  = 90;
int SERVO_CLOSE_ANGLE = 0;

// ================================================================
// OBJECTS
// ================================================================

Servo gateServo;

Adafruit_SSD1306 oled(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

BLEServer* pServer = nullptr;
BLECharacteristic* pWriteChar = nullptr;
BLECharacteristic* pNotifyChar = nullptr;

// ================================================================
// SYSTEM STATE
// ================================================================

bool deviceConnected = false;
bool oldDeviceConnected = false;

bool gateIsOpen = false;
bool faceVerified = false;
bool votingEnabled = false;

unsigned long gateOpenTime = 0;

unsigned long buzzerEndTime = 0;
bool buzzerActive = false;

// ================================================================
// BUTTON DEBOUNCE
// ================================================================

unsigned long btn1LastPress = 0;
unsigned long btn2LastPress = 0;
unsigned long btn3LastPress = 0;

const unsigned long DEBOUNCE = 300;

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================

void handleCommand(String cmd);

void openGate();
void closeGate();

void accessDenied();
void faceFail();

void startBuzzer(unsigned long durationMs);
void stopBuzzer();

void notifyWebsite(const char* msg);

void oledClear();
void oledPrint(
  const char* line1,
  const char* line2 = "",
  const char* line3 = ""
);

void oledLarge(const char* msg);
void oledWelcome(String name);

// ================================================================
// OLED HELPERS
// ================================================================

void oledClear() {
  oled.clearDisplay();
  oled.display();
}

// ---------------------------------------------------------------
// Standard OLED screen
// ---------------------------------------------------------------

void oledPrint(
  const char* line1,
  const char* line2,
  const char* line3
) {

  oled.clearDisplay();

  // Header
  oled.fillRect(
    0,
    0,
    SCREEN_WIDTH,
    12,
    SSD1306_WHITE
  );

  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);

  oled.print(F("VisionVote"));

  // Body
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  if (line1 != nullptr && strlen(line1) > 0) {
    oled.setCursor(2, 16);
    oled.print(line1);
  }

  if (line2 != nullptr && strlen(line2) > 0) {
    oled.setCursor(2, 30);
    oled.print(line2);
  }

  if (line3 != nullptr && strlen(line3) > 0) {
    oled.setCursor(2, 44);
    oled.print(line3);
  }

  // Footer indicators
  oled.fillCircle(60, 59, 2, SSD1306_WHITE);
  oled.fillCircle(68, 59, 2, SSD1306_WHITE);

  oled.display();
}

// ---------------------------------------------------------------
// Large OLED message
// ---------------------------------------------------------------

void oledLarge(const char* msg) {

  oled.clearDisplay();

  oled.setTextColor(SSD1306_WHITE);

  // Header
  oled.fillRect(
    0,
    0,
    SCREEN_WIDTH,
    12,
    SSD1306_WHITE
  );

  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);

  oled.print(F("VisionVote"));

  // Message
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(4, 22);

  oled.print(msg);

  oled.display();
}

// ---------------------------------------------------------------
// Welcome screen
// ---------------------------------------------------------------

void oledWelcome(String name) {

  oled.clearDisplay();

  // Header
  oled.fillRect(
    0,
    0,
    SCREEN_WIDTH,
    12,
    SSD1306_WHITE
  );

  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);

  oled.print(F("ACCESS GRANTED"));

  // Body
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(2, 16);
  oled.print(F("Welcome,"));

  oled.setCursor(2, 28);
  oled.print(name.substring(0, 20));

  oled.setCursor(2, 42);
  oled.print(F("Gate: OPEN"));

  // Border
  oled.drawRect(
    0,
    0,
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    SSD1306_WHITE
  );

  oled.display();
}

// ================================================================
// GATE CONTROL
// ================================================================

void openGate() {

  gateServo.write(SERVO_OPEN_ANGLE);

  gateIsOpen = true;
  gateOpenTime = millis();

  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED, LOW);

  Serial.println(F("[GATE] Opened"));

  oledPrint(
    "ACCESS GRANTED",
    "Gate OPEN",
    "Please proceed"
  );
}

// ---------------------------------------------------------------

void closeGate() {

  gateServo.write(SERVO_CLOSE_ANGLE);

  gateIsOpen = false;

  digitalWrite(PIN_LED_GREEN, LOW);

  Serial.println(F("[GATE] Closed"));

  oledPrint(
    "VisionVote Ready",
    "Enter Voter ID",
    "on website"
  );
}

// ================================================================
// BUZZER
// ================================================================

void startBuzzer(unsigned long durationMs) {

  digitalWrite(PIN_BUZZER, HIGH);

  buzzerActive = true;

  buzzerEndTime = millis() + durationMs;
}

// ---------------------------------------------------------------

void stopBuzzer() {

  digitalWrite(PIN_BUZZER, LOW);

  buzzerActive = false;
}

// ================================================================
// ACCESS DENIED
// ================================================================

void accessDenied() {

  // Make sure gate is closed
  gateServo.write(SERVO_CLOSE_ANGLE);
  gateIsOpen = false;

  // Red LED + buzzer
  for (int i = 0; i < 5; i++) {

    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);

    delay(500);

    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    delay(500);
  }

  oledPrint(
    "ACCESS DENIED",
    "Invalid Voter ID",
    "Try again"
  );

  Serial.println(F("[GATE] Access denied"));
}

// ================================================================
// FACE VERIFICATION FAILURE
// ================================================================

void faceFail() {

  for (int i = 0; i < 3; i++) {

    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);

    delay(300);

    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    delay(300);
  }

  faceVerified = false;
  votingEnabled = false;

  oledPrint(
    "FACE MISMATCH",
    "Not Verified",
    "Try again"
  );

  Serial.println(F("[FACE] Verification failed"));
}

// ================================================================
// BLE COMMAND HANDLER
// ================================================================

void handleCommand(String cmd) {

  cmd.trim();

  Serial.print(F("[BLE] Command: "));
  Serial.println(cmd);

  // -------------------------------------------------------------
  // OPEN GATE
  // -------------------------------------------------------------

  if (cmd == "GATE_OPEN") {

    openGate();
  }

  // -------------------------------------------------------------
  // CLOSE GATE
  // -------------------------------------------------------------

  else if (cmd == "GATE_CLOSE") {

    closeGate();
  }

  // -------------------------------------------------------------
  // ACCESS DENIED
  // -------------------------------------------------------------

  else if (cmd == "ACCESS_DENIED") {

    accessDenied();
  }

  // -------------------------------------------------------------
  // FACE VERIFIED
  // -------------------------------------------------------------

  else if (cmd == "FACE_VERIFIED") {

    faceVerified = true;
    votingEnabled = true;

    // Green LED confirmation
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(200);

    digitalWrite(PIN_LED_GREEN, LOW);
    delay(200);

    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(200);

    digitalWrite(PIN_LED_GREEN, LOW);

    oledPrint(
      "FACE VERIFIED",
      "You may vote now",
      "Press 1, 2 or 3"
    );

    Serial.println(F("[FACE] Verified OK"));
  }

  // -------------------------------------------------------------
  // FACE FAIL
  // -------------------------------------------------------------

  else if (cmd == "FACE_FAIL") {

    faceFail();
  }

  // -------------------------------------------------------------
  // VOTE CAST
  // -------------------------------------------------------------

  else if (cmd == "VOTE_CAST") {

    faceVerified = false;
    votingEnabled = false;

    // Happy melody
    int freqs[] = {
      1000,
      1200,
      1500
    };

    for (int i = 0; i < 3; i++) {

      tone(
        PIN_BUZZER,
        freqs[i],
        150
      );

      delay(200);
    }

    noTone(PIN_BUZZER);

    oledPrint(
      "VOTE CAST!",
      "Thank you!",
      ""
    );

    Serial.println(F("[VOTE] Vote cast"));

    delay(3000);

    oledPrint(
      "VisionVote Ready",
      "Enter Voter ID",
      "on website"
    );
  }

  // -------------------------------------------------------------
  // TEST OPEN
  // -------------------------------------------------------------

  else if (cmd == "TEST_OPEN") {

    Serial.println(F("[TEST] Opening gate"));

    openGate();

    delay(2000);

    closeGate();
  }

  // -------------------------------------------------------------
  // TEST CLOSE
  // -------------------------------------------------------------

  else if (cmd == "TEST_CLOSE") {

    Serial.println(F("[TEST] Closing gate"));

    closeGate();
  }

  // -------------------------------------------------------------
  // SERVO SETTINGS
  //
  // Example:
  // SERVO_OPEN:90;SERVO_CLOSE:0
  // -------------------------------------------------------------

  else if (cmd.startsWith("SERVO_OPEN:")) {

    int sep1 = cmd.indexOf(':');
    int sep2 = cmd.indexOf(';');
    int sep3 = cmd.lastIndexOf(':');

    if (
      sep1 > 0 &&
      sep2 > sep1 &&
      sep3 > sep2
    ) {

      String openValue =
        cmd.substring(sep1 + 1, sep2);

      String closeValue =
        cmd.substring(sep3 + 1);

      SERVO_OPEN_ANGLE = openValue.toInt();
      SERVO_CLOSE_ANGLE = closeValue.toInt();

      SERVO_OPEN_ANGLE =
        constrain(
          SERVO_OPEN_ANGLE,
          0,
          180
        );

      SERVO_CLOSE_ANGLE =
        constrain(
          SERVO_CLOSE_ANGLE,
          0,
          180
        );

      Serial.print(F("[SERVO] Open = "));
      Serial.println(SERVO_OPEN_ANGLE);

      Serial.print(F("[SERVO] Close = "));
      Serial.println(SERVO_CLOSE_ANGLE);

      String openText =
        "Open: " + String(SERVO_OPEN_ANGLE);

      String closeText =
        "Close: " + String(SERVO_CLOSE_ANGLE);

      oledPrint(
        "Servo Updated",
        openText.c_str(),
        closeText.c_str()
      );
    }

    else {

      Serial.println(
        F("[SERVO] Invalid servo command")
      );
    }
  }

  // -------------------------------------------------------------
  // OLED COMMAND
  //
  // Example:
  // OLED:Hello
  //
  // Multiple lines can be separated using |
  // Example:
  // OLED:Line1|Line2|Line3
  // -------------------------------------------------------------

  else if (cmd.startsWith("OLED:")) {

    String text = cmd.substring(5);

    int separator1 = text.indexOf('|');

    if (separator1 >= 0) {

      String line1 =
        text.substring(0, separator1);

      String remaining =
        text.substring(separator1 + 1);

      int separator2 =
        remaining.indexOf('|');

      if (separator2 >= 0) {

        String line2 =
          remaining.substring(0, separator2);

        String line3 =
          remaining.substring(separator2 + 1);

        oledPrint(
          line1.c_str(),
          line2.c_str(),
          line3.c_str()
        );
      }

      else {

        oledPrint(
          line1.c_str(),
          remaining.c_str(),
          ""
        );
      }
    }

    else {

      oledPrint(
        text.c_str(),
        "",
        ""
      );
    }
  }

  // -------------------------------------------------------------
  // UNKNOWN COMMAND
  // -------------------------------------------------------------

  else {

    Serial.print(F("[BLE] Unknown command: "));
    Serial.println(cmd);
  }
}

// ================================================================
// BLE SERVER CALLBACKS
// ================================================================

class ServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer* pSvr) override {

    deviceConnected = true;

    Serial.println(
      F("[BLE] Client connected")
    );

    oledPrint(
      "BLE Connected",
      "Website linked!",
      ""
    );

    digitalWrite(
      PIN_LED_GREEN,
      HIGH
    );

    delay(300);

    digitalWrite(
      PIN_LED_GREEN,
      LOW
    );
  }

  void onDisconnect(BLEServer* pSvr) override {

    deviceConnected = false;

    Serial.println(
      F("[BLE] Client disconnected")
    );

    oledPrint(
      "BLE Disconnected",
      "Open website to",
      "reconnect"
    );
  }
};

// ================================================================
// BLE WRITE CALLBACK
// IMPORTANT:
// getValue() returns Arduino String in your BLE library.
// ================================================================

class WriteCallbacks : public BLECharacteristicCallbacks {

  void onWrite(
    BLECharacteristic* pChar
  ) override {

    // FIX:
    // Do NOT use std::string here.
    String val = pChar->getValue();

    if (val.length() > 0) {

      Serial.print(F("[BLE RX] "));
      Serial.println(val);

      handleCommand(val);
    }
  }
};

// ================================================================
// SETUP
// ================================================================

void setup() {

  // -------------------------------------------------------------
  // SERIAL
  // -------------------------------------------------------------

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println(
    F("================================")
  );

  Serial.println(
    F("      VISIONVOTE ESP32")
  );

  Serial.println(
    F("      CognoSpace")
  );

  Serial.println(
    F("================================")
  );

  // -------------------------------------------------------------
  // PIN MODES
  // -------------------------------------------------------------

  pinMode(
    PIN_LED_GREEN,
    OUTPUT
  );

  pinMode(
    PIN_LED_RED,
    OUTPUT
  );

  pinMode(
    PIN_BUZZER,
    OUTPUT
  );

  pinMode(
    PIN_BTN1,
    INPUT_PULLUP
  );

  pinMode(
    PIN_BTN2,
    INPUT_PULLUP
  );

  pinMode(
    PIN_BTN3,
    INPUT_PULLUP
  );

  // -------------------------------------------------------------
  // DEFAULT OUTPUT STATES
  // -------------------------------------------------------------

  digitalWrite(
    PIN_LED_GREEN,
    LOW
  );

  digitalWrite(
    PIN_LED_RED,
    LOW
  );

  digitalWrite(
    PIN_BUZZER,
    LOW
  );

  // -------------------------------------------------------------
  // SERVO
  // -------------------------------------------------------------

  Serial.println(
    F("[SERVO] Initializing...")
  );

  gateServo.setPeriodHertz(50);

  gateServo.attach(
    PIN_SERVO,
    500,
    2400
  );

  gateServo.write(
    SERVO_CLOSE_ANGLE
  );

  delay(500);

  Serial.println(
    F("[SERVO] Ready")
  );

  // -------------------------------------------------------------
  // OLED
  // -------------------------------------------------------------

  Serial.println(
    F("[OLED] Initializing...")
  );

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );

  if (
    !oled.begin(
      SSD1306_SWITCHCAPVCC,
      0x3C
    )
  ) {

    Serial.println(
      F("[OLED] FAILED")
    );

  }

  else {

    Serial.println(
      F("[OLED] OK")
    );

    // Splash screen
    oled.clearDisplay();

    oled.setTextColor(
      SSD1306_WHITE
    );

    oled.setTextSize(2);

    oled.setCursor(8, 5);
    oled.print(F("VISION"));

    oled.setCursor(8, 25);
    oled.print(F("VOTE"));

    oled.setTextSize(1);

    oled.setCursor(8, 50);
    oled.print(F("by CognoSpace"));

    oled.drawRect(
      0,
      0,
      SCREEN_WIDTH,
      SCREEN_HEIGHT,
      SSD1306_WHITE
    );

    oled.display();

    delay(2500);
  }

  // -------------------------------------------------------------
  // BLE
  // -------------------------------------------------------------

  Serial.println(
    F("[BLE] Starting...")
  );

  BLEDevice::init(
    "VisionVote-ESP32"
  );

  pServer =
    BLEDevice::createServer();

  pServer->setCallbacks(
    new ServerCallbacks()
  );

  // Create BLE service
  BLEService* pService =
    pServer->createService(
      SERVICE_UUID
    );

  // -------------------------------------------------------------
  // WRITE CHARACTERISTIC
  // Website → ESP32
  // -------------------------------------------------------------

  pWriteChar =
    pService->createCharacteristic(
      CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );

  pWriteChar->setCallbacks(
    new WriteCallbacks()
  );

  // -------------------------------------------------------------
  // NOTIFY CHARACTERISTIC
  // ESP32 → Website
  // -------------------------------------------------------------

  pNotifyChar =
    pService->createCharacteristic(
      NOTIFY_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
    );

  pNotifyChar->addDescriptor(
    new BLE2902()
  );

  // -------------------------------------------------------------
  // START BLE SERVICE
  // -------------------------------------------------------------

  pService->start();

  BLEAdvertising* pAdvertising =
    BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(
    SERVICE_UUID
  );

  pAdvertising->setScanResponse(false);

  pAdvertising->setMinPreferred(0x00);

  BLEDevice::startAdvertising();

  Serial.println(
    F("[BLE] Advertising")
  );

  Serial.println(
    F("[BLE] Device: VisionVote-ESP32")
  );

  // -------------------------------------------------------------
  // READY SCREEN
  // -------------------------------------------------------------

  oledPrint(
    "VisionVote Ready",
    "Open website &",
    "Connect via BLE"
  );

  // -------------------------------------------------------------
  // STARTUP SOUND
  // -------------------------------------------------------------

  tone(
    PIN_BUZZER,
    880,
    100
  );

  delay(150);

  tone(
    PIN_BUZZER,
    1100,
    150
  );

  delay(200);

  noTone(PIN_BUZZER);

  Serial.println();
  Serial.println(
    F("[SYSTEM] Ready!")
  );
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {

  unsigned long now = millis();

  // -------------------------------------------------------------
  // AUTO CLOSE GATE AFTER 5 SECONDS
  // -------------------------------------------------------------

  if (
    gateIsOpen &&
    (now - gateOpenTime >= 5000)
  ) {

    closeGate();
  }

  // -------------------------------------------------------------
  // AUTO STOP BUZZER
  // -------------------------------------------------------------

  if (
    buzzerActive &&
    (now >= buzzerEndTime)
  ) {

    stopBuzzer();
  }

  // -------------------------------------------------------------
  // BLE RECONNECTION
  // -------------------------------------------------------------

  if (
    !deviceConnected &&
    oldDeviceConnected
  ) {

    delay(500);

    pServer->startAdvertising();

    Serial.println(
      F("[BLE] Restarted advertising")
    );

    oldDeviceConnected = false;
  }

  if (
    deviceConnected &&
    !oldDeviceConnected
  ) {

    oldDeviceConnected = true;
  }

  // -------------------------------------------------------------
  // PUSH BUTTON VOTING
  // Only works after FACE_VERIFIED
  // -------------------------------------------------------------

  if (votingEnabled) {

    // -----------------------------------------------------------
    // BUTTON 1
    // -----------------------------------------------------------

    if (
      digitalRead(PIN_BTN1) == LOW &&
      (now - btn1LastPress > DEBOUNCE)
    ) {

      btn1LastPress = now;

      Serial.println(
        F("[BTN] Vote 1")
      );

      notifyWebsite(
        "BTN_VOTE:1"
      );

      votingEnabled = false;
      faceVerified = false;

      oledPrint(
        "VOTE 1 SELECTED",
        "Processing...",
        ""
      );
    }

    // -----------------------------------------------------------
    // BUTTON 2
    // -----------------------------------------------------------

    else if (
      digitalRead(PIN_BTN2) == LOW &&
      (now - btn2LastPress > DEBOUNCE)
    ) {

      btn2LastPress = now;

      Serial.println(
        F("[BTN] Vote 2")
      );

      notifyWebsite(
        "BTN_VOTE:2"
      );

      votingEnabled = false;
      faceVerified = false;

      oledPrint(
        "VOTE 2 SELECTED",
        "Processing...",
        ""
      );
    }

    // -----------------------------------------------------------
    // BUTTON 3
    // -----------------------------------------------------------

    else if (
      digitalRead(PIN_BTN3) == LOW &&
      (now - btn3LastPress > DEBOUNCE)
    ) {

      btn3LastPress = now;

      Serial.println(
        F("[BTN] Vote 3")
      );

      notifyWebsite(
        "BTN_VOTE:3"
      );

      votingEnabled = false;
      faceVerified = false;

      oledPrint(
        "VOTE 3 SELECTED",
        "Processing...",
        ""
      );
    }
  }

  // Small delay
  delay(20);
}

// ================================================================
// SEND MESSAGE TO WEBSITE
// ESP32 → Website via BLE Notify
// ================================================================

void notifyWebsite(
  const char* msg
) {

  if (
    pNotifyChar != nullptr &&
    deviceConnected
  ) {

    pNotifyChar->setValue(
      (uint8_t*)msg,
      strlen(msg)
    );

    pNotifyChar->notify();

    Serial.print(
      F("[BLE TX] ")
    );

    Serial.println(msg);
  }
}