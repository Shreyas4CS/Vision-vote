/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║         VISIONVOTE — Smart AVM ESP32 Firmware               ║
 * ║         by CognoSpace, Hyderabad                            ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Hardware:                                                   ║
 * ║   • ESP32 (any variant)                                      ║
 * ║   • SG90 Servo Motor (Gate)                                  ║
 * ║   • Green LED (Access granted)                               ║
 * ║   • Red LED (Access denied)                                  ║
 * ║   • Active Buzzer                                            ║
 * ║   • 0.96" I2C OLED (SSD1306, 128×64)                        ║
 * ║   • 3 Push Buttons (Vote 1, 2, 3)                            ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Wiring:                                                     ║
 * ║   Servo Signal  → GPIO 18                                    ║
 * ║   Green LED +   → GPIO 25 (via 220Ω)                         ║
 * ║   Red LED +     → GPIO 26 (via 220Ω)                         ║
 * ║   Buzzer +      → GPIO 27                                    ║
 * ║   Button 1      → GPIO 34 (INPUT_PULLUP)                     ║
 * ║   Button 2      → GPIO 35 (INPUT_PULLUP)                     ║
 * ║   Button 3      → GPIO 32 (INPUT_PULLUP)                     ║
 * ║   OLED SDA      → GPIO 21                                    ║
 * ║   OLED SCL      → GPIO 22                                    ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Libraries required (install via Arduino Library Manager):   ║
 * ║   • ESP32 BLE Arduino (built-in with ESP32 board package)    ║
 * ║   • Adafruit SSD1306                                         ║
 * ║   • Adafruit GFX Library                                     ║
 * ║   • ESP32Servo                                               ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════
#define PIN_SERVO       18
#define PIN_LED_GREEN   25
#define PIN_LED_RED     26
#define PIN_BUZZER      27
#define PIN_BTN1        34
#define PIN_BTN2        35
#define PIN_BTN3        32
#define OLED_SDA        21
#define OLED_SCL        22
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// ════════════════════════════════════
//  BLE UUIDs (must match website)
// ════════════════════════════════════
#define SERVICE_UUID     "12345678-1234-1234-1234-123456789012"
#define CHAR_UUID        "12345678-1234-1234-1234-123456789013"
#define NOTIFY_UUID      "12345678-1234-1234-1234-123456789014"

// ════════════════════════════════════
//  SERVO ANGLES
// ════════════════════════════════════
int SERVO_OPEN_ANGLE  = 90;
int SERVO_CLOSE_ANGLE = 0;

// ════════════════════════════════════
//  OBJECTS
// ════════════════════════════════════
Servo gateServo;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BLEServer*       pServer       = nullptr;
BLECharacteristic* pWriteChar  = nullptr;
BLECharacteristic* pNotifyChar = nullptr;

// ════════════════════════════════════
//  STATE
// ════════════════════════════════════
bool deviceConnected    = false;
bool oldDeviceConnected = false;
bool gateIsOpen         = false;
bool faceVerified       = false;
bool votingEnabled      = false;

unsigned long gateOpenTime  = 0;
unsigned long buzzerEndTime = 0;
bool buzzerActive           = false;

String lastCommand = "";

// Button debounce
unsigned long btn1LastPress = 0;
unsigned long btn2LastPress = 0;
unsigned long btn3LastPress = 0;
const unsigned long DEBOUNCE = 300;

// ════════════════════════════════════
//  OLED HELPERS
// ════════════════════════════════════
void oledClear() {
  oled.clearDisplay();
  oled.display();
}

void oledPrint(const char* line1, const char* line2 = "", const char* line3 = "") {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Header bar
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);
  oled.print(F("VisionVote  CognoSpace"));
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  if (strlen(line1) > 0) { oled.setCursor(2, 16); oled.print(line1); }
  if (strlen(line2) > 0) { oled.setCursor(2, 30); oled.print(line2); }
  if (strlen(line3) > 0) { oled.setCursor(2, 44); oled.print(line3); }

  // Footer dots (active indicator)
  oled.fillCircle(60, 59, 2, SSD1306_WHITE);
  oled.fillCircle(68, 59, 2, SSD1306_WHITE);

  oled.display();
}

void oledLarge(const char* msg) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);
  oled.print(F("VisionVote"));
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(4, 22);
  oled.print(msg);
  oled.display();
}

void oledWelcome(String name) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);
  oled.print(F("ACCESS GRANTED"));
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(2, 16);
  oled.print(F("Welcome,"));
  oled.setTextSize(1);
  oled.setCursor(2, 28);
  oled.print(name.substring(0, 20));
  oled.setCursor(2, 42);
  oled.print(F("Gate: OPEN (5s)"));
  // Animated border
  oled.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  oled.display();
}

// ════════════════════════════════════
//  GATE CONTROL
// ════════════════════════════════════
void openGate() {
  gateServo.write(SERVO_OPEN_ANGLE);
  gateIsOpen   = true;
  gateOpenTime = millis();
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED,   LOW);
  Serial.println(F("[GATE] Opened"));
}

void closeGate() {
  gateServo.write(SERVO_CLOSE_ANGLE);
  gateIsOpen = false;
  digitalWrite(PIN_LED_GREEN, LOW);
  Serial.println(F("[GATE] Closed"));
  oledPrint("VisionVote Ready", "Enter Voter ID", "on website");
}

// ════════════════════════════════════
//  BUZZER
// ════════════════════════════════════
void startBuzzer(unsigned long durationMs) {
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerActive  = true;
  buzzerEndTime = millis() + durationMs;
}

void stopBuzzer() {
  digitalWrite(PIN_BUZZER, LOW);
  buzzerActive = false;
}

// ════════════════════════════════════
//  ACCESS DENIED
// ════════════════════════════════════
void accessDenied() {
  gateServo.write(SERVO_CLOSE_ANGLE);
  gateIsOpen = false;
  // Blink red LED & buzzer for 5 seconds
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(500);
    digitalWrite(PIN_LED_RED,  LOW);
    digitalWrite(PIN_BUZZER,   LOW);
    delay(500);
  }
  oledPrint("ACCESS DENIED", "Invalid Voter ID", "Try again");
  Serial.println(F("[GATE] Access denied"));
}

// ════════════════════════════════════
//  FACE FAIL
// ════════════════════════════════════
void faceFail() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(300);
    digitalWrite(PIN_LED_RED,  LOW);
    digitalWrite(PIN_BUZZER,   LOW);
    delay(300);
  }
  oledPrint("FACE MISMATCH", "Not Verified", "Try again");
  faceVerified  = false;
  votingEnabled = false;
}

// ════════════════════════════════════
//  BLE COMMAND PARSER
// ════════════════════════════════════
void handleCommand(String cmd) {
  cmd.trim();
  Serial.print(F("[BLE] Command: ")); Serial.println(cmd);

  if (cmd == "GATE_OPEN") {
    openGate();

  } else if (cmd == "GATE_CLOSE") {
    closeGate();

  } else if (cmd == "ACCESS_DENIED") {
    accessDenied();

  } else if (cmd == "FACE_VERIFIED") {
    faceVerified  = true;
    votingEnabled = true;
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(200); digitalWrite(PIN_LED_GREEN, LOW);
    delay(200); digitalWrite(PIN_LED_GREEN, HIGH);
    delay(200); digitalWrite(PIN_LED_GREEN, LOW);
    oledPrint("FACE VERIFIED", "You may vote now", "Press 1, 2 or 3");
    Serial.println(F("[FACE] Verified OK"));

  } else if (cmd == "FACE_FAIL") {
    faceFail();

  } else if (cmd == "VOTE_CAST") {
    faceVerified  = false;
    votingEnabled = false;
    // Happy buzzer melody
    int freqs[] = {1000, 1200, 1500};
    for (int i = 0; i < 3; i++) {
      tone(PIN_BUZZER, freqs[i], 150);
      delay(200);
    }
    noTone(PIN_BUZZER);
    oledPrint("VOTE CAST!", "Thank you!", "");
    delay(3000);
    oledPrint("VisionVote Ready", "Enter Voter ID", "on website");

  } else if (cmd == "TEST_OPEN") {
    openGate();
    delay(2000);
    closeGate();

  } else if (cmd == "TEST_CLOSE") {
    closeGate();

  } else if (cmd.startsWith("SERVO_OPEN:")) {
    // Format: SERVO_OPEN:90;SERVO_CLOSE:0
    int sep1 = cmd.indexOf(':');
    int sep2 = cmd.indexOf(';');
    int sep3 = cmd.lastIndexOf(':');
    if (sep1 > 0 && sep2 > 0 && sep3 > 0) {
      SERVO_OPEN_ANGLE  = cmd.substring(sep1 + 1, sep2).toInt();
      SERVO_CLOSE_ANGLE = cmd.substring(sep3 + 1).toInt();
      SERVO_OPEN_ANGLE  = constrain(SERVO_OPEN_ANGLE,  0, 180);
      SERVO_CLOSE_ANGLE = constrain(SERVO_CLOSE_ANGLE, 0, 180);
      Serial.printf("[SERVO] Open=%d, Close=%d\n", SERVO_OPEN_ANGLE, SERVO_CLOSE_ANGLE);
      oledPrint("Servo Updated", ("Open:"+String(SERVO_OPEN_ANGLE)).c_str(), ("Close:"+String(SERVO_CLOSE_ANGLE)).c_str());
    }

  } else if (cmd.startsWith("OLED:")) {
    // Direct OLED text command
    String text = cmd.substring(5);
    int nl = text.indexOf('\\');
    if (nl >= 0) {
      oledPrint(text.substring(0, nl).c_str(), text.substring(nl+1).c_str());
    } else {
      oledPrint(text.c_str());
    }
  }
}

// ════════════════════════════════════
//  BLE CALLBACKS
// ════════════════════════════════════
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pSvr) override {
    deviceConnected = true;
    Serial.println(F("[BLE] Client connected"));
    oledPrint("BLE Connected", "Website linked!", "");
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(300);
    digitalWrite(PIN_LED_GREEN, LOW);
  }
  void onDisconnect(BLEServer* pSvr) override {
    deviceConnected = false;
    Serial.println(F("[BLE] Client disconnected"));
    oledPrint("BLE Disconnected", "Open website to", "reconnect");
  }
};

class WriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    if (val.length() > 0) {
      String cmd = String(val.c_str());
      handleCommand(cmd);
    }
  }
};

// ════════════════════════════════════
//  SETUP
// ════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n╔══════════════════════════════╗"));
  Serial.println(F("║  VisionVote ESP32 Booting    ║"));
  Serial.println(F("║  by CognoSpace, Hyderabad    ║"));
  Serial.println(F("╚══════════════════════════════╝"));

  // Pin modes
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);

  // Default off
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  digitalWrite(PIN_BUZZER,    LOW);

  // Servo
  gateServo.attach(PIN_SERVO, 500, 2400);
  gateServo.write(SERVO_CLOSE_ANGLE);
  delay(500);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[OLED] FAILED — check wiring"));
  } else {
    Serial.println(F("[OLED] OK"));
    // Splash screen
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(8, 5);
    oled.print(F("VISION"));
    oled.setTextSize(2);
    oled.setCursor(8, 25);
    oled.print(F("VOTE"));
    oled.setTextSize(1);
    oled.setCursor(8, 50);
    oled.print(F("by CognoSpace"));
    oled.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    oled.display();
    delay(2500);
  }

  // BLE
  BLEDevice::init("VisionVote-ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Write characteristic (website → ESP32)
  pWriteChar = pService->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pWriteChar->setCallbacks(new WriteCallbacks());

  // Notify characteristic (ESP32 → website, future use)
  pNotifyChar = pService->createCharacteristic(
    NOTIFY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pNotifyChar->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(false);
  pAdv->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  Serial.println(F("[BLE] Advertising as 'VisionVote-ESP32'"));
  oledPrint("VisionVote Ready", "Open website &", "Connect via BLE");

  // Startup beep
  tone(PIN_BUZZER, 880, 100); delay(150);
  tone(PIN_BUZZER, 1100, 150); delay(200);
  noTone(PIN_BUZZER);
}

// ════════════════════════════════════
//  LOOP
// ════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Auto-close gate after 5 seconds ──
  if (gateIsOpen && (now - gateOpenTime >= 5000)) {
    closeGate();
  }

  // ── Auto-stop buzzer ──
  if (buzzerActive && now >= buzzerEndTime) {
    stopBuzzer();
  }

  // ── BLE reconnect advertising ──
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println(F("[BLE] Restarted advertising"));
    oldDeviceConnected = false;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
  }

  // ── Push button voting (only when face is verified) ──
  if (votingEnabled) {
    if (digitalRead(PIN_BTN1) == LOW && (now - btn1LastPress > DEBOUNCE)) {
      btn1LastPress = now;
      Serial.println(F("[BTN] Vote 1"));
      notifyWebsite("BTN_VOTE:1");
      votingEnabled = false;
      faceVerified  = false;
    }
    if (digitalRead(PIN_BTN2) == LOW && (now - btn2LastPress > DEBOUNCE)) {
      btn2LastPress = now;
      Serial.println(F("[BTN] Vote 2"));
      notifyWebsite("BTN_VOTE:2");
      votingEnabled = false;
      faceVerified  = false;
    }
    if (digitalRead(PIN_BTN3) == LOW && (now - btn3LastPress > DEBOUNCE)) {
      btn3LastPress = now;
      Serial.println(F("[BTN] Vote 3"));
      notifyWebsite("BTN_VOTE:3");
      votingEnabled = false;
      faceVerified  = false;
    }
  }

  delay(20);
}

// ════════════════════════════════════
//  NOTIFY WEBSITE VIA BLE
// ════════════════════════════════════
void notifyWebsite(const char* msg) {
  if (pNotifyChar && deviceConnected) {
    pNotifyChar->setValue((uint8_t*)msg, strlen(msg));
    pNotifyChar->notify();
    Serial.printf("[BLE] Notified: %s\n", msg);
  }
}

/*
 * ════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — Educational Notes for Students
 * ════════════════════════════════════════════════════════════════
 *
 *  1. STARTUP:
 *     - ESP32 boots, displays splash on OLED, plays beep
 *     - BLE starts advertising as "VisionVote-ESP32"
 *
 *  2. WEBSITE → ESP32 (Write Characteristic):
 *     - GATE_OPEN       → Servo opens to SERVO_OPEN_ANGLE, Green LED on, 5s timer
 *     - GATE_CLOSE      → Servo closes to SERVO_CLOSE_ANGLE, LEDs off
 *     - ACCESS_DENIED   → Red LED + Buzzer blink 5 times
 *     - FACE_VERIFIED   → Green LED flash, enable push buttons
 *     - FACE_FAIL       → Red LED + Buzzer blink 3 times
 *     - VOTE_CAST       → Happy melody, OLED "Thank You!"
 *     - SERVO_OPEN:x;SERVO_CLOSE:y → Update servo angles
 *     - TEST_OPEN       → Open gate for 2 seconds then close
 *     - TEST_CLOSE      → Close gate immediately
 *
 *  3. ESP32 → WEBSITE (Notify Characteristic):
 *     - BTN_VOTE:1/2/3 → When push button pressed (face must be verified)
 *
 *  4. PUSH BUTTONS:
 *     - Only active after FACE_VERIFIED command received
 *     - Sends BTN_VOTE:1/2/3 to website via BLE notify
 *     - Website casts the vote for the corresponding candidate
 *     - Buttons deactivate after one vote
 *
 *  5. OLED DISPLAY (128×64 I2C at 0x3C):
 *     - Shows system status, voter welcome, face status
 *     - Header bar always shows "VisionVote" branding
 *
 *  6. SERVO MOTOR:
 *     - GPIO 18, using PWM (500–2400μs pulse range)
 *     - Default: Open=90°, Close=0°
 *     - Configurable from Admin panel on website
 *
 * ════════════════════════════════════════════════════════════════
 */
