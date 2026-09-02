/*
 * ============================================================
 *  VisionVote — ESP32 Firmware  v2.0
 *  by CognoSpace
 * ============================================================
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Pin Definitions ───────────────────────────────────────
#define PIN_SERVO       25
#define PIN_LED_GREEN   18
#define PIN_LED_RED     19
#define PIN_BUZZER       4

#define PIN_BTN1        23
#define PIN_BTN2        12
#define PIN_BTN3        15

#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_WIDTH     128
#define OLED_HEIGHT     64
#define OLED_ADDR     0x3C

// ── BLE UUIDs ─────────────────────────────────────────────
#define SERVICE_UUID  "12345678-1234-1234-1234-123456789012"
#define CHAR_UUID     "12345678-1234-1234-1234-123456789013"

// ── Servo Angle Defaults ──────────────────────────────────
int servoOpenAngle  = 90;
int servoCloseAngle =  0;

// ── Timing Constants ──────────────────────────────────────
#define DEBOUNCE_MS      50
#define LONG_PRESS_MS   700
#define GATE_OPEN_MS   5000
#define LED_PULSE_MS    200

// ── State Machine ─────────────────────────────────────────
enum VoteState {
  STATE_IDLE,
  STATE_GATE_OPEN,
  STATE_FACE_WAIT,
  STATE_VOTING,
  STATE_VOTED
};
VoteState voteState = STATE_IDLE;

// ── Objects ───────────────────────────────────────────────
Servo gateServo;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

BLEServer*         pServer      = nullptr;
BLECharacteristic* pChar        = nullptr;
bool               bleConnected = false;

// ── Button State ──────────────────────────────────────────
struct Button {
  uint8_t  pin;
  bool     lastRaw;
  bool     state;
  uint32_t pressTime;
  bool     longFired;
};
Button btns[3] = {
  {PIN_BTN1, HIGH, HIGH, 0, false},
  {PIN_BTN2, HIGH, HIGH, 0, false},
  {PIN_BTN3, HIGH, HIGH, 0, false}
};

uint32_t gateOpenedAt = 0;
bool     gateIsOpen   = false;

// ── FORWARD DECLARATIONS (fixes "not declared in this scope") ──
void showOLED(const char* line1, const char* line2 = "", const char* line3 = "");
void bleSend(const String& msg);
void handleWebsiteCommand(const String& cmd);
void gateOpen();
void gateClose();
void setGreenLED(bool on);
void setRedLED(bool on);
void allLEDsOff();
void buzz(int freq_ms, int count, int gap_ms = 80);
void buzzSuccess();
void buzzWelcome();
void buzzDenied();
void buzzVoted();
void buzzFaceFail();

// ── BLE Callbacks ─────────────────────────────────────────
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    showOLED("BLE Connected", "Website linked");
    Serial.println("[BLE] Client connected");
    bleSend("ESP32_READY");
  }
  void onDisconnect(BLEServer* srv) override {
    bleConnected = false;
    showOLED("BLE Disconnected", "Waiting...");
    Serial.println("[BLE] Client disconnected — restarting advertising");
    delay(500);
    srv->startAdvertising();
  }
};

class CharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String val = c->getValue().c_str();
    val.trim();
    Serial.print("[BLE RX] "); Serial.println(val);
    handleWebsiteCommand(val);
  }
};

// ── OLED ──────────────────────────────────────────────────
void showOLED(const char* line1, const char* line2, const char* line3) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.println(line1);
  if (strlen(line2)) { oled.setCursor(0, 20); oled.println(line2); }
  if (strlen(line3)) { oled.setCursor(0, 40); oled.println(line3); }
  oled.display();
}

// ── BLE Send ──────────────────────────────────────────────
void bleSend(const String& msg) {
  if (!bleConnected || !pChar) return;
  pChar->setValue(msg.c_str());
  pChar->notify();
  Serial.print("[BLE TX] "); Serial.println(msg);
}

// ── Servo ─────────────────────────────────────────────────
void gateOpen() {
  gateServo.write(servoOpenAngle);
  gateIsOpen   = true;
  gateOpenedAt = millis();
  Serial.println("[SERVO] Gate OPEN");
}
void gateClose() {
  gateServo.write(servoCloseAngle);
  gateIsOpen = false;
  Serial.println("[SERVO] Gate CLOSED");
}

// ── LEDs & Buzzer ─────────────────────────────────────────
void setGreenLED(bool on) { digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); }
void setRedLED  (bool on) { digitalWrite(PIN_LED_RED,   on ? HIGH : LOW); }
void allLEDsOff()         { setGreenLED(false); setRedLED(false); }

void buzz(int freq_ms, int count, int gap_ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_BUZZER, HIGH); delay(freq_ms);
    digitalWrite(PIN_BUZZER, LOW);  delay(gap_ms);
  }
}
void buzzSuccess()  { buzz(120, 3,  60); }
void buzzWelcome()  { buzz(200, 2, 100); }
void buzzDenied()   { buzz(400, 1,   0); }
void buzzVoted()    { buzz(80,  5,  40); }
void buzzFaceFail() { buzz(300, 2, 200); }

// ── Handle Commands from Website ──────────────────────────
void handleWebsiteCommand(const String& cmd) {

  if (cmd == "GATE_OPEN") {
    voteState = STATE_GATE_OPEN;
    gateOpen();
    setGreenLED(true);
    setRedLED(false);
    showOLED("Gate Open", "Welcome!", "Proceed inside");
    buzzWelcome();
    return;
  }

  if (cmd == "GATE_CLOSE") {
    gateClose();
    allLEDsOff();
    showOLED("VisionVote", "Ready", "Enter Voter ID");
    return;
  }

  if (cmd == "ACCESS_DENIED") {
    voteState = STATE_IDLE;
    gateClose();
    setRedLED(true);
    setGreenLED(false);
    showOLED("ACCESS DENIED", "Invalid ID or", "Already Voted");
    buzzDenied();
    delay(3000);
    setRedLED(false);
    showOLED("VisionVote", "Ready", "Enter Voter ID");
    return;
  }

  if (cmd == "FACE_VERIFIED") {
    voteState = STATE_VOTING;
    setGreenLED(true);
    setRedLED(false);
    showOLED("Face Verified!", "Press BTN 1/2/3", "to VOTE");
    buzzSuccess();
    return;
  }

  if (cmd == "FACE_FAIL") {
    voteState = STATE_FACE_WAIT;
    setRedLED(true);
    setGreenLED(false);
    showOLED("Face Mismatch!", "Please retry", "face verify");
    buzzFaceFail();
    delay(2000);
    setRedLED(false);
    showOLED("Retry Face", "Press cam button", "on website");
    return;
  }

  if (cmd == "VOTE_CAST") {
    voteState = STATE_VOTED;
    allLEDsOff();
    gateClose();
    showOLED("Vote Recorded!", "Thank You!", ":)");
    buzzVoted();
    for (int i = 0; i < 4; i++) {
      setGreenLED(true);  delay(150);
      setGreenLED(false); delay(150);
    }
    delay(2000);
    voteState = STATE_IDLE;
    showOLED("VisionVote", "Ready", "Enter Voter ID");
    return;
  }

  if (cmd == "TEST_OPEN")  { gateOpen();  showOLED("TEST", "Gate OPEN");  return; }
  if (cmd == "TEST_CLOSE") { gateClose(); showOLED("TEST", "Gate CLOSE"); return; }

  if (cmd.startsWith("SERVO_OPEN:")) {
    int semiIdx    = cmd.indexOf(';');
    String openPart  = cmd.substring(11, semiIdx);
    int closeStart   = cmd.indexOf("SERVO_CLOSE:") + 12;
    String closePart = cmd.substring(closeStart);
    servoOpenAngle  = constrain(openPart.toInt(),  0, 180);
    servoCloseAngle = constrain(closePart.toInt(), 0, 180);
    Serial.printf("[SERVO CONFIG] Open=%d  Close=%d\n", servoOpenAngle, servoCloseAngle);
    showOLED("Servo Updated",
             ("Open:" + String(servoOpenAngle)).c_str(),
             ("Close:" + String(servoCloseAngle)).c_str());
    gateServo.write(servoCloseAngle);
    return;
  }
}

// ── Button Handling ───────────────────────────────────────
void handleButtons() {
  uint32_t now = millis();

  for (int i = 0; i < 3; i++) {
    bool raw = (digitalRead(btns[i].pin) == LOW);

    if (raw != btns[i].lastRaw) {
      btns[i].lastRaw = raw;
      delay(DEBOUNCE_MS);
      raw = (digitalRead(btns[i].pin) == LOW);
    }

    bool wasPressed = btns[i].state;

    // Rising edge
    if (raw && !wasPressed) {
      btns[i].state     = true;
      btns[i].pressTime = now;
      btns[i].longFired = false;
    }

    // Long press check
    if (raw && wasPressed && !btns[i].longFired) {
      if ((now - btns[i].pressTime) >= LONG_PRESS_MS) {
        btns[i].longFired = true;
        if (i == 0) {
          bleSend("BTN_PREV");
          showOLED("Slide", "< Previous", "");
          buzz(60, 1, 0);
        } else if (i == 2) {
          bleSend("BTN_NEXT");
          showOLED("Slide", "Next >", "");
          buzz(60, 1, 0);
        }
      }
    }

    // Falling edge
    if (!raw && wasPressed) {
      btns[i].state = false;
      uint32_t held = now - btns[i].pressTime;

      if (!btns[i].longFired && held < LONG_PRESS_MS) {
        if (voteState == STATE_VOTING) {
          if (i == 0) {
            bleSend("BTN1_VOTE");
            showOLED("Button 1", "Candidate 1", "Sending vote...");
            buzz(80, 2, 60);
          } else if (i == 1) {
            bleSend("BTN2_VOTE");
            showOLED("Button 2", "Candidate 2", "Sending vote...");
            buzz(80, 2, 60);
          } else if (i == 2) {
            bleSend("BTN3_VOTE");
            showOLED("Button 3", "Candidate 3", "Sending vote...");
            buzz(80, 2, 60);
          }
          voteState = STATE_VOTED;
        } else {
          if (i == 1) {
            showOLED("Not Ready", "Complete face", "verify first");
            buzzDenied();
          }
        }
      }
    }

    btns[i].lastRaw = raw;
  }
}

// ── Auto-close Gate ───────────────────────────────────────
void handleGateTimeout() {
  if (gateIsOpen && (millis() - gateOpenedAt >= GATE_OPEN_MS)) {
    gateClose();
    allLEDsOff();
    voteState = STATE_FACE_WAIT;
    showOLED("Gate Closed", "Waiting for", "face verify");
  }
}

// ── SETUP ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[VisionVote] Booting...");

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  digitalWrite(PIN_BUZZER,    LOW);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, 500, 2400);
  gateServo.write(servoCloseAngle);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] ERROR: not found");
  } else {
    showOLED("VisionVote v2.0", "by CognoSpace", "Starting BLE...");
  }

  BLEDevice::init("VisionVote-Coggy");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_READ  |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pChar->addDescriptor(new BLE2902());
  pChar->setCallbacks(new CharCB());
  pChar->setValue("VisionVote_Ready");

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as 'VisionVote-Coggy'");
  showOLED("VisionVote", "BLE Ready", "Waiting...");
  buzz(100, 2, 80);
}

// ── LOOP ──────────────────────────────────────────────────
void loop() {
  handleButtons();
  handleGateTimeout();
  delay(10);
}