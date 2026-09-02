/*
 * ============================================================
 *  VisionVote — ESP32 Firmware  v2.0
 *  by CognoSpace
 * ============================================================
 *  Hardware Connections
 *  ──────────────────────────────────────────────────────────
 *  Servo         → GPIO 25  (PWM)
 *  LED Green     → GPIO 18  (HIGH = ON)
 *  LED Red       → GPIO 19  (HIGH = ON)
 *  Buzzer        → GPIO  4  (active buzzer, HIGH = ON)
 *  Button 1      → GPIO 23  (Vote Candidate 1 / Prev Slide)
 *  Button 2      → GPIO 12  (Vote Candidate 2 / Confirm)
 *  Button 3      → GPIO 15  (Vote Candidate 3 / Next Slide)
 *  OLED SDA      → GPIO 21
 *  OLED SCL      → GPIO 22
 *
 *  BLE Service UUID  : 12345678-1234-1234-1234-123456789012
 *  BLE Char   UUID  : 12345678-1234-1234-1234-123456789013
 *
 *  Commands FROM website → ESP32 (received via BLE):
 *    GATE_OPEN       – open servo gate, green LED on
 *    GATE_CLOSE      – close servo gate
 *    ACCESS_DENIED   – red LED + denial buzzer
 *    FACE_VERIFIED   – face matched, buttons now active for voting
 *    FACE_FAIL       – face mismatch buzz
 *    VOTE_CAST       – vote recorded, celebratory buzz + reset
 *    TEST_OPEN       – test servo open
 *    TEST_CLOSE      – test servo close
 *    SERVO_OPEN:xx;SERVO_CLOSE:yy  – update servo angles
 *
 *  Commands FROM ESP32 → website (sent via BLE notify):
 *    BTN1_VOTE       – user pressed Button 1 (vote candidate 1 on current slide)
 *    BTN2_VOTE       – user pressed Button 2 (vote candidate 2 on current slide)
 *    BTN3_VOTE       – user pressed Button 3 (vote candidate 3 on current slide)
 *    BTN_PREV        – navigate to previous slide (long-press BTN1)
 *    BTN_NEXT        – navigate to next  slide (long-press BTN3)
 * ============================================================
 */

// ── Dependencies ──────────────────────────────────────────
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

#define PIN_BTN1        23   // Candidate 1 / Prev slide (long-press)
#define PIN_BTN2        12   // Candidate 2 / Confirm
#define PIN_BTN3        15   // Candidate 3 / Next slide (long-press)

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
#define LONG_PRESS_MS   700   // hold > 700ms = slide navigation
#define GATE_OPEN_MS   5000   // auto-close gate after 5 s
#define LED_PULSE_MS    200   // LED blink half-period

// ── State Machine ─────────────────────────────────────────
enum VoteState {
  STATE_IDLE,          // waiting for gate entry (website sets this)
  STATE_GATE_OPEN,     // gate opened, voter walked in
  STATE_FACE_WAIT,     // waiting for face verification result
  STATE_VOTING,        // FACE_VERIFIED received — buttons active
  STATE_VOTED          // vote cast, cooling down
};
VoteState voteState = STATE_IDLE;

// ── Objects ───────────────────────────────────────────────
Servo gateServo;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

BLEServer*           pServer   = nullptr;
BLECharacteristic*   pChar     = nullptr;
bool                 bleConnected = false;

// ── Button State Tracking ─────────────────────────────────
struct Button {
  uint8_t  pin;
  bool     lastRaw;
  bool     state;         // debounced
  uint32_t pressTime;
  bool     longFired;     // long-press already sent
};
Button btns[3] = {
  {PIN_BTN1, HIGH, HIGH, 0, false},
  {PIN_BTN2, HIGH, HIGH, 0, false},
  {PIN_BTN3, HIGH, HIGH, 0, false}
};

// ── Misc State ────────────────────────────────────────────
uint32_t gateOpenedAt = 0;
bool     gateIsOpen   = false;
String   bleRxBuffer  = "";

// ─────────────────────────────────────────────────────────
//  BLE Callbacks
// ─────────────────────────────────────────────────────────
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    showOLED("BLE Connected", "Website linked");
    Serial.println("[BLE] Client connected");
    // notify website we're alive
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

// ─────────────────────────────────────────────────────────
//  OLED Helper
// ─────────────────────────────────────────────────────────
void showOLED(const char* line1, const char* line2 = "", const char* line3 = "") {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(line1);

  if (strlen(line2)) {
    oled.setCursor(0, 20);
    oled.println(line2);
  }
  if (strlen(line3)) {
    oled.setCursor(0, 40);
    oled.println(line3);
  }
  oled.display();
}

// ─────────────────────────────────────────────────────────
//  BLE Send
// ─────────────────────────────────────────────────────────
void bleSend(const String& msg) {
  if (!bleConnected || !pChar) return;
  pChar->setValue(msg.c_str());
  pChar->notify();
  Serial.print("[BLE TX] "); Serial.println(msg);
}

// ─────────────────────────────────────────────────────────
//  Servo Helpers
// ─────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────
//  LED / Buzzer Helpers
// ─────────────────────────────────────────────────────────
void setGreenLED(bool on) { digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); }
void setRedLED  (bool on) { digitalWrite(PIN_LED_RED,   on ? HIGH : LOW); }
void allLEDsOff()         { setGreenLED(false); setRedLED(false); }

void buzz(int freq_ms, int count, int gap_ms = 80) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_BUZZER, HIGH); delay(freq_ms);
    digitalWrite(PIN_BUZZER, LOW);  delay(gap_ms);
  }
}

void buzzSuccess()  { buzz(120, 3,  60); }          // triple chirp
void buzzWelcome()  { buzz(200, 2, 100); }          // double beep
void buzzDenied()   { buzz(400, 1,   0); }          // long low buzz
void buzzVoted()    { buzz(80,  5,  40); }          // rapid celebration
void buzzFaceFail() { buzz(300, 2, 200); }          // two low beeps

// ─────────────────────────────────────────────────────────
//  Handle commands from website
// ─────────────────────────────────────────────────────────
void handleWebsiteCommand(const String& cmd) {

  // ── Gate Open ──────────────────────────────────────────
  if (cmd == "GATE_OPEN") {
    voteState = STATE_GATE_OPEN;
    gateOpen();
    setGreenLED(true);
    setRedLED(false);
    showOLED("Gate Open", "Welcome!", "Proceed inside");
    buzzWelcome();
    return;
  }

  // ── Gate Close ─────────────────────────────────────────
  if (cmd == "GATE_CLOSE") {
    gateClose();
    allLEDsOff();
    showOLED("VisionVote", "Ready", "Enter Voter ID");
    return;
  }

  // ── Access Denied ──────────────────────────────────────
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

  // ── Face Verified — NOW enable buttons ─────────────────
  if (cmd == "FACE_VERIFIED") {
    voteState = STATE_VOTING;
    setGreenLED(true);
    setRedLED(false);
    showOLED("Face Verified!", "Press BTN 1/2/3", "to VOTE");
    buzzSuccess();
    return;
  }

  // ── Face Failed ────────────────────────────────────────
  if (cmd == "FACE_FAIL") {
    voteState = STATE_FACE_WAIT;   // stay in face-wait, let voter retry
    setRedLED(true);
    setGreenLED(false);
    showOLED("Face Mismatch!", "Please retry", "face verify");
    buzzFaceFail();
    delay(2000);
    setRedLED(false);
    showOLED("Retry Face", "Press cam button", "on website");
    return;
  }

  // ── Vote Cast (website confirms the vote was saved) ────
  if (cmd == "VOTE_CAST") {
    voteState = STATE_VOTED;
    allLEDsOff();
    gateClose();
    showOLED("Vote Recorded!", "Thank You!", ":)");
    buzzVoted();
    // Green LED celebratory blink
    for (int i = 0; i < 4; i++) {
      setGreenLED(true);  delay(150);
      setGreenLED(false); delay(150);
    }
    delay(2000);
    voteState = STATE_IDLE;
    showOLED("VisionVote", "Ready", "Enter Voter ID");
    return;
  }

  // ── Test commands from Admin panel ─────────────────────
  if (cmd == "TEST_OPEN")  { gateOpen();  showOLED("TEST", "Gate OPEN");  return; }
  if (cmd == "TEST_CLOSE") { gateClose(); showOLED("TEST", "Gate CLOSE"); return; }

  // ── Servo config update: "SERVO_OPEN:90;SERVO_CLOSE:0" ─
  if (cmd.startsWith("SERVO_OPEN:")) {
    int semiIdx = cmd.indexOf(';');
    String openPart  = cmd.substring(11, semiIdx);          // after "SERVO_OPEN:"
    int closeStart   = cmd.indexOf("SERVO_CLOSE:") + 12;
    String closePart = cmd.substring(closeStart);
    servoOpenAngle  = openPart.toInt();
    servoCloseAngle = closePart.toInt();
    // Clamp
    servoOpenAngle  = constrain(servoOpenAngle,  0, 180);
    servoCloseAngle = constrain(servoCloseAngle, 0, 180);
    Serial.printf("[SERVO CONFIG] Open=%d  Close=%d\n", servoOpenAngle, servoCloseAngle);
    showOLED("Servo Updated", ("Open:"+String(servoOpenAngle)).c_str(), ("Close:"+String(servoCloseAngle)).c_str());
    gateServo.write(servoCloseAngle);   // apply close angle immediately
    return;
  }
}

// ─────────────────────────────────────────────────────────
//  Button Handling
//  Buttons are INPUT_PULLUP → LOW = pressed
//
//  SHORT press (<700ms) while STATE_VOTING:
//    BTN1 → send "BTN1_VOTE"  (candidate 1 on current slide)
//    BTN2 → send "BTN2_VOTE"  (candidate 2)
//    BTN3 → send "BTN3_VOTE"  (candidate 3)
//
//  LONG press (>700ms):
//    BTN1 → send "BTN_PREV"   (prev slide, no vote)
//    BTN3 → send "BTN_NEXT"   (next slide, no vote)
// ─────────────────────────────────────────────────────────
void handleButtons() {
  uint32_t now = millis();

  for (int i = 0; i < 3; i++) {
    bool raw = (digitalRead(btns[i].pin) == LOW);   // LOW = pressed

    // Debounce
    if (raw != btns[i].lastRaw) {
      btns[i].lastRaw = raw;
      delay(DEBOUNCE_MS);
      raw = (digitalRead(btns[i].pin) == LOW);
    }

    bool wasPressed = btns[i].state;

    // Rising edge: button just pressed
    if (raw && !wasPressed) {
      btns[i].state     = true;
      btns[i].pressTime = now;
      btns[i].longFired = false;
    }

    // Held: check for long-press
    if (raw && wasPressed && !btns[i].longFired) {
      if ((now - btns[i].pressTime) >= LONG_PRESS_MS) {
        btns[i].longFired = true;
        // Long-press on BTN1 → Prev slide, BTN3 → Next slide (any state for nav)
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

    // Falling edge: button released
    if (!raw && wasPressed) {
      btns[i].state = false;
      uint32_t held = now - btns[i].pressTime;

      // Short press — only send vote if in VOTING state and long wasn't already handled
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
          // Lock buttons until website confirms with VOTE_CAST
          voteState = STATE_VOTED;
        } else {
          // Buttons pressed outside voting window — warn user
          if (i == 1) {  // only BTN2 gives feedback when not in voting state
            showOLED("Not Ready", "Complete face", "verify first");
            buzzDenied();
          }
        }
      }
    }

    btns[i].lastRaw = raw;
  }
}

// ─────────────────────────────────────────────────────────
//  Auto-close gate after timeout
// ─────────────────────────────────────────────────────────
void handleGateTimeout() {
  if (gateIsOpen && (millis() - gateOpenedAt >= GATE_OPEN_MS)) {
    gateClose();
    allLEDsOff();
    voteState = STATE_FACE_WAIT;    // inside booth; waiting for face verify
    showOLED("Gate Closed", "Waiting for", "face verify");
  }
}

// ─────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[VisionVote] ESP32 Firmware v2.0 booting...");

  // ── Pins ──────────────────────────────────────────────
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  digitalWrite(PIN_BUZZER,    LOW);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);

  // ── Servo ─────────────────────────────────────────────
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, 500, 2400);
  gateServo.write(servoCloseAngle);
  Serial.println("[SERVO] Initialised → CLOSED");

  // ── OLED ──────────────────────────────────────────────
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] ERROR: display not found — check wiring");
  } else {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("VisionVote v2.0");
    oled.setCursor(0, 20);
    oled.println("by CognoSpace");
    oled.setCursor(0, 40);
    oled.println("Starting BLE...");
    oled.display();
    Serial.println("[OLED] OK");
  }

  // ── BLE ───────────────────────────────────────────────
  BLEDevice::init("VisionVote-Coggy");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pChar = pService->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
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

  // Startup buzz
  buzz(100, 2, 80);
  Serial.println("[VisionVote] Boot complete. Ready.");
}

// ─────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────
void loop() {
  handleButtons();
  handleGateTimeout();
  delay(10);   // ~100 Hz polling — responsive without busy-loop
}
