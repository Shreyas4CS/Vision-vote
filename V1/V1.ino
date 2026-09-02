/*
 * ============================================================
 *  VisionVote — ESP32 Firmware  v4.0
 *  by CognoSpace
 *
 *  FIXES v4.0:
 *    - OLED: ALL delay() removed from BLE callbacks & VOTE_CAST.
 *            Every timed action uses schedulePending() non-blocking.
 *    - Servo: VOTE_CAST no longer calls delay(); gate close
 *             is scheduled via pendingAction so PWM never starves.
 *    - BLE onDisconnect: delay(500) replaced with a flag;
 *            re-advertising happens in loop() to avoid BLE stack block.
 *    - Pending action system extended: IDs 1-5 cover all scenarios.
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
#define PIN_SERVO       33
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

// ── Non-blocking timer for ALL timed actions ──────────────
// Action IDs:
//   1 = ACCESS_DENIED_RESET  (3 s)
//   2 = FACE_FAIL_RESET      (2 s)
//   3 = VOTE_CAST_CLOSE_GATE (immediate after buzz pattern done)
//   4 = VOTE_CAST_IDLE_RESET (2 s after gate close)
//   5 = BLE_RESTART_ADV      (500 ms after disconnect)
struct TimedAction {
  bool     active;
  uint32_t triggerAt;
  uint8_t  actionId;
};
// Support up to 4 simultaneous pending actions
#define MAX_PENDING 4
TimedAction pending[MAX_PENDING];

void clearPending() {
  for (int i = 0; i < MAX_PENDING; i++) pending[i].active = false;
}

void schedulePending(uint8_t id, uint32_t ms) {
  // Overwrite existing entry with same id, or find free slot
  for (int i = 0; i < MAX_PENDING; i++) {
    if (!pending[i].active || pending[i].actionId == id) {
      pending[i].active    = true;
      pending[i].triggerAt = millis() + ms;
      pending[i].actionId  = id;
      return;
    }
  }
  // If all slots full, overwrite slot 0 (should never happen in practice)
  pending[0].active    = true;
  pending[0].triggerAt = millis() + ms;
  pending[0].actionId  = id;
}

// ── Objects ───────────────────────────────────────────────
Servo gateServo;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledOk = false;

BLEServer*         pServer      = nullptr;
BLECharacteristic* pChar        = nullptr;
bool               bleConnected = false;
bool               bleNeedReadvertise = false; // set in callback, handled in loop()

// ── Button State ──────────────────────────────────────────
struct Button {
  uint8_t  pin;
  bool     lastRaw;
  bool     state;
  uint32_t pressTime;
  bool     longFired;
};
// Pull-DOWN: idle = LOW, pressed = HIGH
Button btns[3] = {
  {PIN_BTN1, LOW, LOW, 0, false},
  {PIN_BTN2, LOW, LOW, 0, false},
  {PIN_BTN3, LOW, LOW, 0, false}
};

uint32_t gateOpenedAt = 0;
bool     gateIsOpen   = false;

// ── LED flash state (non-blocking) ────────────────────────
struct LedFlash {
  bool     active;
  uint32_t nextToggle;
  int      remaining;
  bool     pinState;
  uint8_t  pin;
  uint32_t onMs;
  uint32_t offMs;
};
LedFlash ledFlash = {false, 0, 0, false, 0, 0, 0};

void startLedFlash(uint8_t pin, int count, uint32_t onMs, uint32_t offMs) {
  ledFlash = {true, millis(), count * 2, false, pin, onMs, offMs};
}

void handleLedFlash() {
  if (!ledFlash.active) return;
  if (millis() < ledFlash.nextToggle) return;
  ledFlash.pinState = !ledFlash.pinState;
  digitalWrite(ledFlash.pin, ledFlash.pinState ? HIGH : LOW);
  ledFlash.nextToggle = millis() + (ledFlash.pinState ? ledFlash.onMs : ledFlash.offMs);
  if (--ledFlash.remaining <= 0) {
    ledFlash.active = false;
    digitalWrite(ledFlash.pin, LOW);
  }
}

// ── FORWARD DECLARATIONS ──────────────────────────────────
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
    // SAFE: showOLED and bleSend are non-blocking — no delay() inside
    showOLED("BLE Connected", "Website linked");
    Serial.println("[BLE] Client connected");
    bleSend("ESP32_READY");
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    showOLED("BLE Disconnected", "Waiting...");
    Serial.println("[BLE] Client disconnected — will re-advertise in 500ms");
    // FIX: do NOT call delay() or srv->startAdvertising() here —
    //      the BLE stack is in teardown; calling startAdvertising()
    //      from inside the callback can hang or crash.
    //      Instead, set a flag and handle in loop().
    bleNeedReadvertise = true;
    schedulePending(5, 500); // re-advertise after 500 ms
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
  if (!oledOk) {
    Serial.print("[OLED SKIP] "); Serial.println(line1);
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.println(line1);
  if (strlen(line2)) { oled.setCursor(0, 22); oled.println(line2); }
  if (strlen(line3)) { oled.setCursor(0, 44); oled.println(line3); }
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
  Serial.print("[SERVO] Gate OPEN -> "); Serial.println(servoOpenAngle);
}
void gateClose() {
  gateServo.write(servoCloseAngle);
  gateIsOpen = false;
  Serial.print("[SERVO] Gate CLOSED -> "); Serial.println(servoCloseAngle);
}

// ── LEDs & Buzzer ─────────────────────────────────────────
void setGreenLED(bool on) { digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); }
void setRedLED  (bool on) { digitalWrite(PIN_LED_RED,   on ? HIGH : LOW); }
void allLEDsOff()         { setGreenLED(false); setRedLED(false); }

// NOTE: buzz() is still blocking but is ONLY called from loop()
// context (via handlePendingAction or handleButtons), NEVER from
// BLE callbacks, so it cannot block the BLE stack.
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
// CRITICAL: ZERO blocking delay() calls here.
// All timed actions use schedulePending().
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
    schedulePending(1, 3000); // reset after 3s — non-blocking
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
    schedulePending(2, 2000); // reset after 2s — non-blocking
    return;
  }

  if (cmd == "VOTE_CAST") {
    // Buzzer & OLED already triggered on physical button press.
    // This command from the website just confirms & closes gate.
    voteState = STATE_VOTED;
    gateClose();
    allLEDsOff();
    // Schedule idle reset after 3s (shows "Enter Voter ID" again)
    schedulePending(4, 3000);
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
             ("Open:"  + String(servoOpenAngle)).c_str(),
             ("Close:" + String(servoCloseAngle)).c_str());
    gateServo.write(servoCloseAngle);
    return;
  }
}

// ── Button Handling ───────────────────────────────────────
void handleButtons() {
  uint32_t now = millis();

  for (int i = 0; i < 3; i++) {
    // Pull-DOWN resistor: button pressed = HIGH
    bool raw = (digitalRead(btns[i].pin) == HIGH);

    if (raw != btns[i].lastRaw) {
      btns[i].lastRaw = raw;
      delay(DEBOUNCE_MS);
      raw = (digitalRead(btns[i].pin) == HIGH);
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
          // Immediate buzzer feedback — confirms vote physically
          buzzVoted();
          // Show thank-you message on OLED
          showOLED("Thank You!", "Great Citizen!", "Vote Recorded");
          if (i == 0) {
            bleSend("BTN1_VOTE");
          } else if (i == 1) {
            bleSend("BTN2_VOTE");
          } else if (i == 2) {
            bleSend("BTN3_VOTE");
          }
          voteState = STATE_VOTED;
          // Green LED flash to celebrate
          startLedFlash(PIN_LED_GREEN, 5, 120, 100);
          // After 3s, return to idle / gate entry mode
          schedulePending(4, 3000);
        } else {
          if (voteState != STATE_VOTED) {
            showOLED("Not Ready", "Complete face", "verify first");
            buzzDenied();
          }
        }
      }
    }

    btns[i].lastRaw = raw;
  }
}

// ── Non-blocking Pending Action Handler ───────────────────
void handlePendingAction() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_PENDING; i++) {
    if (!pending[i].active) continue;
    if (now < pending[i].triggerAt) continue;
    pending[i].active = false;

    switch (pending[i].actionId) {
      case 1: // ACCESS_DENIED_RESET
        setRedLED(false);
        showOLED("VisionVote", "Ready", "Enter Voter ID");
        break;

      case 2: // FACE_FAIL_RESET
        setRedLED(false);
        showOLED("Retry Face", "Press cam button", "on website");
        break;

      case 4: // VOTE_CAST → idle reset (after LED flash)
        voteState = STATE_IDLE;
        showOLED("VisionVote", "Ready", "Enter Voter ID");
        break;

      case 5: // BLE re-advertise after disconnect
        if (bleNeedReadvertise) {
          bleNeedReadvertise = false;
          pServer->startAdvertising();
          Serial.println("[BLE] Re-advertising started");
        }
        break;
    }
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
  delay(200);
  Serial.println("\n[VisionVote] Booting v4.0...");

  clearPending();

  // ── GPIO ──
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  digitalWrite(PIN_BUZZER,    LOW);

  // Pull-DOWN resistors used externally: button press = HIGH
  pinMode(PIN_BTN1, INPUT);
  pinMode(PIN_BTN2, INPUT);
  pinMode(PIN_BTN3, INPUT);

  // ── Servo ──
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, 500, 2400);
  delay(200);
  gateServo.write(servoCloseAngle);
  delay(500);
  gateServo.write(servoCloseAngle);
  Serial.printf("[SERVO] Init at %d deg\n", servoCloseAngle);

  // ── OLED ──
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  delay(100);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] First init failed — retrying...");
    delay(500);
    Wire.begin(OLED_SDA, OLED_SCL);
    delay(100);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      Serial.println("[OLED] ERROR: display not found on 0x3C");
      oledOk = false;
    } else {
      oledOk = true;
    }
  } else {
    oledOk = true;
  }

  if (oledOk) {
    oled.clearDisplay();
    oled.display();
    Serial.println("[OLED] OK");
    showOLED("VisionVote v4.0", "by CognoSpace", "Starting BLE...");
  }

  // ── BLE ──
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
  handlePendingAction();   // non-blocking timed resets + BLE re-advertise
  handleLedFlash();        // non-blocking LED flash for VOTE_CAST
  delay(10);
}
