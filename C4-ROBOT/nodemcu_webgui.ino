#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Servo.h>

#include <LittleFS.h>
#include <ArduinoJson.h>

// ============================================================
// Firmware version
// ============================================================
static const char* FW_VERSION = "0.89";

// ============================================================
// Persistent config (LittleFS + JSON)
// ============================================================
static const char* CONFIG_PATH = "/config.json";

// Defaults
static const char* DEFAULT_WIFI_SSID = "Z-Wave Automation";
static const char* DEFAULT_WIFI_PASS = "Fiber714Cvet";

static const uint16_t DEFAULT_MAX_SPS = 800;
static const float    DEFAULT_ACCEL   = 2500.0f;

static const uint8_t  POS_COUNT = 40;

// Servo microsecond clamp range
static const uint16_t SERVO_US_MIN_CLAMP = 400;
static const uint16_t SERVO_US_MAX_CLAMP = 2600;

// Axis type
enum AxisType : uint8_t { AXIS_V = 0, AXIS_H = 1 };

struct AppConfig {
  String wifiSsid;
  String wifiPass;
  AxisType axis = AXIS_V;

  uint16_t maxSps = DEFAULT_MAX_SPS;
  float accel = DEFAULT_ACCEL;

  // Motor pins
  uint8_t pinEn = 16;    // D0
  uint8_t pinDir = 5;    // D1
  uint8_t pinStep = 4;   // D2

  // Endstops
  uint8_t pinEndBegin = 12;  // D6 (NEG)
  uint8_t pinEndEnd = 13;    // D7 (POS)

  // Logic levels (requested)
  bool enActiveHigh = true;     // true: EN=HIGH enables driver
  bool dirActiveHigh = true;    // true: DIR=HIGH means positive direction
  bool stepActiveHigh = true;   // true: STEP pulse is HIGH->LOW (default)

  // Servo config (horizontal mode)
  uint8_t  servoTopPin     = 14;   // D5
  uint16_t servoTopMin     = 1000; // release position µs
  uint16_t servoTopMax     = 2000; // press position µs

  uint8_t  servoBotPin     = 0;    // D3
  uint16_t servoBotMin     = 1000; // release position µs
  uint16_t servoBotMax     = 2000; // press position µs

  uint16_t servoPress_ms   = 250;  // hold at press position
  uint16_t servoRelease_ms = 250;  // hold at release position
  uint16_t servoPause_ms   = 250;  // pause between consecutive clicks

  long positions[POS_COUNT];
};

AppConfig gCfg;

// Servo objects (used only in AXIS_H mode)
Servo gServoTop;
Servo gServoBot;

// ============================================================
// NodeMCU blue LED (D4 / GPIO2) - ACTIVE LOW on most boards
// Requirement: LED ON while motor is moving
// ============================================================
static const uint8_t STATUS_LED_PIN = LED_BUILTIN; // GPIO2 (D4)
static const bool    STATUS_LED_ACTIVE_LOW = true;

static inline void ledSet(bool on) {
  bool level = STATUS_LED_ACTIVE_LOW ? !on : on;
  digitalWrite(STATUS_LED_PIN, level ? HIGH : LOW);
}

// ============================================================
// Device identity
// ============================================================
String deviceName;
String mqttClientId;

static AxisType strToAxis(const String& s) { return (s == "H" || s == "h") ? AXIS_H : AXIS_V; }
static const char* axisSegment(AxisType a) { return (a == AXIS_H) ? "horizontal" : "vertical"; }
static const char* controllerTitle(AxisType a) { return (a == AXIS_H) ? "Horizontal Axis Controller" : "Vertical Axis Controller"; }
static const char* posLabel(AxisType a) { return (a == AXIS_H) ? "Device" : "Row"; }

static void setDefaults() {
  gCfg.wifiSsid = DEFAULT_WIFI_SSID;
  gCfg.wifiPass = DEFAULT_WIFI_PASS;
  gCfg.axis = AXIS_V;
  gCfg.maxSps = DEFAULT_MAX_SPS;
  gCfg.accel = DEFAULT_ACCEL;

  gCfg.pinEn = 16;
  gCfg.pinDir = 5;
  gCfg.pinStep = 4;
  gCfg.pinEndBegin = 12;
  gCfg.pinEndEnd = 13;

  gCfg.enActiveHigh = true;
  gCfg.dirActiveHigh = true;
  gCfg.stepActiveHigh = true;

  gCfg.servoTopPin     = 14;
  gCfg.servoTopMin     = 1000;
  gCfg.servoTopMax     = 2000;
  gCfg.servoBotPin     = 0;
  gCfg.servoBotMin     = 1000;
  gCfg.servoBotMax     = 2000;
  gCfg.servoPress_ms   = 250;
  gCfg.servoRelease_ms = 250;
  gCfg.servoPause_ms   = 250;

  for (uint8_t i = 0; i < POS_COUNT; i++) gCfg.positions[i] = 0;
}

// ============================================================
// htmlEscape
// ============================================================
static String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
}

// ============================================================
// GPIO->Dx labeling
// ============================================================
static const char* gpioToDxLabel(uint8_t gpio) {
  switch (gpio) {
    case 16: return "D0";
    case 5:  return "D1";
    case 4:  return "D2";
    case 0:  return "D3";
    case 2:  return "D4";
    case 14: return "D5";
    case 12: return "D6";
    case 13: return "D7";
    case 15: return "D8";
    case 3:  return "RX";
    case 1:  return "TX";
    default: return "";
  }
}
static String pinLabel(uint8_t gpio) {
  const char* dx = gpioToDxLabel(gpio);
  if (dx && dx[0]) return String("GPIO") + gpio + " (" + dx + ")";
  return String("GPIO") + gpio;
}
static void appendPinOptions(String& s, uint8_t selectedGpio) {
  const uint8_t pins[] = { 16, 5, 4, 0, 2, 14, 12, 13, 15, 3, 1 };
  for (size_t i = 0; i < sizeof(pins); i++) {
    uint8_t p = pins[i];
    s += F("<option value='");
    s += String(p);
    s += F("'");
    if (p == selectedGpio) s += F(" selected");
    s += F(">");
    s += pinLabel(p);
    s += F("</option>");
  }
}

// ============================================================
// ArduinoJson helpers
// ============================================================
static String jsonGetString(JsonVariantConst v, const char* def) {
  if (v.is<const char*>()) return String(v.as<const char*>());
  if (v.is<String>()) return v.as<String>();
  return String(def);
}
static uint16_t jsonGetU16(JsonVariantConst v, uint16_t def) {
  if (v.is<uint16_t>()) return v.as<uint16_t>();
  if (v.is<int>()) {
    int x = v.as<int>();
    if (x < 0) x = 0;
    if (x > 65535) x = 65535;
    return (uint16_t)x;
  }
  if (v.is<const char*>()) {
    long x = String(v.as<const char*>()).toInt();
    if (x < 0) x = 0;
    if (x > 65535) x = 65535;
    return (uint16_t)x;
  }
  return def;
}
static float jsonGetFloat(JsonVariantConst v, float def) {
  if (v.is<float>() || v.is<double>()) return v.as<float>();
  if (v.is<int>()) return (float)v.as<int>();
  if (v.is<const char*>()) return String(v.as<const char*>()).toFloat();
  return def;
}
static uint8_t jsonGetU8(JsonVariantConst v, uint8_t def) {
  if (v.is<uint8_t>()) return v.as<uint8_t>();
  if (v.is<int>()) {
    int x = v.as<int>();
    if (x < 0) x = 0;
    if (x > 255) x = 255;
    return (uint8_t)x;
  }
  if (v.is<const char*>()) {
    long x = String(v.as<const char*>()).toInt();
    if (x < 0) x = 0;
    if (x > 255) x = 255;
    return (uint8_t)x;
  }
  return def;
}
static long jsonGetLong(JsonVariantConst v, long def) {
  if (v.is<long>()) return v.as<long>();
  if (v.is<int>()) return (long)v.as<int>();
  if (v.is<const char*>()) return String(v.as<const char*>()).toInt();
  return def;
}

// ============================================================
// Config load/save
// ============================================================
bool loadConfig() {
  if (!LittleFS.begin()) { setDefaults(); return false; }
  if (!LittleFS.exists(CONFIG_PATH)) { setDefaults(); return true; }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) { setDefaults(); return false; }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { setDefaults(); return false; }

  gCfg.wifiSsid = jsonGetString(doc["wifi"]["ssid"], DEFAULT_WIFI_SSID);
  gCfg.wifiPass = jsonGetString(doc["wifi"]["pass"], DEFAULT_WIFI_PASS);
  gCfg.axis = strToAxis(jsonGetString(doc["axis"], "V"));

  gCfg.maxSps = jsonGetU16(doc["tune"]["max_sps"], DEFAULT_MAX_SPS);
  gCfg.accel  = jsonGetFloat(doc["tune"]["accel"], DEFAULT_ACCEL);

  gCfg.pinEn       = jsonGetU8(doc["pins"]["motor"]["en"], 16);
  gCfg.pinDir      = jsonGetU8(doc["pins"]["motor"]["dir"], 5);
  gCfg.pinStep     = jsonGetU8(doc["pins"]["motor"]["step"], 4);
  gCfg.pinEndBegin = jsonGetU8(doc["pins"]["endstops"]["begin"], 12);
  gCfg.pinEndEnd   = jsonGetU8(doc["pins"]["endstops"]["end"], 13);

  gCfg.enActiveHigh   = (bool)(doc["logic"]["en_active_high"]   | true);
  gCfg.dirActiveHigh  = (bool)(doc["logic"]["dir_active_high"]  | true);
  gCfg.stepActiveHigh = (bool)(doc["logic"]["step_active_high"] | true);

  // Servo config
  gCfg.servoTopPin     = jsonGetU8 (doc["servos"]["top"]["pin"],    14);
  gCfg.servoTopMin     = jsonGetU16(doc["servos"]["top"]["min_us"], 1000);
  gCfg.servoTopMax     = jsonGetU16(doc["servos"]["top"]["max_us"], 2000);
  gCfg.servoBotPin     = jsonGetU8 (doc["servos"]["bot"]["pin"],    0);
  gCfg.servoBotMin     = jsonGetU16(doc["servos"]["bot"]["min_us"], 1000);
  gCfg.servoBotMax     = jsonGetU16(doc["servos"]["bot"]["max_us"], 2000);
  gCfg.servoPress_ms   = jsonGetU16(doc["servos"]["press_ms"],      250);
  gCfg.servoRelease_ms = jsonGetU16(doc["servos"]["release_ms"],    250);
  gCfg.servoPause_ms   = jsonGetU16(doc["servos"]["pause_ms"],      250);

  for (uint8_t i = 0; i < POS_COUNT; i++) gCfg.positions[i] = 0;
  if (doc["positions"].is<JsonArrayConst>()) {
    JsonArrayConst arr = doc["positions"].as<JsonArrayConst>();
    uint8_t i = 0;
    for (JsonVariantConst v : arr) {
      if (i >= POS_COUNT) break;
      gCfg.positions[i] = jsonGetLong(v, 0);
      i++;
    }
  }
  return true;
}

bool saveConfig() {
  JsonDocument doc;
  doc["axis"] = (gCfg.axis == AXIS_H) ? "H" : "V";
  doc["wifi"]["ssid"] = gCfg.wifiSsid;
  doc["wifi"]["pass"] = gCfg.wifiPass;

  doc["tune"]["max_sps"] = gCfg.maxSps;
  doc["tune"]["accel"] = gCfg.accel;

  doc["pins"]["motor"]["en"] = gCfg.pinEn;
  doc["pins"]["motor"]["dir"] = gCfg.pinDir;
  doc["pins"]["motor"]["step"] = gCfg.pinStep;

  doc["pins"]["endstops"]["begin"] = gCfg.pinEndBegin;
  doc["pins"]["endstops"]["end"] = gCfg.pinEndEnd;

  doc["logic"]["en_active_high"]   = gCfg.enActiveHigh;
  doc["logic"]["dir_active_high"]  = gCfg.dirActiveHigh;
  doc["logic"]["step_active_high"] = gCfg.stepActiveHigh;

  // Servo config
  doc["servos"]["top"]["pin"]    = gCfg.servoTopPin;
  doc["servos"]["top"]["min_us"] = gCfg.servoTopMin;
  doc["servos"]["top"]["max_us"] = gCfg.servoTopMax;
  doc["servos"]["bot"]["pin"]    = gCfg.servoBotPin;
  doc["servos"]["bot"]["min_us"] = gCfg.servoBotMin;
  doc["servos"]["bot"]["max_us"] = gCfg.servoBotMax;
  doc["servos"]["press_ms"]      = gCfg.servoPress_ms;
  doc["servos"]["release_ms"]    = gCfg.servoRelease_ms;
  doc["servos"]["pause_ms"]      = gCfg.servoPause_ms;

  JsonArray pos = doc["positions"].to<JsonArray>();
  for (uint8_t i = 0; i < POS_COUNT; i++) pos.add(gCfg.positions[i]);

  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) return false;
  bool ok = serializeJson(doc, f) > 0;
  f.close();
  return ok;
}

// ============================================================
// Identity
// ============================================================
static String macLast3BytesHex() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[7];
  snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(buf);
}
static void computeDeviceIdentity() {
  deviceName = String("nodemcu-axis-") + axisSegment(gCfg.axis) + "-" + macLast3BytesHex();
  mqttClientId = deviceName;
}

// ============================================================
// WiFi + OTA + MQTT + Motion
// ============================================================
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;

IPAddress MQTT_HOST(192, 168, 1, 6);
const uint16_t MQTT_PORT = 1883;

String mqttBase;
String T_MOVE;
String T_GOTO;
String T_SPEED;
String T_ACC;
String T_STOP;
String T_STATE;

static const uint16_t STEP_PULSE_HIGH_US = 20;
static const uint16_t STEP_PULSE_LOW_US  = 20;
static const uint16_t DIR_SETUP_US       = 50;

static const long MAX_STEPS_PER_CMD = 20000000;
static const long HOME_STEPS = 5000000;

static const uint16_t SPEED_SPS_MIN = 50;
static const uint16_t SPEED_SPS_MAX = 10000;

static const float ACC_MIN = 50.0f;
static const float ACC_MAX = 80000.0f;

volatile uint16_t maxSps = DEFAULT_MAX_SPS;
volatile float accSps2 = DEFAULT_ACCEL;

volatile long posSteps = 0;
volatile bool stopRequested = false;

// Counter model
volatile unsigned long moveCounterLiveAbs = 0;
volatile unsigned long lastMoveDoneAbsFinal = 0;
volatile bool isMoving = false;

// Test mode state
volatile bool testModeActive = false;
volatile uint8_t testModeCycle = 0;      // 0..10
volatile bool testModeTargetEnd = true;  // true: go to END, false: go to BEGIN

ESP8266WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastMqttReconnectAttemptMs = 0;
unsigned long lastStatePublishMs = 0;

static inline bool endstopPressed(uint8_t pin) { return digitalRead(pin) == LOW; }

// Endstop lockout
static inline bool bothEndstopsPressed() {
  return endstopPressed(gCfg.pinEndBegin) && endstopPressed(gCfg.pinEndEnd);
}
void requestStop() { stopRequested = true; }
static inline bool motionBlockedByEndstops() {
  if (!bothEndstopsPressed()) return false;
  requestStop();
  return true;
}

static inline uint16_t clampSps(long sps) {
  if (sps < SPEED_SPS_MIN) return SPEED_SPS_MIN;
  if (sps > SPEED_SPS_MAX) return SPEED_SPS_MAX;
  return (uint16_t)sps;
}
static inline float clampAcc(float a) {
  if (a < ACC_MIN) return ACC_MIN;
  if (a > ACC_MAX) return ACC_MAX;
  return a;
}

// Background service (keep web+OTA responsive while moving)
static inline void serviceBackground() {
  yield();
  ArduinoOTA.handle();
  mqtt.loop();
  server.handleClient();
}

// Logic-level aware pin functions (requested)
static inline void setEnable(bool on) {
  bool level = gCfg.enActiveHigh ? on : !on;
  digitalWrite(gCfg.pinEn, level ? HIGH : LOW);
}
static inline void setDirPositive(bool positive) {
  bool level = gCfg.dirActiveHigh ? positive : !positive;
  digitalWrite(gCfg.pinDir, level ? HIGH : LOW);
}
static inline void stepPulse() {
  if (gCfg.stepActiveHigh) {
    digitalWrite(gCfg.pinStep, HIGH);
    delayMicroseconds(STEP_PULSE_HIGH_US);
    digitalWrite(gCfg.pinStep, LOW);
    delayMicroseconds(STEP_PULSE_LOW_US);
  } else {
    digitalWrite(gCfg.pinStep, LOW);
    delayMicroseconds(STEP_PULSE_HIGH_US);
    digitalWrite(gCfg.pinStep, HIGH);
    delayMicroseconds(STEP_PULSE_LOW_US);
  }
}

static inline unsigned long atomicReadULong(volatile unsigned long& v) {
  noInterrupts();
  unsigned long x = v;
  interrupts();
  return x;
}
static inline long atomicReadLong(volatile long& v) {
  noInterrupts();
  long x = v;
  interrupts();
  return x;
}
static inline bool atomicReadBool(volatile bool& v) {
  noInterrupts();
  bool x = v;
  interrupts();
  return x;
}
static inline uint8_t atomicReadU8(volatile uint8_t& v) {
  noInterrupts();
  uint8_t x = v;
  interrupts();
  return x;
}

void clearStop() { stopRequested = false; }

// ============================================================
// Servo helpers
// ============================================================
static inline uint16_t clampServoUs(uint16_t us) {
  if (us < SERVO_US_MIN_CLAMP) return SERVO_US_MIN_CLAMP;
  if (us > SERVO_US_MAX_CLAMP) return SERVO_US_MAX_CLAMP;
  return us;
}

static void servoWriteUs(Servo& sv, uint8_t pin, uint16_t us) {
  us = clampServoUs(us);
  if (!sv.attached()) sv.attach(pin, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);
  sv.writeMicroseconds(us);
}

static void servoClick(Servo& sv, uint8_t pin, uint16_t minUs, uint16_t maxUs, uint8_t n) {
  minUs = clampServoUs(minUs);
  maxUs = clampServoUs(maxUs);
  if (n == 0) n = 1;
  if (n > 50) n = 50;
  if (!sv.attached()) sv.attach(pin, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);

  for (uint8_t i = 0; i < n; i++) {
    // Press (move to max_us)
    sv.writeMicroseconds(maxUs);
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < gCfg.servoPress_ms) {
      serviceBackground();
      yield();
    }
    // Release (move to min_us)
    sv.writeMicroseconds(minUs);
    t0 = millis();
    while ((uint32_t)(millis() - t0) < gCfg.servoRelease_ms) {
      serviceBackground();
      yield();
    }
    // Pause between clicks (skip after last click)
    if (i < (uint8_t)(n - 1)) {
      t0 = millis();
      while ((uint32_t)(millis() - t0) < gCfg.servoPause_ms) {
        serviceBackground();
        yield();
      }
    }
  }
}

// ============================================================
// WiFi/MQTT helpers
// ============================================================
bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.hostname(deviceName);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  WiFi.begin(gCfg.wifiSsid.c_str(), gCfg.wifiPass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    yield();
  }
  return (WiFi.status() == WL_CONNECTED);
}

void buildMqttTopics() {
  mqttBase = String("nodemcu/axis/") + axisSegment(gCfg.axis) + "/";
  T_MOVE  = mqttBase + "cmd/move";
  T_GOTO  = mqttBase + "cmd/goto";
  T_SPEED = mqttBase + "cmd/speed";
  T_ACC   = mqttBase + "cmd/accel";
  T_STOP  = mqttBase + "cmd/stop";
  T_STATE = mqttBase + "state";
}

// ============================================================
// Motion
// ============================================================
bool moveStepsAccel(long signedSteps) {
  if (motionBlockedByEndstops()) return false;

  clearStop();
  moveCounterLiveAbs = 0;

  if (signedSteps == 0) return true;

  // LED ON while moving
  ledSet(true);
  isMoving = true;

  if (signedSteps > MAX_STEPS_PER_CMD) signedSteps = MAX_STEPS_PER_CMD;
  if (signedSteps < -MAX_STEPS_PER_CMD) signedSteps = -MAX_STEPS_PER_CMD;

  bool positive = (signedSteps > 0);

  // pos=0 rule on BEGIN; block moving further negative
  if (endstopPressed(gCfg.pinEndBegin)) {
    posSteps = 0;
    if (!positive) {
      isMoving = false;
      ledSet(false);
      return false;
    }
  }

  long stepsTotal = labs(signedSteps);
  uint16_t vmax = clampSps((long)maxSps);
  float a = clampAcc(accSps2);

  setDirPositive(positive);
  delayMicroseconds(DIR_SETUP_US);
  setEnable(true);

  float v = 0.0f;
  unsigned long doneAbs = 0;
  bool aborted = false;

  for (long i = 0; i < stepsTotal; i++) {
    if (stopRequested) { aborted = true; break; }
    if (bothEndstopsPressed()) { aborted = true; requestStop(); break; }

    if (!positive && endstopPressed(gCfg.pinEndBegin)) { posSteps = 0; aborted = true; break; }
    if ( positive && endstopPressed(gCfg.pinEndEnd))   { aborted = true; break; }

    long remaining = stepsTotal - i;
    float dStop = (v * v) / (2.0f * a);
    float dt = 1.0f / (v > 1.0f ? v : 1.0f);

    if ((float)remaining <= dStop + 1.0f) {
      v -= a * dt;
      if (v < 1.0f) v = 1.0f;
    } else {
      v += a * dt;
      if (v > (float)vmax) v = (float)vmax;
      if (v < 1.0f) v = 1.0f;
    }

    stepPulse();
    doneAbs++;
    moveCounterLiveAbs = doneAbs;

    posSteps += positive ? 1 : -1;

    uint32_t periodUs = (uint32_t)(1000000.0f / v);
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < periodUs) {
      serviceBackground();
      if (stopRequested) { aborted = true; break; }
    }
    if (aborted) break;
  }

  lastMoveDoneAbsFinal = doneAbs;

  isMoving = false;
  ledSet(false);

  return (!aborted && (doneAbs == (unsigned long)stepsTotal));
}

bool goEnd() {
  if (motionBlockedByEndstops()) return false;
  if (endstopPressed(gCfg.pinEndEnd)) return true;
  return moveStepsAccel(+HOME_STEPS);
}
bool goBegin() {
  if (motionBlockedByEndstops()) return false;
  if (!endstopPressed(gCfg.pinEndBegin)) {
    bool ok = moveStepsAccel(-HOME_STEPS);
    if (!ok && !endstopPressed(gCfg.pinEndBegin)) return false;
  }
  posSteps = 0;
  return true;
}
bool gotoIndex(uint8_t idx1to40) {
  if (motionBlockedByEndstops()) return false;
  if (idx1to40 < 1 || idx1to40 > POS_COUNT) return false;
  long target = gCfg.positions[idx1to40 - 1];
  long delta = target - posSteps;
  return moveStepsAccel(delta);
}

// ============================================================
// Test mode: 10 cycles endstop-to-endstop with 5s wait at endstop
// ============================================================
static void testModeStop() {
  testModeActive = false;
  testModeCycle = 0;
  testModeTargetEnd = true;
  requestStop();
}
static bool waitWithService(uint32_t ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ms) {
    serviceBackground();
    if (stopRequested) return false;
    if (!testModeActive) return false;
    delay(5);
  }
  return true;
}
static bool moveToEndstop(bool toEnd) {
  if (toEnd) {
    if (endstopPressed(gCfg.pinEndEnd)) return true;
    return moveStepsAccel(+HOME_STEPS);
  } else {
    if (endstopPressed(gCfg.pinEndBegin)) { posSteps = 0; return true; }
    bool ok = moveStepsAccel(-HOME_STEPS);
    if (endstopPressed(gCfg.pinEndBegin)) posSteps = 0;
    return ok || endstopPressed(gCfg.pinEndBegin);
  }
}
static void runTestModeBlocking() {
  if (motionBlockedByEndstops()) return;

  testModeActive = true;
  testModeCycle = 0;
  testModeTargetEnd = true;

  clearStop();

  for (uint8_t cycle = 1; cycle <= 10; cycle++) {
    if (!testModeActive || stopRequested || motionBlockedByEndstops()) break;

    testModeCycle = cycle;

    testModeTargetEnd = true;
    moveToEndstop(true);
    if (!testModeActive || stopRequested) break;
    if (!waitWithService(5000)) break;

    testModeTargetEnd = false;
    moveToEndstop(false);
    if (!testModeActive || stopRequested) break;
    if (!waitWithService(5000)) break;
  }

  testModeActive = false;
}

// ============================================================
// Status shown in GUI (plain text)
// ============================================================
String statusText() {
  String s;
  s.reserve(1800);

  unsigned long doneFinal = atomicReadULong(lastMoveDoneAbsFinal);
  unsigned long live = atomicReadULong(moveCounterLiveAbs);

  bool movingSnap = atomicReadBool(isMoving);
  long posSnap = atomicReadLong(posSteps);

  bool tm = atomicReadBool(testModeActive);
  uint8_t cyc = atomicReadU8(testModeCycle);
  bool tgtEnd = atomicReadBool(testModeTargetEnd);

  s += "FW=" + String(FW_VERSION);
  s += "\nAxis=" + String(axisSegment(gCfg.axis));

  if (bothEndstopsPressed()) s += "\nERROR=Both endstops pressed (movement disabled)";

  s += "\nMOVING=" + String(movingSnap ? "yes" : "no");
  s += "\npos=" + String(posSnap);

  if (tm) {
    s += "\nTEST_MODE=ACTIVE";
    s += "\nTEST_CYCLE=" + String((int)cyc) + "/10";
    s += "\nTEST_TARGET=" + String(tgtEnd ? "END" : "BEGIN");
  } else {
    s += "\nTEST_MODE=OFF";
  }

  if (movingSnap) s += "\nDone steps (live abs)=" + String(live);
  else            s += "\nLast done steps (final abs)=" + String(doneFinal);

  s += "\nEndstops: BEGIN=" + String(endstopPressed(gCfg.pinEndBegin) ? "PRESSED" : "open");
  s += " END=" + String(endstopPressed(gCfg.pinEndEnd) ? "PRESSED" : "open");

  s += "\nPins: EN(Blue)=" + pinLabel(gCfg.pinEn) +
       " STEP(Green)=" + pinLabel(gCfg.pinStep) +
       " DIR(Yellow)=" + pinLabel(gCfg.pinDir);

  s += "\nLogic: EN_active=" + String(gCfg.enActiveHigh ? "HIGH" : "LOW");
  s += " DIR_pos=" + String(gCfg.dirActiveHigh ? "HIGH" : "LOW");
  s += " STEP_pulse=" + String(gCfg.stepActiveHigh ? "HIGH->LOW" : "LOW->HIGH");

  return s;
}

void publishState() {
  if (!mqtt.connected()) return;
  String st = statusText();
  mqtt.publish(T_STATE.c_str(), st.c_str(), true);
}

// ============================================================
// HTML
// ============================================================
String htmlHeader(const __FlashStringHelper* pageTitle) {
  String s;
  s.reserve(3400);
  s += F("<!doctype html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>");
  s += F("<style>"
         "body{font-family:system-ui,Arial;margin:16px;max-width:1100px}"
         "a{color:#0b57d0;text-decoration:none}"
         "button{padding:10px 12px;margin:4px;font-size:14px;border:1px solid #ddd;border-radius:10px}"
         "input,select{padding:8px;font-size:14px;width:240px;max-width:100%}"
         "label{min-width:140px}"
         ".row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}"
         ".card{border:1px solid #ddd;border-radius:10px;padding:14px;margin:12px 0}"
         "pre{background:#f7f7f7;border:1px solid #eee;padding:10px;border-radius:8px;white-space:pre-wrap}"
         ".stop{background:#b00020;color:white;border:none;border-radius:10px}"
         ".home{background:#cde7ff;}"
         ".go{background:#c8f7c5;}"
         ".test{background:#ffe08a;}"
         ".save{background:#1565c0;color:white;border:none;}"
         ".nav{display:flex;gap:12px;margin:8px 0 16px 0}"
         "table{border-collapse:collapse;width:100%}"
         "td,th{border:1px solid #eee;padding:6px;text-align:left;vertical-align:top}"
         ".small{font-size:12px;color:#555}"
         ".posInput{width:140px}"
         ".err{color:#b00020;font-weight:700}"
         ".blue{color:#1565c0;font-weight:700}"
         ".green{color:#2e7d32;font-weight:700}"
         ".yellow{color:#f9a825;font-weight:700}"
         "</style>");
  s += F("<title>");
  s += pageTitle;
  s += F("</title></head><body>");
  s += F("<div class='nav'>"
         "<a href='/'>Home</a>"
         "<a href='/config'>Config</a>"
         "</div>");
  return s;
}

static void appendGotoOptions(String& s) {
  for (uint8_t i = 1; i <= POS_COUNT; i++) {
    s += F("<option value='");
    s += String(i);
    s += F("'>");
    s += posLabel(gCfg.axis);
    s += F(" ");
    s += String(i);
    s += F("</option>");
  }
}

String htmlHome() {
  String s;
  s.reserve(20000);

  s += htmlHeader(F("Axis Controller"));
  s += F("<h2>");
  s += controllerTitle(gCfg.axis);
  s += F("</h2>");

  if (bothEndstopsPressed()) {
    s += F("<div class='card err'>ERROR: Both endstops are PRESSED. Movement is disabled.</div>");
  }

  // Control
  s += F("<div class='card'><h3>Control</h3>"
         "<div class='row'>"
         "<button class='home' onclick='beginPos()'>BEGIN</button>"
         "<button class='stop' onclick='stopAll()'>STOP</button>"
         "<button class='home' onclick='endPos()'>END</button>"
         "<label style='margin-left:8px'>GOTO ");
  s += posLabel(gCfg.axis);
  s += F(":</label>"
         "<select id='gotoSel'>");
  appendGotoOptions(s);
  s += F("</select>"
         "<button class='go' onclick='gotoSel()'>GO</button>"
         "<button class='test' onclick='testMode()'>TEST MODE (10x)</button>"
         "</div></div>");

  // Move
  s += F("<div class='card'><h3>Move</h3>"
         "<div class='row'>"
         "<button onclick='mv(-10000)'>-10000</button>"
         "<button onclick='mv(-1000)'>-1000</button>"
         "<button onclick='mv(-100)'>-100</button>"
         "<button onclick='mv(-10)'>-10</button>"
         "<button onclick='mv(10)'>+10</button>"
         "<button onclick='mv(100)'>+100</button>"
         "<button onclick='mv(1000)'>+1000</button>"
         "<button onclick='mv(10000)'>+10000</button>"
         "</div>"
         "<div class='row'>"
         "<label>custom:</label>"
         "<input id='steps' type='number' value='200' step='1'>"
         "<button onclick='mvCustom(-1)'>-custom</button>"
         "<button onclick='mvCustom(1)'>+custom</button>"
         "</div></div>");

  // Pins summary on Home (colored + Dx)
  s += F("<div class='card'><h3>Pins</h3><div class='row'>"
         "<div><span class='blue'>EN (Blue)</span>: ");
  s += htmlEscape(pinLabel(gCfg.pinEn));
  s += F("</div><div><span class='green'>STEP (Green)</span>: ");
  s += htmlEscape(pinLabel(gCfg.pinStep));
  s += F("</div><div><span class='yellow'>DIR (Yellow)</span>: ");
  s += htmlEscape(pinLabel(gCfg.pinDir));
  s += F("</div></div>"
         "<div class='small'>Logic: EN=");
  s += (gCfg.enActiveHigh ? "HIGH" : "LOW");
  s += F(" DIR_POS=");
  s += (gCfg.dirActiveHigh ? "HIGH" : "LOW");
  s += F(" STEP=");
  s += (gCfg.stepActiveHigh ? "HIGH->LOW" : "LOW->HIGH");
  s += F("</div></div>");

  // Status
  s += F("<div class='card'><h3>Status</h3>"
         "<pre id='status'>Loading...</pre>"
         "<div class='row'><button onclick='refresh()'>Refresh</button></div>"
         "</div>");

  s += F("<script>"
         "async function refresh(){document.getElementById('status').innerText=await (await fetch('/status')).text();}"
         "async function beginPos(){document.getElementById('status').innerText=await (await fetch('/home?dir=begin')).text();}"
         "async function endPos(){document.getElementById('status').innerText=await (await fetch('/home?dir=end')).text();}"
         "async function stopAll(){document.getElementById('status').innerText=await (await fetch('/stop')).text();}"
         "async function mv(steps){document.getElementById('status').innerText=await (await fetch('/move?steps='+steps)).text();}"
         "async function mvCustom(sign){"
         "  const v=parseInt(document.getElementById('steps').value||'0',10);"
         "  document.getElementById('status').innerText=await (await fetch('/move?steps='+(sign*v))).text();"
         "}"
         "async function gotoSel(){"
         "  const i=parseInt(document.getElementById('gotoSel').value||'0',10);"
         "  document.getElementById('status').innerText=await (await fetch('/goto?i='+i)).text();"
         "}"
         "async function testMode(){"
         "  document.getElementById('status').innerText=await (await fetch('/test')).text();"
         "}"
         "refresh();"
         "</script>");

  s += F("</body></html>");
  return s;
}

static void appendBoolSelect(String& s, const char* name, bool val, const __FlashStringHelper* onLabel, const __FlashStringHelper* offLabel) {
  s += F("<select name='");
  s += name;
  s += F("'>");
  if (val) {
    s += F("<option value='1' selected>");
    s += onLabel;
    s += F("</option><option value='0'>");
    s += offLabel;
    s += F("</option>");
  } else {
    s += F("<option value='1'>");
    s += onLabel;
    s += F("</option><option value='0' selected>");
    s += offLabel;
    s += F("</option>");
  }
  s += F("</select>");
}

String htmlConfig() {
  String s;
  s.reserve(40000);

  s += htmlHeader(F("Config"));
  s += F("<h2>");
  s += controllerTitle(gCfg.axis);
  s += F(" - Config</h2>");

  if (bothEndstopsPressed()) {
    s += F("<div class='card err'>ERROR: Both endstops are PRESSED. Movement is disabled.</div>");
  }

  s += F("<div class='card'>"
         "<form method='POST' action='/config/save'>");

  // ---- Save button at the TOP ----
  s += F("<div class='row' style='margin-bottom:12px'>"
         "<button type='submit' class='save'>&#128190; Save config</button>"
         "</div>");

  s += F("<div class='row'><label>WiFi SSID</label><input name='ssid' required value='");
  s += htmlEscape(gCfg.wifiSsid);
  s += F("'></div>");

  s += F("<div class='row'><label>WiFi Password (also OTA password)</label><input name='pass' type='password' value='");
  s += htmlEscape(gCfg.wifiPass);
  s += F("'></div>");

  s += F("<div class='row'><label>Axis</label><select name='axis' id='axisSelect'>");
  s += (gCfg.axis == AXIS_V) ? F("<option value='V' selected>Vertical (V)</option>") : F("<option value='V'>Vertical (V)</option>");
  s += (gCfg.axis == AXIS_H) ? F("<option value='H' selected>Horizontal (H)</option>") : F("<option value='H'>Horizontal (H)</option>");
  s += F("</select></div>");

  s += F("<div class='row'><label>Default max_sps</label><input name='max_sps' type='number' min='50' max='10000' step='10' value='");
  s += String(gCfg.maxSps);
  s += F("'></div>");

  s += F("<div class='row'><label>Default accel</label><input name='accel' type='number' min='50' max='80000' step='50' value='");
  s += String((int)gCfg.accel);
  s += F("'></div>");

  // Pins
  s += F("<div class='card'><h3>Motor Pins</h3>");

  s += F("<div class='row'><label><span class='blue'>EN (Blue)</span></label><select name='pin_en'>");
  appendPinOptions(s, gCfg.pinEn);
  s += F("</select></div>");

  s += F("<div class='row'><label><span class='green'>STEP (Green)</span></label><select name='pin_step'>");
  appendPinOptions(s, gCfg.pinStep);
  s += F("</select></div>");

  s += F("<div class='row'><label><span class='yellow'>DIR (Yellow)</span></label><select name='pin_dir'>");
  appendPinOptions(s, gCfg.pinDir);
  s += F("</select></div>");

  s += F("</div>");

  // Logic levels
  s += F("<div class='card'><h3>Logic levels</h3>");

  s += F("<div class='row'><label>EN active</label>");
  appendBoolSelect(s, "en_ah", gCfg.enActiveHigh, F("HIGH enables"), F("LOW enables"));
  s += F("</div>");

  s += F("<div class='row'><label>DIR positive</label>");
  appendBoolSelect(s, "dir_ah", gCfg.dirActiveHigh, F("HIGH = POS"), F("LOW = POS"));
  s += F("</div>");

  s += F("<div class='row'><label>STEP pulse</label>");
  appendBoolSelect(s, "step_ah", gCfg.stepActiveHigh, F("HIGH->LOW"), F("LOW->HIGH"));
  s += F("</div>");

  s += F("</div>");

  // ---- Servo config (shown only when Horizontal axis) ----
  s += F("<div class='card' id='servoCard'");
  if (gCfg.axis != AXIS_H) s += F(" style='display:none'");
  s += F("><h3>Button Servos (Horizontal mode)</h3>");

  // Top Button Servo
  s += F("<h4>Top Button Servo</h4>");
  s += F("<div class='row'><label>Pin</label><select name='sv_top_pin'>");
  appendPinOptions(s, gCfg.servoTopPin);
  s += F("</select></div>");
  s += F("<div class='row'><label>Min &micro;s (release)</label>"
         "<input name='sv_top_min' type='number' min='400' max='2600' step='10' value='");
  s += String(gCfg.servoTopMin);
  s += F("'></div>");
  s += F("<div class='row'><label>Max &micro;s (press)</label>"
         "<input name='sv_top_max' type='number' min='400' max='2600' step='10' value='");
  s += String(gCfg.servoTopMax);
  s += F("'></div>");

  // Bottom Button Servo
  s += F("<h4>Bottom Button Servo</h4>");
  s += F("<div class='row'><label>Pin</label><select name='sv_bot_pin'>");
  appendPinOptions(s, gCfg.servoBotPin);
  s += F("</select></div>");
  s += F("<div class='row'><label>Min &micro;s (release)</label>"
         "<input name='sv_bot_min' type='number' min='400' max='2600' step='10' value='");
  s += String(gCfg.servoBotMin);
  s += F("'></div>");
  s += F("<div class='row'><label>Max &micro;s (press)</label>"
         "<input name='sv_bot_max' type='number' min='400' max='2600' step='10' value='");
  s += String(gCfg.servoBotMax);
  s += F("'></div>");

  // Servo timing
  s += F("<h4>Click timing</h4>");
  s += F("<div class='row'><label>Press hold (ms)</label>"
         "<input name='sv_press_ms' type='number' min='50' max='5000' step='50' value='");
  s += String(gCfg.servoPress_ms);
  s += F("'></div>");
  s += F("<div class='row'><label>Release hold (ms)</label>"
         "<input name='sv_release_ms' type='number' min='50' max='5000' step='50' value='");
  s += String(gCfg.servoRelease_ms);
  s += F("'></div>");
  s += F("<div class='row'><label>Pause between clicks (ms)</label>"
         "<input name='sv_pause_ms' type='number' min='50' max='5000' step='50' value='");
  s += String(gCfg.servoPause_ms);
  s += F("'></div>");

  // Servo test buttons
  s += F("<div class='card'><h3>Servo test</h3>");
  s += F("<p><strong>Top Servo:</strong></p>");
  s += F("<div class='row'>"
         "<button type='button' onclick=\"svTest('/servo/top/min')\">Top MIN</button>"
         "<button type='button' onclick=\"svTest('/servo/top/max')\">Top MAX</button>"
         "<button type='button' onclick=\"svTest('/servo/top/click?n=1')\">Top &times;1</button>"
         "<button type='button' onclick=\"svTest('/servo/top/click?n=3')\">Top &times;3</button>"
         "<button type='button' onclick=\"svTest('/servo/top/click?n=4')\">Top &times;4</button>"
         "<button type='button' onclick=\"svTest('/servo/top/click?n=9')\">Top &times;9</button>"
         "<button type='button' onclick=\"svTest('/servo/top/click?n=13')\">Top &times;13</button>"
         "</div>");
  s += F("<p><strong>Bottom Servo:</strong></p>");
  s += F("<div class='row'>"
         "<button type='button' onclick=\"svTest('/servo/bot/min')\">Bot MIN</button>"
         "<button type='button' onclick=\"svTest('/servo/bot/max')\">Bot MAX</button>"
         "<button type='button' onclick=\"svTest('/servo/bot/click?n=1')\">Bot &times;1</button>"
         "<button type='button' onclick=\"svTest('/servo/bot/click?n=3')\">Bot &times;3</button>"
         "<button type='button' onclick=\"svTest('/servo/bot/click?n=4')\">Bot &times;4</button>"
         "<button type='button' onclick=\"svTest('/servo/bot/click?n=9')\">Bot &times;9</button>"
         "<button type='button' onclick=\"svTest('/servo/bot/click?n=13')\">Bot &times;13</button>"
         "</div>");
  s += F("<pre id='svtest' style='margin-top:10px'>Servo test output...</pre></div>");

  s += F("</div>"); // end servoCard

  // Pin test buttons (requested)
  s += F("<div class='card'><h3>Pin test</h3>"
         "<div class='row'>"
         "<button type='button' onclick=\"doTest('/pin/en?on=1')\">EN ON</button>"
         "<button type='button' onclick=\"doTest('/pin/en?on=0')\">EN OFF</button>"
         "<button type='button' onclick=\"doTest('/pin/dir?pos=1')\">DIR POS</button>"
         "<button type='button' onclick=\"doTest('/pin/dir?pos=0')\">DIR NEG</button>"
         "</div>"
         "<div class='row'>"
         "<button type='button' onclick=\"doTest('/pin/step?n=1')\">STEP x1</button>"
         "<button type='button' onclick=\"doTest('/pin/step?n=10')\">STEP x10</button>"
         "<button type='button' onclick=\"doTest('/pin/step?n=100')\">STEP x100</button>"
         "</div>"
         "<pre id='ptest' style='margin-top:10px'>Pin test output...</pre>"
         "<div class='small'>WARNING: STEP test will move motor. Endstops are not enforced here.</div>"
         "</div>");

  // Positions 4-col
  s += F("<div class='card'><h3>Positions (steps from BEGIN=0)</h3>");
  s += F("<table><tr>"
         "<th>#</th><th>Steps</th>"
         "<th>#</th><th>Steps</th>"
         "<th>#</th><th>Steps</th>"
         "<th>#</th><th>Steps</th>"
         "</tr>");
  for (uint8_t r = 0; r < 10; r++) {
    s += F("<tr>");
    for (uint8_t c = 0; c < 4; c++) {
      uint8_t i = (uint8_t)(r + c * 10);
      s += F("<td>");
      s += posLabel(gCfg.axis);
      s += F(" ");
      s += String(i + 1);
      s += F("</td><td><input class='posInput' name='pos_");
      s += String(i + 1);
      s += F("' type='number' step='1' value='");
      s += String(gCfg.positions[i]);
      s += F("'></td>");
    }
    s += F("</tr>");
  }
  s += F("</table></div>");

  // ---- Save button at the BOTTOM ----
  s += F("<div class='row'><button type='submit' class='save'>&#128190; Save config</button></div>"
         "<p class='small'>After saving, device will reboot.</p>"
         "</form></div>");

  // ---- Backup / Restore card (outside main form) ----
  s += F("<div class='card'><h3>&#128462; Backup / Restore config</h3>"
         "<div class='row'>"
         "<a href='/config/download'>"
         "<button type='button'>&#11015; Download config.json</button>"
         "</a>"
         "</div>"
         "<div class='row' style='margin-top:12px'>"
         "<form method='POST' action='/config/upload' enctype='multipart/form-data'>"
         "<input type='file' name='file' accept='.json' required style='width:auto'>"
         "<button type='submit'>&#11014; Upload &amp; Restore</button>"
         "</form>"
         "</div>"
         "<div class='small' style='margin-top:6px'>Upload overwrites /config.json and reboots the device.</div>"
         "</div>");

  s += F("<script>"
         "async function doTest(url){"
         "  const t=document.getElementById('ptest');"
         "  t.textContent='Running '+url+' ...';"
         "  try{ t.textContent=await (await fetch(url)).text(); }catch(e){ t.textContent='ERR '+e; }"
         "}"
         "async function svTest(url){"
         "  const t=document.getElementById('svtest');"
         "  t.textContent='Running '+url+' ...';"
         "  try{ t.textContent=await (await fetch(url)).text(); }catch(e){ t.textContent='ERR '+e; }"
         "}"
         "function updateServoVis(){"
         "  var ax=document.getElementById('axisSelect').value;"
         "  document.getElementById('servoCard').style.display=(ax==='H')?'':'none';"
         "}"
         "document.getElementById('axisSelect').addEventListener('change',updateServoVis);"
         "</script>");

  s += F("</body></html>");
  return s;
}

// ============================================================
// Web Handlers
// ============================================================
void handleRoot() { server.send(200, "text/html; charset=utf-8", htmlHome()); }
void handleStatus() { server.send(200, "text/plain; charset=utf-8", statusText()); }
void handleConfig() { server.send(200, "text/html; charset=utf-8", htmlConfig()); }

void handleConfigSave() {
  // required
  if (!server.hasArg("ssid") || !server.hasArg("pass") || !server.hasArg("axis") ||
      !server.hasArg("max_sps") || !server.hasArg("accel") ||
      !server.hasArg("pin_en") || !server.hasArg("pin_dir") || !server.hasArg("pin_step") ||
      !server.hasArg("en_ah") || !server.hasArg("dir_ah") || !server.hasArg("step_ah")) {
    server.send(400, "text/plain; charset=utf-8", "Missing required config fields");
    return;
  }

  gCfg.wifiSsid = server.arg("ssid");
  gCfg.wifiPass = server.arg("pass");
  gCfg.axis = strToAxis(server.arg("axis"));

  gCfg.maxSps = clampSps(server.arg("max_sps").toInt());
  gCfg.accel = clampAcc(server.arg("accel").toFloat());

  gCfg.pinEn = (uint8_t)server.arg("pin_en").toInt();
  gCfg.pinStep = (uint8_t)server.arg("pin_step").toInt();
  gCfg.pinDir = (uint8_t)server.arg("pin_dir").toInt();

  gCfg.enActiveHigh   = server.arg("en_ah").toInt() == 1;
  gCfg.dirActiveHigh  = server.arg("dir_ah").toInt() == 1;
  gCfg.stepActiveHigh = server.arg("step_ah").toInt() == 1;

  // Servo params (always save so they persist regardless of current axis)
  if (server.hasArg("sv_top_pin"))    gCfg.servoTopPin     = (uint8_t)server.arg("sv_top_pin").toInt();
  if (server.hasArg("sv_top_min"))    gCfg.servoTopMin     = clampServoUs((uint16_t)server.arg("sv_top_min").toInt());
  if (server.hasArg("sv_top_max"))    gCfg.servoTopMax     = clampServoUs((uint16_t)server.arg("sv_top_max").toInt());
  if (server.hasArg("sv_bot_pin"))    gCfg.servoBotPin     = (uint8_t)server.arg("sv_bot_pin").toInt();
  if (server.hasArg("sv_bot_min"))    gCfg.servoBotMin     = clampServoUs((uint16_t)server.arg("sv_bot_min").toInt());
  if (server.hasArg("sv_bot_max"))    gCfg.servoBotMax     = clampServoUs((uint16_t)server.arg("sv_bot_max").toInt());
  if (server.hasArg("sv_press_ms"))   gCfg.servoPress_ms   = (uint16_t)max(50, min(5000, (int)server.arg("sv_press_ms").toInt()));
  if (server.hasArg("sv_release_ms")) gCfg.servoRelease_ms = (uint16_t)max(50, min(5000, (int)server.arg("sv_release_ms").toInt()));
  if (server.hasArg("sv_pause_ms"))   gCfg.servoPause_ms   = (uint16_t)max(50, min(5000, (int)server.arg("sv_pause_ms").toInt()));

  for (uint8_t i = 1; i <= POS_COUNT; i++) {
    String k = String("pos_") + String(i);
    if (server.hasArg(k)) gCfg.positions[i - 1] = server.arg(k).toInt();
  }

  bool ok = saveConfig();
  if (!ok) {
    server.send(500, "text/plain; charset=utf-8", "FAILED to save config");
    return;
  }

  // wait 5s, redirect to home, reboot
  String page;
  page.reserve(1600);
  page += F("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Saved</title>"
            "<style>body{font-family:system-ui,Arial;margin:16px} .muted{color:#666}</style>"
            "</head><body>");
  page += F("<h3>Saved.</h3>");
  page += F("<p>Rebooting in <span id='sec'>5</span> seconds&#8230;</p>");
  page += F("<p class='muted'>If you changed WiFi/axis, the IP/hostname may change after reboot.</p>");
  page += F("<script>"
            "let n=5;"
            "const el=document.getElementById('sec');"
            "const t=setInterval(()=>{n--; el.textContent=String(n); if(n<=0){clearInterval(t);}},1000);"
            "setTimeout(()=>{ window.location.href='/'; }, 5000);"
            "</script>");
  page += F("</body></html>");

  server.send(200, "text/html; charset=utf-8", page);
  delay(5000);
  ESP.restart();
}

// ---- Config Download ----
void handleConfigDownload() {
  if (!LittleFS.exists(CONFIG_PATH)) {
    server.send(404, "text/plain; charset=utf-8", "No config file found");
    return;
  }
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    server.send(500, "text/plain; charset=utf-8", "Cannot open config");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"config.json\"");
  server.streamFile(f, "application/json");
  f.close();
}

// ---- Config Upload ----
static File gUploadFile;

void handleConfigUploadFile() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    gUploadFile = LittleFS.open(CONFIG_PATH, "w");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (gUploadFile) gUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (gUploadFile) gUploadFile.close();
  }
}

void handleConfigUploadDone() {
  // Verify uploaded file contains valid JSON
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    server.send(500, "text/plain; charset=utf-8", "Upload failed: cannot re-open file");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    LittleFS.remove(CONFIG_PATH);
    server.send(400, "text/plain; charset=utf-8",
                String("Invalid JSON: ") + err.c_str() + ". File removed.");
    return;
  }

  String page;
  page.reserve(1000);
  page += F("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Restored</title>"
            "<style>body{font-family:system-ui,Arial;margin:16px}</style>"
            "</head><body>");
  page += F("<h3>Config restored.</h3>"
            "<p>Rebooting in <span id='sec'>5</span> seconds&#8230;</p>");
  page += F("<script>"
            "let n=5;"
            "const el=document.getElementById('sec');"
            "const t=setInterval(()=>{n--; el.textContent=String(n); if(n<=0){clearInterval(t);}},1000);"
            "setTimeout(()=>{ window.location.href='/'; }, 5000);"
            "</script>");
  page += F("</body></html>");

  server.send(200, "text/html; charset=utf-8", page);
  delay(5000);
  ESP.restart();
}

void handleStop() {
  testModeStop();
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText());
}

void handleHome() {
  if (!server.hasArg("dir")) { server.send(400, "text/plain; charset=utf-8", "Missing dir"); return; }
  String dir = server.arg("dir");
  bool ok = false;
  if (dir == "begin") ok = goBegin();
  else if (dir == "end") ok = goEnd();
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText() + (ok ? "\nhome=OK" : "\nhome=FAILED"));
}

void handleMove() {
  if (!server.hasArg("steps")) { server.send(400, "text/plain; charset=utf-8", "Missing steps"); return; }
  long steps = server.arg("steps").toInt();
  bool ok = moveStepsAccel(steps);
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText() + (ok ? "\nmove=OK" : "\nmove=STOP_OR_ENDSTOP"));
}

void handleGoto() {
  if (!server.hasArg("i")) { server.send(400, "text/plain; charset=utf-8", "Missing i"); return; }
  int i = server.arg("i").toInt();
  bool ok = gotoIndex((uint8_t)i);
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText() + (ok ? "\ngoto=OK" : "\ngoto=FAILED"));
}

void handleTest() {
  if (bothEndstopsPressed()) {
    server.send(409, "text/plain; charset=utf-8", "ERROR: Both endstops pressed. Movement disabled.");
    return;
  }
  if (testModeActive) {
    testModeStop();
    server.send(200, "text/plain; charset=utf-8", "TEST MODE stopped.\n\n" + statusText());
    return;
  }

  server.send(200, "text/plain; charset=utf-8", "TEST MODE starting (10 cycles). Use STOP to abort.\n\n" + statusText());
  delay(50);

  runTestModeBlocking();
  publishState();
}

// Pin test endpoints (requested)
void handlePinEn() {
  if (!server.hasArg("on")) { server.send(400, "text/plain; charset=utf-8", "missing on"); return; }
  bool on = server.arg("on").toInt() == 1;
  setEnable(on);
  server.send(200, "text/plain; charset=utf-8", String("OK EN=") + (on ? "ON" : "OFF") +
                                            " (EN active " + (gCfg.enActiveHigh ? "HIGH" : "LOW") + ")");
}

void handlePinDir() {
  if (!server.hasArg("pos")) { server.send(400, "text/plain; charset=utf-8", "missing pos"); return; }
  bool pos = server.arg("pos").toInt() == 1;
  setDirPositive(pos);
  delayMicroseconds(DIR_SETUP_US);
  server.send(200, "text/plain; charset=utf-8", String("OK DIR=") + (pos ? "POS" : "NEG") +
                                            " (POS is " + (gCfg.dirActiveHigh ? "HIGH" : "LOW") + ")");
}

void handlePinStep() {
  int n = server.hasArg("n") ? server.arg("n").toInt() : 1;
  if (n < 1) n = 1;
  if (n > 5000) n = 5000;

  // WARNING: no endstop enforcement here (as requested "force for test")
  setEnable(true);
  for (int i = 0; i < n; i++) {
    stepPulse();
    serviceBackground();
  }

  server.send(200, "text/plain; charset=utf-8", String("OK STEP pulses=") + n +
                                            " (pulse " + (gCfg.stepActiveHigh ? "HIGH->LOW" : "LOW->HIGH") + ")");
}

// ---- Servo endpoints ----
void handleServoTopMin() {
  if (gCfg.axis != AXIS_H) { server.send(400, "text/plain; charset=utf-8", "Servo only in H mode"); return; }
  servoWriteUs(gServoTop, gCfg.servoTopPin, gCfg.servoTopMin);
  server.send(200, "text/plain; charset=utf-8",
              String("OK top MIN ") + gCfg.servoTopMin + F("us on GPIO") + gCfg.servoTopPin);
}

void handleServoTopMax() {
  if (gCfg.axis != AXIS_H) { server.send(400, "text/plain; charset=utf-8", "Servo only in H mode"); return; }
  servoWriteUs(gServoTop, gCfg.servoTopPin, gCfg.servoTopMax);
  server.send(200, "text/plain; charset=utf-8",
              String("OK top MAX ") + gCfg.servoTopMax + F("us on GPIO") + gCfg.servoTopPin);
}

void handleServoTopClick() {
  if (gCfg.axis != AXIS_H) { server.send(400, "text/plain; charset=utf-8", "Servo only in H mode"); return; }
  int n = server.hasArg("n") ? server.arg("n").toInt() : 1;
  if (n < 1) n = 1;
  if (n > 50) n = 50;
  servoClick(gServoTop, gCfg.servoTopPin, gCfg.servoTopMin, gCfg.servoTopMax, (uint8_t)n);
  server.send(200, "text/plain; charset=utf-8", String("OK top click x") + n);
}

void handleServoBotMin() {
  if (gCfg.axis != AXIS_H) { server.send(400, "text/plain; charset=utf-8", "Servo only in H mode"); return; }
  servoWriteUs(gServoBot, gCfg.servoBotPin, gCfg.servoBotMin);
  server.send(200, "text/plain; charset=utf-8",
              String("OK bot MIN ") + gCfg.servoBotMin + F("us on GPIO") + gCfg.servoBotPin);
}

void handleServoBotMax() {
  if (gCfg.axis != AXIS_H) { server.send(400, "text/plain; charset=utf-8", "Servo only in H mode"); return; }
  servoWriteUs(gServoBot, gCfg.servoBotPin, gCfg.servoBotMax);
  server.send(200, "text/plain; charset=utf-8",
              String("OK bot MAX ") + gCfg.servoBotMax + F("us on GPIO") + gCfg.servoBotPin);
}

void handleServoBotClick() {
  if (gCfg.axis != AXIS_H) { server.send(400, "text/plain; charset=utf-8", "Servo only in H mode"); return; }
  int n = server.hasArg("n") ? server.arg("n").toInt() : 1;
  if (n < 1) n = 1;
  if (n > 50) n = 50;
  servoClick(gServoBot, gCfg.servoBotPin, gCfg.servoBotMin, gCfg.servoBotMax, (uint8_t)n);
  server.send(200, "text/plain; charset=utf-8", String("OK bot click x") + n);
}

// ============================================================
// MQTT
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length + 1);
  for (unsigned i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String t(topic);
  long ival = msg.toInt();
  float fval = msg.toFloat();

  if (t == T_STOP) requestStop();
  else if (t == T_SPEED) maxSps = clampSps(ival);
  else if (t == T_ACC) accSps2 = clampAcc(fval);
  else if (t == T_MOVE) moveStepsAccel(ival);
  else if (t == T_GOTO) gotoIndex((uint8_t)ival);

  publishState();
}

bool mqttEnsureConnected() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (mqtt.connected()) return true;

  unsigned long now = millis();
  if (now - lastMqttReconnectAttemptMs < 2000) return false;
  lastMqttReconnectAttemptMs = now;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  if (!mqtt.connect(mqttClientId.c_str())) return false;

  mqtt.subscribe(T_MOVE.c_str());
  mqtt.subscribe(T_GOTO.c_str());
  mqtt.subscribe(T_SPEED.c_str());
  mqtt.subscribe(T_ACC.c_str());
  mqtt.subscribe(T_STOP.c_str());
  publishState();
  return true;
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(STATUS_LED_PIN, OUTPUT);
  ledSet(false);

  loadConfig();
  computeDeviceIdentity();
  buildMqttTopics();

  maxSps = clampSps((long)gCfg.maxSps);
  accSps2 = clampAcc(gCfg.accel);

  // Apply configurable pins
  pinMode(gCfg.pinEn, OUTPUT);
  pinMode(gCfg.pinDir, OUTPUT);
  pinMode(gCfg.pinStep, OUTPUT);

  // Put pins into a defined idle state
  digitalWrite(gCfg.pinStep, LOW);
  setDirPositive(true);
  setEnable(true);

  pinMode(gCfg.pinEndBegin, INPUT_PULLUP);
  pinMode(gCfg.pinEndEnd, INPUT_PULLUP);

  ensureWiFiConnected();

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/config", handleConfig);
  server.on("/config/save", HTTP_POST, handleConfigSave);
  server.on("/config/download", HTTP_GET, handleConfigDownload);
  server.on("/config/upload", HTTP_POST, handleConfigUploadDone, handleConfigUploadFile);
  server.on("/stop", handleStop);
  server.on("/home", handleHome);
  server.on("/move", handleMove);
  server.on("/goto", handleGoto);
  server.on("/test", handleTest);

  // pin test
  server.on("/pin/en", handlePinEn);
  server.on("/pin/dir", handlePinDir);
  server.on("/pin/step", handlePinStep);

  // servo endpoints (horizontal mode)
  server.on("/servo/top/min",   handleServoTopMin);
  server.on("/servo/top/max",   handleServoTopMax);
  server.on("/servo/top/click", handleServoTopClick);
  server.on("/servo/bot/min",   handleServoBotMin);
  server.on("/servo/bot/max",   handleServoBotMax);
  server.on("/servo/bot/click", handleServoBotClick);

  server.begin();

  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.setPassword(gCfg.wifiPass.c_str());
  ArduinoOTA.onStart([]() { requestStop(); });
  ArduinoOTA.begin();

  mqttEnsureConnected();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  mqttEnsureConnected();
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastStatePublishMs > 2000) {
    lastStatePublishMs = now;
    publishState();
  }
}
