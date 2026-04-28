#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Servo.h>

// ============================================================
// Firmware version
// ============================================================
static const char* FW_VERSION = "0.92";

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

  long positions[POS_COUNT];

  // Servo button (Horizontal mode only)
  uint8_t  servoTopPin        = 14;   // D5
  uint16_t servoTopMinUs      = 600;  // hardware calibration: pulse at 0°
  uint16_t servoTopMaxUs      = 2400; // hardware calibration: pulse at 180°

  // Angle-based positions (degrees 0-180)
  uint8_t  servoTopMinDeg      = 0;   // idle / rest
  uint8_t  servoTopApproachDeg = 90;  // hover near button
  uint8_t  servoTopMaxDeg      = 180; // full press

  uint8_t  servoBotPin        = 12;   // D6
  uint16_t servoBotMinUs      = 600;  // hardware calibration: pulse at 0°
  uint16_t servoBotMaxUs      = 2400; // hardware calibration: pulse at 180°

  // Angle-based positions (degrees 0-180)
  uint8_t  servoBotMinDeg      = 0;
  uint8_t  servoBotApproachDeg = 90;
  uint8_t  servoBotMaxDeg      = 180;

  // Shared click timing (ms)
  uint16_t clickApproachMs = 300;
  uint16_t clickPressMs    = 180;
  uint16_t clickBetweenMs  = 220;
  uint16_t clickReturnMs   = 300;
};

AppConfig gCfg;

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

  gCfg.servoTopPin        = 14;
  gCfg.servoTopMinUs      = 600;
  gCfg.servoTopMaxUs      = 2400;
  gCfg.servoTopMinDeg      = 0;
  gCfg.servoTopApproachDeg = 90;
  gCfg.servoTopMaxDeg      = 180;

  gCfg.servoBotPin        = 12;
  gCfg.servoBotMinUs      = 600;
  gCfg.servoBotMaxUs      = 2400;
  gCfg.servoBotMinDeg      = 0;
  gCfg.servoBotApproachDeg = 90;
  gCfg.servoBotMaxDeg      = 180;

  gCfg.clickApproachMs = 300;
  gCfg.clickPressMs    = 180;
  gCfg.clickBetweenMs  = 220;
  gCfg.clickReturnMs   = 300;

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

  // Servo fields (Horizontal mode)
  gCfg.servoTopPin    = jsonGetU8 (doc["servo"]["top"]["pin"],    14);
  gCfg.servoTopMinUs  = jsonGetU16(doc["servo"]["top"]["min_us"], 600);
  gCfg.servoTopMaxUs  = jsonGetU16(doc["servo"]["top"]["max_us"], 2400);

  gCfg.servoTopMinDeg      = jsonGetU8(doc["servo"]["top"]["min_deg"],      0);
  gCfg.servoTopApproachDeg = jsonGetU8(doc["servo"]["top"]["approach_deg"], 90);
  gCfg.servoTopMaxDeg      = jsonGetU8(doc["servo"]["top"]["max_deg"],      180);

  gCfg.servoBotPin    = jsonGetU8 (doc["servo"]["bottom"]["pin"],    12);
  gCfg.servoBotMinUs  = jsonGetU16(doc["servo"]["bottom"]["min_us"], 600);
  gCfg.servoBotMaxUs  = jsonGetU16(doc["servo"]["bottom"]["max_us"], 2400);

  gCfg.servoBotMinDeg      = jsonGetU8(doc["servo"]["bottom"]["min_deg"],      0);
  gCfg.servoBotApproachDeg = jsonGetU8(doc["servo"]["bottom"]["approach_deg"], 90);
  gCfg.servoBotMaxDeg      = jsonGetU8(doc["servo"]["bottom"]["max_deg"],      180);

  gCfg.clickApproachMs = jsonGetU16(doc["servo"]["timing"]["approach_ms"], 300);
  gCfg.clickPressMs    = jsonGetU16(doc["servo"]["timing"]["press_ms"],    180);
  gCfg.clickBetweenMs  = jsonGetU16(doc["servo"]["timing"]["between_ms"],  220);
  gCfg.clickReturnMs   = jsonGetU16(doc["servo"]["timing"]["return_ms"],   300);

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

  // Servo fields
  doc["servo"]["top"]["pin"]         = gCfg.servoTopPin;
  doc["servo"]["top"]["min_us"]      = gCfg.servoTopMinUs;
  doc["servo"]["top"]["max_us"]      = gCfg.servoTopMaxUs;
  doc["servo"]["top"]["min_deg"]     = gCfg.servoTopMinDeg;
  doc["servo"]["top"]["approach_deg"]= gCfg.servoTopApproachDeg;
  doc["servo"]["top"]["max_deg"]     = gCfg.servoTopMaxDeg;

  doc["servo"]["bottom"]["pin"]         = gCfg.servoBotPin;
  doc["servo"]["bottom"]["min_us"]      = gCfg.servoBotMinUs;
  doc["servo"]["bottom"]["max_us"]      = gCfg.servoBotMaxUs;
  doc["servo"]["bottom"]["min_deg"]     = gCfg.servoBotMinDeg;
  doc["servo"]["bottom"]["approach_deg"]= gCfg.servoBotApproachDeg;
  doc["servo"]["bottom"]["max_deg"]     = gCfg.servoBotMaxDeg;

  doc["servo"]["timing"]["approach_ms"] = gCfg.clickApproachMs;
  doc["servo"]["timing"]["press_ms"]    = gCfg.clickPressMs;
  doc["servo"]["timing"]["between_ms"]  = gCfg.clickBetweenMs;
  doc["servo"]["timing"]["return_ms"]   = gCfg.clickReturnMs;

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
String T_EVENT;

// Name of the command currently driving motion (set by each handler before
// calling motion functions; used as the "command" field in event payloads).
static String activeCommand = "";

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

// Non-blocking wait that keeps web responsive
static void waitMs(uint32_t ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ms) {
    serviceBackground();
    delay(2);
  }
}

// ============================================================
// Servo helpers (Horizontal button mode)
// ============================================================
static const uint16_t SERVO_US_MIN = 400;
static const uint16_t SERVO_US_MAX = 2600;
static const uint8_t  SERVO_CLICKS_MAX = 50;

static Servo gServoTop;
static Servo gServoBot;

static inline uint16_t clampServoUs(int v) {
  if (v < SERVO_US_MIN) v = SERVO_US_MIN;
  if (v > SERVO_US_MAX) v = SERVO_US_MAX;
  return (uint16_t)v;
}

// Map angle in degrees (0-180) linearly to the calibrated µs range.
static inline uint16_t degToUs(uint8_t deg, uint16_t minUs, uint16_t maxUs) {
  if (deg > 180) deg = 180;
  return clampServoUs((int)minUs + (int)((uint32_t)(maxUs - minUs) * deg / 180));
}

static void servoAttachIfNeeded(Servo& sv, uint8_t pin) {
  if (!sv.attached()) sv.attach(pin);
}

// Write a specific angle (degrees) to a servo using the calibrated µs range.
static void servoWriteDeg(Servo& sv, uint8_t pin,
                          uint8_t deg,
                          uint16_t minUs, uint16_t maxUs) {
  servoAttachIfNeeded(sv, pin);
  sv.writeMicroseconds(degToUs(deg, minUs, maxUs));
}

static void servoInit() {
  if (gCfg.axis != AXIS_H) return;
  servoAttachIfNeeded(gServoTop, gCfg.servoTopPin);
  gServoTop.writeMicroseconds(degToUs(gCfg.servoTopMinDeg, gCfg.servoTopMinUs, gCfg.servoTopMaxUs));
  servoAttachIfNeeded(gServoBot, gCfg.servoBotPin);
  gServoBot.writeMicroseconds(degToUs(gCfg.servoBotMinDeg, gCfg.servoBotMinUs, gCfg.servoBotMaxUs));
}

// Move servo to a named position: "min", "approach", "max".
// Angles come from the degree config; µs calibration is applied via degToUs.
// Returns false if name unknown.
static bool servoMoveToPos(Servo& sv, uint8_t pin,
                           uint8_t minDeg, uint8_t approachDeg, uint8_t maxDeg,
                           uint16_t minUs, uint16_t maxUs,
                           const String& posName) {
  servoAttachIfNeeded(sv, pin);
  if (posName == "min") {
    sv.writeMicroseconds(degToUs(minDeg, minUs, maxUs));
  } else if (posName == "approach") {
    sv.writeMicroseconds(degToUs(approachDeg, minUs, maxUs));
  } else if (posName == "max") {
    sv.writeMicroseconds(degToUs(maxDeg, minUs, maxUs));
  } else {
    return false;
  }
  return true;
}

// Human-like N-click sequence: MIN → APPROACH → (MAX→APPROACH)×N → MIN
// All positions expressed in degrees; converted to µs via calibration.
static void servoClick(Servo& sv, uint8_t pin,
                       uint8_t minDeg, uint8_t approachDeg, uint8_t maxDeg,
                       uint16_t minUs, uint16_t maxUs,
                       uint8_t n) {
  if (n < 1) n = 1;
  if (n > SERVO_CLICKS_MAX) n = SERVO_CLICKS_MAX;
  servoAttachIfNeeded(sv, pin);
  sv.writeMicroseconds(degToUs(minDeg, minUs, maxUs));
  waitMs(20);
  sv.writeMicroseconds(degToUs(approachDeg, minUs, maxUs));
  waitMs(gCfg.clickApproachMs);
  for (uint8_t i = 0; i < n; i++) {
    sv.writeMicroseconds(degToUs(maxDeg, minUs, maxUs));
    waitMs(gCfg.clickPressMs);
    sv.writeMicroseconds(degToUs(approachDeg, minUs, maxUs));
    if (i < n - 1) waitMs(gCfg.clickBetweenMs);
  }
  sv.writeMicroseconds(degToUs(minDeg, minUs, maxUs));
  waitMs(gCfg.clickReturnMs);
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
  T_EVENT = mqttBase + "event";
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

  // Publish motion_started event before the step loop.
  // mqtt.publish() buffers the message; it is transmitted during the
  // first serviceBackground() / mqtt.loop() call inside the loop.
  publishEvent("motion_started", posSteps);

  float v = 0.0f;
  unsigned long doneAbs = 0;
  bool aborted = false;
  bool endstopHit = false;

  for (long i = 0; i < stepsTotal; i++) {
    if (stopRequested) { aborted = true; break; }
    if (bothEndstopsPressed()) { aborted = true; endstopHit = true; requestStop(); break; }

    if (!positive && endstopPressed(gCfg.pinEndBegin)) { posSteps = 0; aborted = true; endstopHit = true; break; }
    if ( positive && endstopPressed(gCfg.pinEndEnd))   { aborted = true; endstopHit = true; break; }

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

  // Publish endstop_reached event after motion has stopped.
  if (endstopHit) publishEvent("endstop_reached", posSteps);

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
  activeCommand = String("test:") + (toEnd ? "end" : "begin");
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
// MQTT event publishing (motion_started / endstop_reached)
// ============================================================

// Returns an ISO-8601 UTC timestamp string.
// Falls back to a millis()-based string when NTP is not yet synced.
static String isoTimestamp() {
  time_t now = time(nullptr);
  if (now < 1000000000UL) {
    char buf[24];
    snprintf(buf, sizeof(buf), "T+%lums", millis());
    return String(buf);
  }
  char buf[25];
  struct tm* t = gmtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", t);
  return String(buf);
}

// Maximum serialised byte length of one event payload.
// Breakdown: ~15 (keys+braces) + 20 (event value) + 30 (command) +
//            12 (steps) + 25 (timestamp) + margin = ~160 bytes.
static const size_t MQTT_EVENT_BUF_SIZE = 160;

// Publish a JSON event message to T_EVENT.
// Payload fields: event, command, steps, timestamp.
// Non-blocking: mqtt.publish() buffers the message; it is flushed
// during the next mqtt.loop() call (serviceBackground in the step loop).
static void publishEvent(const char* event, long steps) {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  doc["event"]     = event;
  doc["command"]   = activeCommand.length() ? activeCommand : String("unknown");
  doc["steps"]     = steps;
  doc["timestamp"] = isoTimestamp();
  char buf[MQTT_EVENT_BUF_SIZE];
  serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(T_EVENT.c_str(), buf);
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
  s.reserve(32000);

  s += htmlHeader(F("Config"));
  s += F("<h2>");
  s += controllerTitle(gCfg.axis);
  s += F(" - Config</h2>");

  if (bothEndstopsPressed()) {
    s += F("<div class='card err'>ERROR: Both endstops are PRESSED. Movement is disabled.</div>");
  }

  s += F("<div class='card'>"
         "<form method='POST' action='/config/save'>");

  // Save button at TOP
  s += F("<div class='row'><button type='submit' class='go'>&#128190; Save to flash</button></div>"
         "<p class='small'>After saving, device will reboot.</p>");

  s += F("<div class='row'><label>WiFi SSID</label><input name='ssid' required value='");
  s += htmlEscape(gCfg.wifiSsid);
  s += F("'></div>");

  s += F("<div class='row'><label>WiFi Password (also OTA password)</label><input name='pass' type='password' value='");
  s += htmlEscape(gCfg.wifiPass);
  s += F("'></div>");

  s += F("<div class='row'><label>Axis</label><select name='axis'>");
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

  // ---- Servo config (Horizontal mode only) ----
  if (gCfg.axis == AXIS_H) {
    s += F("<div class='card'><h3>Button Servos (Horizontal mode)</h3>"
           "<p class='small'>Positions are specified in degrees (0-180). "
           "Calibration (min/max µs) maps 0° and 180° to the physical pulse widths.</p>");

    // Top Button servo
    s += F("<h4>Top Button Servo</h4>");
    s += F("<div class='row'><label>Pin</label><select name='sv_top_pin'>");
    appendPinOptions(s, gCfg.servoTopPin);
    s += F("</select></div>");

    s += F("<div class='row'><label>Calib MIN (µs, = 0°)</label>"
           "<input name='sv_top_min_us' type='number' min='400' max='2600' step='10' value='");
    s += String(gCfg.servoTopMinUs);
    s += F("'></div>");
    s += F("<div class='row'><label>Calib MAX (µs, = 180°)</label>"
           "<input name='sv_top_max_us' type='number' min='400' max='2600' step='10' value='");
    s += String(gCfg.servoTopMaxUs);
    s += F("'></div>");

    s += F("<div class='row'><label>MIN idle (°)</label>"
           "<input name='sv_top_min_deg' type='number' min='0' max='180' step='1' value='");
    s += String(gCfg.servoTopMinDeg);
    s += F("'></div>");
    s += F("<div class='row'><label>APPROACH (°)</label>"
           "<input name='sv_top_approach_deg' type='number' min='0' max='180' step='1' value='");
    s += String(gCfg.servoTopApproachDeg);
    s += F("'></div>");
    s += F("<div class='row'><label>MAX press (°)</label>"
           "<input name='sv_top_max_deg' type='number' min='0' max='180' step='1' value='");
    s += String(gCfg.servoTopMaxDeg);
    s += F("'></div>");

    s += F("<div class='row'>"
           "<button type='button' onclick=\"doServo('/servo/top?pos=min')\">Top MIN</button>"
           "<button type='button' onclick=\"doServo('/servo/top?pos=approach')\">Top APPROACH</button>"
           "<button type='button' onclick=\"doServo('/servo/top?pos=max')\">Top MAX</button>"
           "</div>"
           "<div class='row'>"
           "<label>Custom angle (°):</label>"
           "<input id='topAngle' type='number' min='0' max='180' step='1' value='90' style='width:80px'>"
           "<button type='button' onclick=\"doServo('/servo/top?angle='+document.getElementById('topAngle').value)\">Top GO</button>"
           "</div>"
           "<div class='row'>"
           "<button type='button' onclick=\"doServo('/servo/top?clicks=1')\">Top Click 1</button>"
           "<button type='button' onclick=\"doServo('/servo/top?clicks=3')\">Top Click 3</button>"
           "<button type='button' onclick=\"doServo('/servo/top?clicks=4')\">Top Click 4</button>"
           "<button type='button' onclick=\"doServo('/servo/top?clicks=9')\">Top Click 9</button>"
           "<button type='button' onclick=\"doServo('/servo/top?clicks=13')\">Top Click 13</button>"
           "</div>");

    // Bottom Button servo
    s += F("<h4>Bottom Button Servo</h4>");
    s += F("<div class='row'><label>Pin</label><select name='sv_bot_pin'>");
    appendPinOptions(s, gCfg.servoBotPin);
    s += F("</select></div>");

    s += F("<div class='row'><label>Calib MIN (µs, = 0°)</label>"
           "<input name='sv_bot_min_us' type='number' min='400' max='2600' step='10' value='");
    s += String(gCfg.servoBotMinUs);
    s += F("'></div>");
    s += F("<div class='row'><label>Calib MAX (µs, = 180°)</label>"
           "<input name='sv_bot_max_us' type='number' min='400' max='2600' step='10' value='");
    s += String(gCfg.servoBotMaxUs);
    s += F("'></div>");

    s += F("<div class='row'><label>MIN idle (°)</label>"
           "<input name='sv_bot_min_deg' type='number' min='0' max='180' step='1' value='");
    s += String(gCfg.servoBotMinDeg);
    s += F("'></div>");
    s += F("<div class='row'><label>APPROACH (°)</label>"
           "<input name='sv_bot_approach_deg' type='number' min='0' max='180' step='1' value='");
    s += String(gCfg.servoBotApproachDeg);
    s += F("'></div>");
    s += F("<div class='row'><label>MAX press (°)</label>"
           "<input name='sv_bot_max_deg' type='number' min='0' max='180' step='1' value='");
    s += String(gCfg.servoBotMaxDeg);
    s += F("'></div>");

    s += F("<div class='row'>"
           "<button type='button' onclick=\"doServo('/servo/bottom?pos=min')\">Bottom MIN</button>"
           "<button type='button' onclick=\"doServo('/servo/bottom?pos=approach')\">Bottom APPROACH</button>"
           "<button type='button' onclick=\"doServo('/servo/bottom?pos=max')\">Bottom MAX</button>"
           "</div>"
           "<div class='row'>"
           "<label>Custom angle (°):</label>"
           "<input id='botAngle' type='number' min='0' max='180' step='1' value='90' style='width:80px'>"
           "<button type='button' onclick=\"doServo('/servo/bottom?angle='+document.getElementById('botAngle').value)\">Bottom GO</button>"
           "</div>"
           "<div class='row'>"
           "<button type='button' onclick=\"doServo('/servo/bottom?clicks=1')\">Bottom Click 1</button>"
           "<button type='button' onclick=\"doServo('/servo/bottom?clicks=3')\">Bottom Click 3</button>"
           "<button type='button' onclick=\"doServo('/servo/bottom?clicks=4')\">Bottom Click 4</button>"
           "<button type='button' onclick=\"doServo('/servo/bottom?clicks=9')\">Bottom Click 9</button>"
           "<button type='button' onclick=\"doServo('/servo/bottom?clicks=13')\">Bottom Click 13</button>"
           "</div>");

    // Shared click timing
    s += F("<h4>Shared Click Timing</h4>");
    s += F("<div class='row'><label>approach_ms</label>"
           "<input name='sv_approach_ms' type='number' min='0' max='5000' step='10' value='");
    s += String(gCfg.clickApproachMs);
    s += F("'></div>");
    s += F("<div class='row'><label>press_ms</label>"
           "<input name='sv_press_ms' type='number' min='0' max='5000' step='10' value='");
    s += String(gCfg.clickPressMs);
    s += F("'></div>");
    s += F("<div class='row'><label>between_ms</label>"
           "<input name='sv_between_ms' type='number' min='0' max='5000' step='10' value='");
    s += String(gCfg.clickBetweenMs);
    s += F("'></div>");
    s += F("<div class='row'><label>return_ms</label>"
           "<input name='sv_return_ms' type='number' min='0' max='5000' step='10' value='");
    s += String(gCfg.clickReturnMs);
    s += F("'></div>");

    s += F("<pre id='svout' style='margin-top:10px'>Servo output...</pre>"
           "</div>");
  }

  s += F("<div class='row'><button type='submit' class='go'>&#128190; Save to flash</button></div>"
         "<p class='small'>After saving, device will reboot.</p>"
         "</form></div>");

  // ---- Backup / Restore ----
  s += F("<div class='card'><h3>Backup / Restore config.json</h3>"
         "<div class='row'>"
         "<button type='button' onclick=\"window.location.href='/config/download'\">&#11015; Download config.json</button>"
         "</div>"
         "<form method='POST' action='/config/upload' enctype='multipart/form-data'>"
         "<div class='row'>"
         "<input type='file' name='config' accept='.json'>"
         "<button type='submit' class='stop'>&#11014; Upload &amp; Restore (overwrites config!)</button>"
         "</div>"
         "<p class='small'>Uploading will overwrite config.json and reboot the device. "
         "Only valid JSON files are accepted.</p>"
         "</form></div>");

  s += F("<script>"
         "async function doTest(url){"
         "  const t=document.getElementById('ptest');"
         "  t.textContent='Running '+url+' ...';"
         "  try{ t.textContent=await (await fetch(url)).text(); }catch(e){ t.textContent='ERR '+e; }"
         "}"
         "async function doServo(url){"
         "  const t=document.getElementById('svout');"
         "  if(!t) return;"
         "  t.textContent='Running '+url+' ...';"
         "  try{ t.textContent=await (await fetch(url)).text(); }catch(e){ t.textContent='ERR '+e; }"
         "}"
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

  // Servo fields (always saved; relevant only in H mode)
  if (server.hasArg("sv_top_pin"))         gCfg.servoTopPin        = (uint8_t)server.arg("sv_top_pin").toInt();
  if (server.hasArg("sv_top_min_us"))      gCfg.servoTopMinUs      = clampServoUs(server.arg("sv_top_min_us").toInt());
  if (server.hasArg("sv_top_max_us"))      gCfg.servoTopMaxUs      = clampServoUs(server.arg("sv_top_max_us").toInt());
  if (server.hasArg("sv_top_min_deg"))     gCfg.servoTopMinDeg     = (uint8_t)constrain(server.arg("sv_top_min_deg").toInt(), 0, 180);
  if (server.hasArg("sv_top_approach_deg"))gCfg.servoTopApproachDeg= (uint8_t)constrain(server.arg("sv_top_approach_deg").toInt(), 0, 180);
  if (server.hasArg("sv_top_max_deg"))     gCfg.servoTopMaxDeg     = (uint8_t)constrain(server.arg("sv_top_max_deg").toInt(), 0, 180);

  if (server.hasArg("sv_bot_pin"))         gCfg.servoBotPin        = (uint8_t)server.arg("sv_bot_pin").toInt();
  if (server.hasArg("sv_bot_min_us"))      gCfg.servoBotMinUs      = clampServoUs(server.arg("sv_bot_min_us").toInt());
  if (server.hasArg("sv_bot_max_us"))      gCfg.servoBotMaxUs      = clampServoUs(server.arg("sv_bot_max_us").toInt());
  if (server.hasArg("sv_bot_min_deg"))     gCfg.servoBotMinDeg     = (uint8_t)constrain(server.arg("sv_bot_min_deg").toInt(), 0, 180);
  if (server.hasArg("sv_bot_approach_deg"))gCfg.servoBotApproachDeg= (uint8_t)constrain(server.arg("sv_bot_approach_deg").toInt(), 0, 180);
  if (server.hasArg("sv_bot_max_deg"))     gCfg.servoBotMaxDeg     = (uint8_t)constrain(server.arg("sv_bot_max_deg").toInt(), 0, 180);

  if (server.hasArg("sv_approach_ms")) gCfg.clickApproachMs = (uint16_t)server.arg("sv_approach_ms").toInt();
  if (server.hasArg("sv_press_ms"))    gCfg.clickPressMs    = (uint16_t)server.arg("sv_press_ms").toInt();
  if (server.hasArg("sv_between_ms"))  gCfg.clickBetweenMs  = (uint16_t)server.arg("sv_between_ms").toInt();
  if (server.hasArg("sv_return_ms"))   gCfg.clickReturnMs   = (uint16_t)server.arg("sv_return_ms").toInt();

  // Validate degree order: min <= approach <= max for each servo
  if (gCfg.servoTopApproachDeg < gCfg.servoTopMinDeg) gCfg.servoTopApproachDeg = gCfg.servoTopMinDeg;
  if (gCfg.servoTopMaxDeg      < gCfg.servoTopApproachDeg) gCfg.servoTopMaxDeg = gCfg.servoTopApproachDeg;
  if (gCfg.servoBotApproachDeg < gCfg.servoBotMinDeg) gCfg.servoBotApproachDeg = gCfg.servoBotMinDeg;
  if (gCfg.servoBotMaxDeg      < gCfg.servoBotApproachDeg) gCfg.servoBotMaxDeg = gCfg.servoBotApproachDeg;

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
  page += F("<p>Rebooting in <span id='sec'>5</span> seconds…</p>");
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

void handleStop() {
  testModeStop();
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText());
}

void handleHome() {
  if (!server.hasArg("dir")) { server.send(400, "text/plain; charset=utf-8", "Missing dir"); return; }
  String dir = server.arg("dir");
  activeCommand = "home:" + dir;
  bool ok = false;
  if (dir == "begin") ok = goBegin();
  else if (dir == "end") ok = goEnd();
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText() + (ok ? "\nhome=OK" : "\nhome=FAILED"));
}

void handleMove() {
  if (!server.hasArg("steps")) { server.send(400, "text/plain; charset=utf-8", "Missing steps"); return; }
  long steps = server.arg("steps").toInt();
  activeCommand = "move:" + String(steps);
  bool ok = moveStepsAccel(steps);
  publishState();
  server.send(200, "text/plain; charset=utf-8", statusText() + (ok ? "\nmove=OK" : "\nmove=STOP_OR_ENDSTOP"));
}

void handleGoto() {
  if (!server.hasArg("i")) { server.send(400, "text/plain; charset=utf-8", "Missing i"); return; }
  int i = server.arg("i").toInt();
  activeCommand = "goto:" + String(i);
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

// ============================================================
// Config download / upload
// ============================================================
void handleConfigDownload() {
  if (!LittleFS.exists(CONFIG_PATH)) {
    server.send(404, "text/plain; charset=utf-8", "config.json not found");
    return;
  }
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    server.send(500, "text/plain; charset=utf-8", "Cannot open config.json");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"config.json\"");
  server.streamFile(f, "application/json");
  f.close();
}

static File gUploadFile;

void handleConfigUploadDone() {
  if (gUploadFile) { gUploadFile.close(); gUploadFile = File(); }

  // Validate JSON
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    server.send(500, "text/plain; charset=utf-8", "Upload failed: cannot read back file");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    LittleFS.remove(CONFIG_PATH);
    server.send(400, "text/plain; charset=utf-8",
                String("Upload rejected: invalid JSON (") + err.c_str() + ")");
    return;
  }

  String page;
  page.reserve(800);
  page += F("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Restored</title>"
            "<style>body{font-family:system-ui,Arial;margin:16px}</style>"
            "</head><body>");
  page += F("<h3>Config restored successfully.</h3>");
  page += F("<p>Rebooting in <span id='sec'>3</span> seconds...</p>");
  page += F("<script>"
            "let n=3;"
            "const el=document.getElementById('sec');"
            "const t=setInterval(()=>{n--; el.textContent=String(n); if(n<=0){clearInterval(t);}},1000);"
            "setTimeout(()=>{ window.location.href='/'; }, 3000);"
            "</script>");
  page += F("</body></html>");
  server.send(200, "text/html; charset=utf-8", page);
  delay(500);  // allow TCP buffer to flush before reboot
  ESP.restart();
}

void handleConfigUploadChunk() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    gUploadFile = LittleFS.open(CONFIG_PATH, "w");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (gUploadFile) gUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (gUploadFile) { gUploadFile.close(); gUploadFile = File(); }
  }
}

// ============================================================
// Servo endpoints
// ============================================================
static void handleServoGeneric(Servo& sv, uint8_t pin,
                               uint8_t minDeg, uint8_t approachDeg, uint8_t maxDeg,
                               uint16_t minUs, uint16_t maxUs,
                               const char* label) {
  if (gCfg.axis != AXIS_H) {
    server.send(409, "text/plain; charset=utf-8", "Servo available only in Horizontal mode");
    return;
  }
  // Direct angle control: ?angle=<0-180>
  if (server.hasArg("angle")) {
    int deg = server.arg("angle").toInt();
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    servoWriteDeg(sv, pin, (uint8_t)deg, minUs, maxUs);
    server.send(200, "text/plain; charset=utf-8",
                String("OK servo=") + label + " angle=" + deg + " deg"
                + " (" + degToUs((uint8_t)deg, minUs, maxUs) + " us)");
    return;
  }
  if (server.hasArg("pos")) {
    String pos = server.arg("pos");
    if (!servoMoveToPos(sv, pin, minDeg, approachDeg, maxDeg, minUs, maxUs, pos)) {
      server.send(400, "text/plain; charset=utf-8", "pos must be min|approach|max");
      return;
    }
    server.send(200, "text/plain; charset=utf-8",
                String("OK servo=") + label + " pos=" + pos);
    return;
  }
  if (server.hasArg("clicks")) {
    int n = server.arg("clicks").toInt();
    if (n < 1) n = 1;
    if (n > SERVO_CLICKS_MAX) n = SERVO_CLICKS_MAX;
    servoClick(sv, pin, minDeg, approachDeg, maxDeg, minUs, maxUs, (uint8_t)n);
    server.send(200, "text/plain; charset=utf-8",
                String("OK servo=") + label + " clicks=" + n);
    return;
  }
  server.send(400, "text/plain; charset=utf-8", "Missing arg: angle, pos, or clicks");
}

void handleServoTop() {
  handleServoGeneric(gServoTop, gCfg.servoTopPin,
                     gCfg.servoTopMinDeg, gCfg.servoTopApproachDeg, gCfg.servoTopMaxDeg,
                     gCfg.servoTopMinUs, gCfg.servoTopMaxUs,
                     "top");
}

void handleServoBottom() {
  handleServoGeneric(gServoBot, gCfg.servoBotPin,
                     gCfg.servoBotMinDeg, gCfg.servoBotApproachDeg, gCfg.servoBotMaxDeg,
                     gCfg.servoBotMinUs, gCfg.servoBotMaxUs,
                     "bottom");
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
  else if (t == T_MOVE) { activeCommand = "mqtt-move:" + msg; moveStepsAccel(ival); }
  else if (t == T_GOTO) { activeCommand = "mqtt-goto:" + msg; gotoIndex((uint8_t)ival); }

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

  servoInit();

  ensureWiFiConnected();
  // Sync system time via SNTP for ISO-8601 timestamps in event payloads.
  // Sync happens in the background; isoTimestamp() falls back to uptime
  // until the first successful sync.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/config", handleConfig);
  server.on("/config/save", HTTP_POST, handleConfigSave);
  server.on("/config/download", handleConfigDownload);
  server.on("/config/upload", HTTP_POST, handleConfigUploadDone, handleConfigUploadChunk);
  server.on("/stop", handleStop);
  server.on("/home", handleHome);
  server.on("/move", handleMove);
  server.on("/goto", handleGoto);
  server.on("/test", handleTest);

  // pin test
  server.on("/pin/en", handlePinEn);
  server.on("/pin/dir", handlePinDir);
  server.on("/pin/step", handlePinStep);

  // servo endpoints
  server.on("/servo/top", handleServoTop);
  server.on("/servo/bottom", handleServoBottom);

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
