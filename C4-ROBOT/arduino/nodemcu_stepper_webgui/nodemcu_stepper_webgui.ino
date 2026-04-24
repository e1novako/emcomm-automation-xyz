// nodemcu_stepper_webgui.ino
// ESP8266 (NodeMCU) stepper motor web GUI  –  firmware v0.9
//
// Features:
//   Home page  : motor status, axis mode, pin info, servo click buttons (horizontal mode),
//                backup/restore config.json
//   Config page: all settings, Save button at top AND bottom
//   Config     : persisted in LittleFS as /config.json
//   Backup     : GET  /backup             → download config.json
//   Restore    : POST /restore            → upload config.json (multipart)
//   Motor pins : EN / STEP / DIR with label, color, logic level (active H/L)
//   Pin test   : GET /test/en|step|dir?state=0|1
//   LED        : NodeMCU blue LED (GPIO2) ON while servo is clicking / motor moving
//   Axis modes : Vertical (default) / Horizontal
//   Horizontal : Top & Bottom button servos (pin + min/max µs)
//                Click buttons: 1, 3, 4, 9, 13 times
//   Servo HTTP : GET /servo/top?clicks=N   (N = 1..50)
//                GET /servo/bottom?clicks=N
//                GET /servo/top?pos=min|max  (test position)
//                GET /servo/bottom?pos=min|max
//
// Build: Arduino IDE + ESP8266 core + ArduinoJson + Servo (built-in)
// Board : NodeMCU 1.0 (ESP-12E Module)  –  Flash: 4 MB (FS: 1 MB LittleFS)

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Servo.h>

// ─────────────────────────────────────────────────────────────────────────────
// Pins
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t LED_PIN = 2;  // NodeMCU built-in blue LED (active LOW)

// ─────────────────────────────────────────────────────────────────────────────
// Axis modes
// ─────────────────────────────────────────────────────────────────────────────
enum AxisMode : uint8_t { AXIS_V = 0, AXIS_H = 1 };

// ─────────────────────────────────────────────────────────────────────────────
// Configuration struct  (all fields have safe defaults)
// ─────────────────────────────────────────────────────────────────────────────
struct AppConfig {
    // WiFi AP credentials
    char ssid[32]     = "NodeMCU-Stepper";
    char password[32] = "12345678";

    // Stepper pins
    uint8_t enPin   = 4;   // D2 / GPIO4
    uint8_t stepPin = 5;   // D1 / GPIO5
    uint8_t dirPin  = 0;   // D3 / GPIO0

    // Pin labels shown in the UI
    char enLabel[16]   = "EN";
    char stepLabel[16] = "STEP";
    char dirLabel[16]  = "DIR";

    // Pin badge colours (CSS hex strings)
    char enColor[8]   = "#e74c3c";
    char stepColor[8] = "#2ecc71";
    char dirColor[8]  = "#3498db";

    // Logic levels: true = active HIGH, false = active LOW
    bool enActiveHigh   = false;  // most stepper drivers: EN active LOW = enabled
    bool stepActiveHigh = true;
    bool dirActiveHigh  = true;

    // Axis mode
    AxisMode axis = AXIS_V;

    // Horizontal mode – Top button servo (microseconds)
    uint8_t  topServoPin   = 14;    // D5 / GPIO14
    uint16_t topServoMinUs = 600;   // released
    uint16_t topServoMaxUs = 2400;  // pressed

    // Horizontal mode – Bottom button servo (microseconds)
    uint8_t  bottomServoPin   = 12;   // D6 / GPIO12
    uint16_t bottomServoMinUs = 600;
    uint16_t bottomServoMaxUs = 2400;
};

AppConfig gCfg;

// ─────────────────────────────────────────────────────────────────────────────
// Global objects
// ─────────────────────────────────────────────────────────────────────────────
ESP8266WebServer server(80);
bool             isMoving      = false;
bool             stopRequested = false;

Servo  gTopServo;
Servo  gBottomServo;
bool   gServosAttached = false;

static const uint16_t SERVO_MIN_US_LIMIT      = 400;
static const uint16_t SERVO_MAX_US_LIMIT      = 2600;
static const uint16_t SERVO_PRESS_MS          = 250;
static const uint16_t SERVO_RELEASE_MS        = 250;
static const uint16_t SERVO_BETWEEN_CLICKS_MS = 250;

// ─────────────────────────────────────────────────────────────────────────────
// JSON helper utilities
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t jsonGetU8(JsonVariant v, uint8_t def) {
    return v.isNull() ? def : (uint8_t)v.as<int>();
}
static uint16_t jsonGetU16(JsonVariant v, uint16_t def) {
    return v.isNull() ? def : (uint16_t)v.as<int>();
}
static bool jsonGetBool(JsonVariant v, bool def) {
    return v.isNull() ? def : v.as<bool>();
}
static void jsonGetStr(JsonVariant v, char* buf, size_t sz, const char* def) {
    const char* src = v.isNull() ? def : v.as<const char*>();
    strncpy(buf, src, sz - 1);
    buf[sz - 1] = '\0';
}

// ─────────────────────────────────────────────────────────────────────────────
// Load / Save config  (LittleFS  /config.json)
// ─────────────────────────────────────────────────────────────────────────────
static bool loadConfig() {
    if (!LittleFS.exists("/config.json")) return false;
    File f = LittleFS.open("/config.json", "r");
    if (!f) return false;

    DynamicJsonDocument doc(2048);
    if (deserializeJson(doc, f)) { f.close(); return false; }
    f.close();

    jsonGetStr(doc["ssid"],     gCfg.ssid,     sizeof(gCfg.ssid),     "NodeMCU-Stepper");
    jsonGetStr(doc["password"], gCfg.password, sizeof(gCfg.password), "12345678");

    gCfg.enPin   = jsonGetU8(doc["enPin"],   4);
    gCfg.stepPin = jsonGetU8(doc["stepPin"], 5);
    gCfg.dirPin  = jsonGetU8(doc["dirPin"],  0);

    jsonGetStr(doc["enLabel"],   gCfg.enLabel,   sizeof(gCfg.enLabel),   "EN");
    jsonGetStr(doc["stepLabel"], gCfg.stepLabel, sizeof(gCfg.stepLabel), "STEP");
    jsonGetStr(doc["dirLabel"],  gCfg.dirLabel,  sizeof(gCfg.dirLabel),  "DIR");

    jsonGetStr(doc["enColor"],   gCfg.enColor,   sizeof(gCfg.enColor),   "#e74c3c");
    jsonGetStr(doc["stepColor"], gCfg.stepColor, sizeof(gCfg.stepColor), "#2ecc71");
    jsonGetStr(doc["dirColor"],  gCfg.dirColor,  sizeof(gCfg.dirColor),  "#3498db");

    gCfg.enActiveHigh   = jsonGetBool(doc["enActiveHigh"],   false);
    gCfg.stepActiveHigh = jsonGetBool(doc["stepActiveHigh"], true);
    gCfg.dirActiveHigh  = jsonGetBool(doc["dirActiveHigh"],  true);

    gCfg.axis = (AxisMode)jsonGetU8(doc["axis"], (uint8_t)AXIS_V);

    gCfg.topServoPin   = jsonGetU8(doc["servos"]["top"]["pin"],    14);
    gCfg.topServoMinUs = jsonGetU16(doc["servos"]["top"]["min_us"], 600);
    gCfg.topServoMaxUs = jsonGetU16(doc["servos"]["top"]["max_us"], 2400);

    gCfg.bottomServoPin   = jsonGetU8(doc["servos"]["bottom"]["pin"],    12);
    gCfg.bottomServoMinUs = jsonGetU16(doc["servos"]["bottom"]["min_us"], 600);
    gCfg.bottomServoMaxUs = jsonGetU16(doc["servos"]["bottom"]["max_us"], 2400);

    return true;
}

static bool saveConfig() {
    DynamicJsonDocument doc(2048);

    doc["ssid"]     = gCfg.ssid;
    doc["password"] = gCfg.password;

    doc["enPin"]   = gCfg.enPin;
    doc["stepPin"] = gCfg.stepPin;
    doc["dirPin"]  = gCfg.dirPin;

    doc["enLabel"]   = gCfg.enLabel;
    doc["stepLabel"] = gCfg.stepLabel;
    doc["dirLabel"]  = gCfg.dirLabel;

    doc["enColor"]   = gCfg.enColor;
    doc["stepColor"] = gCfg.stepColor;
    doc["dirColor"]  = gCfg.dirColor;

    doc["enActiveHigh"]   = gCfg.enActiveHigh;
    doc["stepActiveHigh"] = gCfg.stepActiveHigh;
    doc["dirActiveHigh"]  = gCfg.dirActiveHigh;

    doc["axis"] = (uint8_t)gCfg.axis;

    doc["servos"]["top"]["pin"]    = gCfg.topServoPin;
    doc["servos"]["top"]["min_us"] = gCfg.topServoMinUs;
    doc["servos"]["top"]["max_us"] = gCfg.topServoMaxUs;

    doc["servos"]["bottom"]["pin"]    = gCfg.bottomServoPin;
    doc["servos"]["bottom"]["min_us"] = gCfg.bottomServoMinUs;
    doc["servos"]["bottom"]["max_us"] = gCfg.bottomServoMaxUs;

    File f = LittleFS.open("/config.json", "w");
    if (!f) return false;
    serializeJson(doc, f);
    f.close();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// LED  (active LOW on NodeMCU)
// ─────────────────────────────────────────────────────────────────────────────
static void setLed(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor pin helpers
// ─────────────────────────────────────────────────────────────────────────────
static void applyPinModes() {
    pinMode(gCfg.enPin,   OUTPUT);
    pinMode(gCfg.stepPin, OUTPUT);
    pinMode(gCfg.dirPin,  OUTPUT);
    // Disable motor by default
    digitalWrite(gCfg.enPin,   gCfg.enActiveHigh   ? LOW  : HIGH);
    digitalWrite(gCfg.stepPin, gCfg.stepActiveHigh ? LOW  : HIGH);
    digitalWrite(gCfg.dirPin,  LOW);
}

static inline void setEn(bool active) {
    digitalWrite(gCfg.enPin, (active == gCfg.enActiveHigh) ? HIGH : LOW);
}
static inline void setDir(bool cw) {
    digitalWrite(gCfg.dirPin, (cw == gCfg.dirActiveHigh) ? HIGH : LOW);
}
static inline void pulseStep() {
    bool h = gCfg.stepActiveHigh;
    digitalWrite(gCfg.stepPin, h ? HIGH : LOW);
    delayMicroseconds(10);
    digitalWrite(gCfg.stepPin, h ? LOW : HIGH);
    delayMicroseconds(10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Servo helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline uint16_t clampUs(uint16_t us) {
    if (us < SERVO_MIN_US_LIMIT) return SERVO_MIN_US_LIMIT;
    if (us > SERVO_MAX_US_LIMIT) return SERVO_MAX_US_LIMIT;
    return us;
}

static void ensureServosAttached() {
    if (gCfg.axis != AXIS_H || gServosAttached) return;
    gTopServo.attach(gCfg.topServoPin, SERVO_MIN_US_LIMIT, SERVO_MAX_US_LIMIT);
    gBottomServo.attach(gCfg.bottomServoPin, SERVO_MIN_US_LIMIT, SERVO_MAX_US_LIMIT);
    gTopServo.writeMicroseconds(clampUs(gCfg.topServoMinUs));
    gBottomServo.writeMicroseconds(clampUs(gCfg.bottomServoMinUs));
    gServosAttached = true;
}

static void detachServos() {
    if (!gServosAttached) return;
    gTopServo.detach();
    gBottomServo.detach();
    gServosAttached = false;
}

static void serviceBackground() {
    server.handleClient();
}

static bool waitMsWithService(uint32_t ms) {
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {
        serviceBackground();
        if (stopRequested) return false;
        delay(5);
    }
    return true;
}

// Perform N click cycles on a servo (press → release).
static bool clickServo(bool top, uint8_t clicks) {
    if (gCfg.axis != AXIS_H) return false;
    if (clicks < 1)  clicks = 1;
    if (clicks > 50) clicks = 50;
    ensureServosAttached();

    uint16_t minUs = top ? gCfg.topServoMinUs : gCfg.bottomServoMinUs;
    uint16_t maxUs = top ? gCfg.topServoMaxUs : gCfg.bottomServoMaxUs;

    for (uint8_t i = 0; i < clicks; i++) {
        if (stopRequested) return false;

        // Press
        if (top) gTopServo.writeMicroseconds(clampUs(maxUs));
        else     gBottomServo.writeMicroseconds(clampUs(maxUs));
        if (!waitMsWithService(SERVO_PRESS_MS)) return false;

        // Release
        if (top) gTopServo.writeMicroseconds(clampUs(minUs));
        else     gBottomServo.writeMicroseconds(clampUs(minUs));
        if (!waitMsWithService(SERVO_RELEASE_MS)) return false;

        // Pause between clicks
        if (i + 1 < clicks) {
            if (!waitMsWithService(SERVO_BETWEEN_CLICKS_MS)) return false;
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// HTML generation helpers
// ─────────────────────────────────────────────────────────────────────────────
static String pinName(uint8_t pin) {
    switch (pin) {
        case 16: return F("D0/GPIO16");
        case  5: return F("D1/GPIO5");
        case  4: return F("D2/GPIO4");
        case  0: return F("D3/GPIO0");
        case  2: return F("D4/GPIO2");
        case 14: return F("D5/GPIO14");
        case 12: return F("D6/GPIO12");
        case 13: return F("D7/GPIO13");
        case 15: return F("D8/GPIO15");
        case  3: return F("RX/GPIO3");
        case  1: return F("TX/GPIO1");
        default: return "GPIO" + String(pin);
    }
}

static void appendPinOptions(String& s, uint8_t selected) {
    const uint8_t pins[] = {16, 5, 4, 0, 2, 14, 12, 13, 15, 3, 1};
    for (uint8_t pin : pins) {
        s += "<option value='" + String(pin) + "'";
        if (pin == selected) s += F(" selected");
        s += ">" + pinName(pin) + "</option>";
    }
}

static String htmlHeader(const String& title) {
    String s;
    s.reserve(1400);
    s += F("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
    s += "<title>" + title + "</title>";
    s += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    s += F("<style>"
           "*{box-sizing:border-box;margin:0;padding:0;}"
           "body{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;padding:10px;}"
           "h1{color:#00b4d8;margin:10px 0;}"
           "h3{color:#90e0ef;margin:8px 0;}"
           "a{color:#00b4d8;text-decoration:none;}"
           "nav{margin-bottom:16px;}nav a{margin-right:14px;font-size:1.1em;}"
           ".card{background:#16213e;border-radius:8px;padding:14px;margin-bottom:12px;}"
           ".row{display:flex;align-items:center;flex-wrap:wrap;margin:6px 0;gap:8px;}"
           "label{min-width:200px;font-size:.9em;color:#aaa;}"
           "input,select{background:#0f3460;color:#e0e0e0;border:1px solid #444;"
           "border-radius:4px;padding:4px 8px;font-size:.9em;width:100%;max-width:220px;}"
           "button,.btn{background:#00b4d8;color:#000;border:none;border-radius:4px;"
           "padding:6px 14px;cursor:pointer;font-size:.9em;margin:2px;}"
           "button:hover,.btn:hover{background:#90e0ef;}"
           ".btn-sm{padding:4px 10px;font-size:.82em;}"
           ".badge{display:inline-block;padding:2px 8px;border-radius:12px;"
           "font-size:.8em;color:#fff;margin:2px;}"
           ".warn{color:#f39c12;}.ok{color:#2ecc71;}"
           "</style></head><body>");
    s += F("<nav><a href='/'>&#127968; Home</a><a href='/config'>&#9881; Config</a></nav>");
    s += "<h1>" + title + "</h1>";
    return s;
}

static String htmlFooter() {
    return F("</body></html>");
}

// ─────────────────────────────────────────────────────────────────────────────
// Home page
// ─────────────────────────────────────────────────────────────────────────────
static String htmlHome() {
    String s = htmlHeader("NodeMCU Stepper WebGUI");

    // Status card
    s += F("<div class='card'><h3>Status</h3>");
    s += "<div class='row'><label>Moving:</label><span class='"
         + String(isMoving ? "warn" : "ok") + "'>"
         + String(isMoving ? "YES" : "NO") + "</span></div>";
    s += "<div class='row'><label>Axis mode:</label><span>"
         + String(gCfg.axis == AXIS_V ? "Vertical" : "Horizontal") + "</span></div>";
    s += F("</div>");

    // Motor pins card
    s += F("<div class='card'><h3>Motor Pins</h3>");
    s += "<div class='row'>"
         "<span class='badge' style='background:" + String(gCfg.enColor) + "'>"
         + String(gCfg.enLabel) + "</span>"
         "<span>" + pinName(gCfg.enPin) + " &mdash; Active "
         + String(gCfg.enActiveHigh ? "HIGH" : "LOW") + "</span></div>";
    s += "<div class='row'>"
         "<span class='badge' style='background:" + String(gCfg.stepColor) + "'>"
         + String(gCfg.stepLabel) + "</span>"
         "<span>" + pinName(gCfg.stepPin) + " &mdash; Active "
         + String(gCfg.stepActiveHigh ? "HIGH" : "LOW") + "</span></div>";
    s += "<div class='row'>"
         "<span class='badge' style='background:" + String(gCfg.dirColor) + "'>"
         + String(gCfg.dirLabel) + "</span>"
         "<span>" + pinName(gCfg.dirPin) + " &mdash; Active "
         + String(gCfg.dirActiveHigh ? "HIGH" : "LOW") + "</span></div>";
    s += F("</div>");

    // Pin test card
    s += F("<div class='card'><h3>Pin Test</h3><div class='row'>");
    s += F("<a class='btn btn-sm' href='/test/en?state=1'>EN HIGH</a>");
    s += F("<a class='btn btn-sm' href='/test/en?state=0'>EN LOW</a>");
    s += F("<a class='btn btn-sm' href='/test/step?state=1'>STEP HIGH</a>");
    s += F("<a class='btn btn-sm' href='/test/step?state=0'>STEP LOW</a>");
    s += F("<a class='btn btn-sm' href='/test/dir?state=1'>DIR HIGH</a>");
    s += F("<a class='btn btn-sm' href='/test/dir?state=0'>DIR LOW</a>");
    s += F("</div></div>");

    // Horizontal-mode servo card
    if (gCfg.axis == AXIS_H) {
        const uint8_t clickCounts[] = {1, 3, 4, 9, 13};
        s += F("<div class='card'><h3>Horizontal Mode &mdash; Servo Clicks</h3>");

        s += F("<div class='row'><strong>Top Button</strong></div><div class='row'>");
        for (uint8_t c : clickCounts) {
            s += "<a class='btn btn-sm' href='/servo/top?clicks=" + String(c) + "'>"
                 + String(c) + "&#215;</a>";
        }
        s += F("</div>");

        s += F("<div class='row'><strong>Bottom Button</strong></div><div class='row'>");
        for (uint8_t c : clickCounts) {
            s += "<a class='btn btn-sm' href='/servo/bottom?clicks=" + String(c) + "'>"
                 + String(c) + "&#215;</a>";
        }
        s += F("</div>");

        s += F("<div class='row'>"
               "<a class='btn btn-sm' href='/servo/top?pos=min'>Top MIN</a>"
               "<a class='btn btn-sm' href='/servo/top?pos=max'>Top MAX</a>"
               "<a class='btn btn-sm' href='/servo/bottom?pos=min'>Bottom MIN</a>"
               "<a class='btn btn-sm' href='/servo/bottom?pos=max'>Bottom MAX</a>"
               "</div>");
        s += F("</div>");
    }

    // Backup / Restore card
    s += F("<div class='card'><h3>Config Backup &amp; Restore</h3>");
    s += F("<div class='row'>"
           "<a class='btn' href='/backup'>&#8595; Download config.json</a>"
           "</div>");
    s += F("<div class='row'>"
           "<form action='/restore' method='POST' enctype='multipart/form-data'>"
           "<input type='file' name='config' accept='.json'>&nbsp;"
           "<button type='submit'>&#8593; Upload &amp; Restore</button>"
           "</form></div>");
    s += F("</div>");

    s += htmlFooter();
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Config page
// ─────────────────────────────────────────────────────────────────────────────
static String htmlConfig() {
    String s = htmlHeader("Configuration");
    s += F("<form action='/save' method='POST'>");

    // ── Save button TOP ──────────────────────────────────────────────────────
    s += F("<div class='card'>"
           "<button type='submit'>&#128190; Save Configuration</button>"
           "</div>");

    // ── WiFi / AP ────────────────────────────────────────────────────────────
    s += F("<div class='card'><h3>WiFi / AP</h3>");
    s += F("<div class='row'><label>SSID</label>"
           "<input name='ssid' maxlength='31' value='");
    s += String(gCfg.ssid); s += F("'></div>");
    s += F("<div class='row'><label>Password</label>"
           "<input name='password' type='password' maxlength='31' value='");
    s += String(gCfg.password); s += F("'></div>");
    s += F("</div>");

    // ── Stepper pins ─────────────────────────────────────────────────────────
    s += F("<div class='card'><h3>Stepper Pins</h3>");

    // EN
    s += F("<div class='row'><label>EN pin</label><select name='en_pin'>");
    appendPinOptions(s, gCfg.enPin); s += F("</select></div>");
    s += F("<div class='row'><label>EN label</label>"
           "<input name='en_label' maxlength='15' value='");
    s += String(gCfg.enLabel); s += F("'></div>");
    s += F("<div class='row'><label>EN colour</label>"
           "<input name='en_color' type='color' value='");
    s += String(gCfg.enColor); s += F("'></div>");
    s += F("<div class='row'><label>EN active level</label>"
           "<select name='en_active'>"
           "<option value='0'");
    if (!gCfg.enActiveHigh) s += F(" selected");
    s += F(">Active LOW</option><option value='1'");
    if (gCfg.enActiveHigh) s += F(" selected");
    s += F(">Active HIGH</option></select></div>");

    // STEP
    s += F("<div class='row'><label>STEP pin</label><select name='step_pin'>");
    appendPinOptions(s, gCfg.stepPin); s += F("</select></div>");
    s += F("<div class='row'><label>STEP label</label>"
           "<input name='step_label' maxlength='15' value='");
    s += String(gCfg.stepLabel); s += F("'></div>");
    s += F("<div class='row'><label>STEP colour</label>"
           "<input name='step_color' type='color' value='");
    s += String(gCfg.stepColor); s += F("'></div>");
    s += F("<div class='row'><label>STEP active level</label>"
           "<select name='step_active'>"
           "<option value='0'");
    if (!gCfg.stepActiveHigh) s += F(" selected");
    s += F(">Active LOW</option><option value='1'");
    if (gCfg.stepActiveHigh) s += F(" selected");
    s += F(">Active HIGH</option></select></div>");

    // DIR
    s += F("<div class='row'><label>DIR pin</label><select name='dir_pin'>");
    appendPinOptions(s, gCfg.dirPin); s += F("</select></div>");
    s += F("<div class='row'><label>DIR label</label>"
           "<input name='dir_label' maxlength='15' value='");
    s += String(gCfg.dirLabel); s += F("'></div>");
    s += F("<div class='row'><label>DIR colour</label>"
           "<input name='dir_color' type='color' value='");
    s += String(gCfg.dirColor); s += F("'></div>");
    s += F("<div class='row'><label>DIR active level</label>"
           "<select name='dir_active'>"
           "<option value='0'");
    if (!gCfg.dirActiveHigh) s += F(" selected");
    s += F(">Active LOW</option><option value='1'");
    if (gCfg.dirActiveHigh) s += F(" selected");
    s += F(">Active HIGH</option></select></div>");

    s += F("</div>"); // end stepper card

    // ── Axis mode ────────────────────────────────────────────────────────────
    s += F("<div class='card'><h3>Axis Mode</h3>"
           "<div class='row'><label>Mode</label><select name='axis'>"
           "<option value='0'");
    if (gCfg.axis == AXIS_V) s += F(" selected");
    s += F(">Vertical</option><option value='1'");
    if (gCfg.axis == AXIS_H) s += F(" selected");
    s += F(">Horizontal</option></select></div></div>");

    // ── Horizontal mode – Button Servos ─────────────────────────────────────
    s += F("<div class='card'><h3>Horizontal Mode &mdash; Button Servos</h3>"
           "<p style='font-size:.8em;color:#aaa;margin-bottom:8px;'>"
           "Settings apply when Axis = Horizontal.</p>");

    s += F("<div class='row'><label>Top servo pin</label><select name='top_servo_pin'>");
    appendPinOptions(s, gCfg.topServoPin); s += F("</select></div>");
    s += F("<div class='row'><label>Top MIN &micro;s (released)</label>"
           "<input name='top_servo_min_us' type='number' min='400' max='2600' value='");
    s += String(gCfg.topServoMinUs); s += F("'></div>");
    s += F("<div class='row'><label>Top MAX &micro;s (pressed)</label>"
           "<input name='top_servo_max_us' type='number' min='400' max='2600' value='");
    s += String(gCfg.topServoMaxUs); s += F("'></div>");

    s += F("<div class='row'><label>Bottom servo pin</label><select name='bottom_servo_pin'>");
    appendPinOptions(s, gCfg.bottomServoPin); s += F("</select></div>");
    s += F("<div class='row'><label>Bottom MIN &micro;s (released)</label>"
           "<input name='bottom_servo_min_us' type='number' min='400' max='2600' value='");
    s += String(gCfg.bottomServoMinUs); s += F("'></div>");
    s += F("<div class='row'><label>Bottom MAX &micro;s (pressed)</label>"
           "<input name='bottom_servo_max_us' type='number' min='400' max='2600' value='");
    s += String(gCfg.bottomServoMaxUs); s += F("'></div>");

    s += F("</div>"); // end servo card

    // ── Save button BOTTOM ───────────────────────────────────────────────────
    s += F("<div class='card'>"
           "<button type='submit'>&#128190; Save Configuration</button>"
           "</div>");

    s += F("</form>");
    s += htmlFooter();
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP handlers
// ─────────────────────────────────────────────────────────────────────────────
static void handleRoot() {
    server.send(200, "text/html", htmlHome());
}

static void handleConfig() {
    server.send(200, "text/html", htmlConfig());
}

static void handleSave() {
    auto arg = [&](const char* name) -> String {
        return server.hasArg(name) ? server.arg(name) : "";
    };

    if (server.hasArg("ssid"))
        strncpy(gCfg.ssid, arg("ssid").c_str(), sizeof(gCfg.ssid) - 1);
    if (server.hasArg("password"))
        strncpy(gCfg.password, arg("password").c_str(), sizeof(gCfg.password) - 1);

    if (server.hasArg("en_pin"))   gCfg.enPin   = (uint8_t)arg("en_pin").toInt();
    if (server.hasArg("step_pin")) gCfg.stepPin = (uint8_t)arg("step_pin").toInt();
    if (server.hasArg("dir_pin"))  gCfg.dirPin  = (uint8_t)arg("dir_pin").toInt();

    if (server.hasArg("en_label"))
        strncpy(gCfg.enLabel, arg("en_label").c_str(), sizeof(gCfg.enLabel) - 1);
    if (server.hasArg("step_label"))
        strncpy(gCfg.stepLabel, arg("step_label").c_str(), sizeof(gCfg.stepLabel) - 1);
    if (server.hasArg("dir_label"))
        strncpy(gCfg.dirLabel, arg("dir_label").c_str(), sizeof(gCfg.dirLabel) - 1);

    if (server.hasArg("en_color"))
        strncpy(gCfg.enColor, arg("en_color").c_str(), sizeof(gCfg.enColor) - 1);
    if (server.hasArg("step_color"))
        strncpy(gCfg.stepColor, arg("step_color").c_str(), sizeof(gCfg.stepColor) - 1);
    if (server.hasArg("dir_color"))
        strncpy(gCfg.dirColor, arg("dir_color").c_str(), sizeof(gCfg.dirColor) - 1);

    gCfg.enActiveHigh   = server.hasArg("en_active")   && arg("en_active")   == "1";
    gCfg.stepActiveHigh = server.hasArg("step_active") && arg("step_active") == "1";
    gCfg.dirActiveHigh  = server.hasArg("dir_active")  && arg("dir_active")  == "1";

    if (server.hasArg("axis"))
        gCfg.axis = (AxisMode)arg("axis").toInt();

    if (server.hasArg("top_servo_pin"))
        gCfg.topServoPin    = (uint8_t)arg("top_servo_pin").toInt();
    if (server.hasArg("top_servo_min_us"))
        gCfg.topServoMinUs  = (uint16_t)arg("top_servo_min_us").toInt();
    if (server.hasArg("top_servo_max_us"))
        gCfg.topServoMaxUs  = (uint16_t)arg("top_servo_max_us").toInt();

    if (server.hasArg("bottom_servo_pin"))
        gCfg.bottomServoPin    = (uint8_t)arg("bottom_servo_pin").toInt();
    if (server.hasArg("bottom_servo_min_us"))
        gCfg.bottomServoMinUs  = (uint16_t)arg("bottom_servo_min_us").toInt();
    if (server.hasArg("bottom_servo_max_us"))
        gCfg.bottomServoMaxUs  = (uint16_t)arg("bottom_servo_max_us").toInt();

    applyPinModes();
    detachServos();
    if (gCfg.axis == AXIS_H) ensureServosAttached();
    saveConfig();

    server.sendHeader("Location", "/config");
    server.send(302, "text/plain", "Saved");
}

static void handleBackup() {
    if (!LittleFS.exists("/config.json")) {
        server.send(404, "text/plain", "No config.json");
        return;
    }
    File f = LittleFS.open("/config.json", "r");
    if (!f) { server.send(500, "text/plain", "Open failed"); return; }
    server.sendHeader("Content-Disposition", "attachment; filename=\"config.json\"");
    server.streamFile(f, "application/json");
    f.close();
}

static File gUploadFile;

static void handleRestoreUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        if (LittleFS.exists("/config.json"))
            LittleFS.rename("/config.json", "/config.json.bak");
        gUploadFile = LittleFS.open("/config.json", "w");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (gUploadFile) gUploadFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (gUploadFile) gUploadFile.close();
        if (!loadConfig()) {
            LittleFS.remove("/config.json");
            if (LittleFS.exists("/config.json.bak"))
                LittleFS.rename("/config.json.bak", "/config.json");
            loadConfig(); // reload backup if available
        }
        applyPinModes();
        detachServos();
        if (gCfg.axis == AXIS_H) ensureServosAttached();
    }
}

static void handleRestoreFinish() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Config restored");
}

static void handleTestEn() {
    if (server.hasArg("state")) {
        int v = server.arg("state").toInt();
        digitalWrite(gCfg.enPin, v ? HIGH : LOW);
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "OK");
}

static void handleTestStep() {
    if (server.hasArg("state")) {
        int v = server.arg("state").toInt();
        digitalWrite(gCfg.stepPin, v ? HIGH : LOW);
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "OK");
}

static void handleTestDir() {
    if (server.hasArg("state")) {
        int v = server.arg("state").toInt();
        digitalWrite(gCfg.dirPin, v ? HIGH : LOW);
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "OK");
}

static void handleServoClick(bool top) {
    if (gCfg.axis != AXIS_H) {
        server.send(400, "text/plain", "Not in Horizontal mode");
        return;
    }

    // Test position mode: /servo/top?pos=min|max
    if (server.hasArg("pos")) {
        ensureServosAttached();
        String pos = server.arg("pos");
        if (top) {
            gTopServo.writeMicroseconds(
                clampUs(pos == "max" ? gCfg.topServoMaxUs : gCfg.topServoMinUs));
        } else {
            gBottomServo.writeMicroseconds(
                clampUs(pos == "max" ? gCfg.bottomServoMaxUs : gCfg.bottomServoMinUs));
        }
        server.send(200, "text/plain", "OK");
        return;
    }

    uint8_t clicks = 1;
    if (server.hasArg("clicks")) clicks = (uint8_t)server.arg("clicks").toInt();

    setLed(true);
    isMoving = true;
    bool ok = clickServo(top, clicks);
    isMoving = false;
    setLed(false);

    if (ok) {
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "Done");
    } else {
        server.send(500, "text/plain", "Aborted");
    }
}

static void handleServoTop()    { handleServoClick(true);  }
static void handleServoBottom() { handleServoClick(false); }

static void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println(F("\nNodeMCU Stepper WebGUI booting…"));

    // LED
    pinMode(LED_PIN, OUTPUT);
    setLed(false);

    // LittleFS
    if (!LittleFS.begin()) {
        Serial.println(F("LittleFS mount failed!"));
    } else {
        Serial.println(F("LittleFS mounted."));
        loadConfig();
    }

    // Motor pins
    applyPinModes();

    // Servos (horizontal mode)
    if (gCfg.axis == AXIS_H) ensureServosAttached();

    // WiFi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                      IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    WiFi.softAP(gCfg.ssid, gCfg.password);
    Serial.print(F("AP IP: "));
    Serial.println(WiFi.softAPIP());

    // HTTP routes
    server.on("/",             HTTP_GET,  handleRoot);
    server.on("/config",       HTTP_GET,  handleConfig);
    server.on("/save",         HTTP_POST, handleSave);
    server.on("/backup",       HTTP_GET,  handleBackup);
    server.on("/restore",      HTTP_POST, handleRestoreFinish, handleRestoreUpload);
    server.on("/test/en",      HTTP_GET,  handleTestEn);
    server.on("/test/step",    HTTP_GET,  handleTestStep);
    server.on("/test/dir",     HTTP_GET,  handleTestDir);
    server.on("/servo/top",    HTTP_GET,  handleServoTop);
    server.on("/servo/bottom", HTTP_GET,  handleServoBottom);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println(F("HTTP server started."));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    server.handleClient();
}
