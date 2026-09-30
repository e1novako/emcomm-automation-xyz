#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SD_MMC.h>
#include "esp_camera.h"

#define FIRMWARE_VERSION "1.0.0"

// AI-Thinker ESP32-CAM / TY-OV2 (OV2640) pin map.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define STATUS_LED 33
#define FLASH_LED 4

Preferences prefs;
WebServer server(80);
WebServer streamServer(81);

struct Config {
  String ssid, password, hostname, otaPassword;
  framesize_t framesize = FRAMESIZE_VGA;
  int jpegQuality = 12, brightness = 0, contrast = 0, saturation = 0;
  bool vflip = false, hmirror = false, flash = false, debug = false;
} config;

bool sdReady = false;
bool otaUpdating = false;
unsigned long bootMillis;
unsigned long lastLed = 0;
bool ledState = false;

#define DBG(tag, format, ...) do { if (config.debug) Serial.printf("[DEBUG][" tag "] " format "\n", ##__VA_ARGS__); } while (0)
#define INFO(format, ...) Serial.printf("[INFO] " format "\n", ##__VA_ARGS__)

void setStatus(bool on) { digitalWrite(STATUS_LED, on ? LOW : HIGH); }
void saveConfig() {
  prefs.begin("vibrant", false);
  prefs.putString("ssid", config.ssid); prefs.putString("password", config.password);
  prefs.putString("hostname", config.hostname); prefs.putString("ota", config.otaPassword);
  prefs.putUInt("framesize", config.framesize); prefs.putInt("quality", config.jpegQuality);
  prefs.putInt("brightness", config.brightness); prefs.putInt("contrast", config.contrast);
  prefs.putInt("saturation", config.saturation); prefs.putBool("vflip", config.vflip);
  prefs.putBool("hmirror", config.hmirror); prefs.putBool("flash", config.flash);
  prefs.putBool("debug", config.debug); prefs.end();
  DBG("CONFIG", "configuration saved");
}
void loadConfig() {
  prefs.begin("vibrant", true);
  config.ssid = prefs.getString("ssid", "");
  config.password = prefs.getString("password", "");
  config.hostname = prefs.getString("hostname", "vibrant-esp32cam");
  config.otaPassword = prefs.getString("ota", "");
  config.framesize = (framesize_t)prefs.getUInt("framesize", FRAMESIZE_VGA);
  config.jpegQuality = constrain(prefs.getInt("quality", 12), 10, 63);
  config.brightness = constrain(prefs.getInt("brightness", 0), -2, 2);
  config.contrast = constrain(prefs.getInt("contrast", 0), -2, 2);
  config.saturation = constrain(prefs.getInt("saturation", 0), -2, 2);
  config.vflip = prefs.getBool("vflip", false); config.hmirror = prefs.getBool("hmirror", false);
  config.flash = prefs.getBool("flash", false); config.debug = prefs.getBool("debug", false);
  prefs.end();
}

bool initCamera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sscb_sda = SIOD_GPIO_NUM; c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM; c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = config.framesize; c.jpeg_quality = config.jpegQuality; c.fb_count = psramFound() ? 2 : 1;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t result = esp_camera_init(&c);
  if (result != ESP_OK) { INFO("camera init failed: 0x%x", result); return false; }
  sensor_t *s = esp_camera_sensor_get();
  s->set_brightness(s, config.brightness); s->set_contrast(s, config.contrast);
  s->set_saturation(s, config.saturation); s->set_vflip(s, config.vflip);
  s->set_hmirror(s, config.hmirror);
  DBG("CAMERA", "camera initialized, PSRAM=%s", psramFound() ? "yes" : "no");
  return true;
}
void applyCamera() {
  sensor_t *s = esp_camera_sensor_get(); if (!s) return;
  s->set_framesize(s, config.framesize); s->set_quality(s, config.jpegQuality);
  s->set_brightness(s, config.brightness); s->set_contrast(s, config.contrast);
  s->set_saturation(s, config.saturation); s->set_vflip(s, config.vflip);
  s->set_hmirror(s, config.hmirror);
}
void initSd() {
  sdReady = SD_MMC.begin("/sdcard", true);
  if (!sdReady) { INFO("SD card not available; snapshots remain disabled"); return; }
  DBG("SD", "SD card mounted, type=%u, size=%llu MB", SD_MMC.cardType(), SD_MMC.cardSize() / (1024 * 1024));
}
String jsonStatus() {
  JsonDocument doc; doc["version"] = FIRMWARE_VERSION; doc["uptime"] = (millis() - bootMillis) / 1000;
  doc["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["rssi"] = WiFi.isConnected() ? WiFi.RSSI() : 0; doc["heap"] = ESP.getFreeHeap();
  doc["psram"] = ESP.getFreePsram(); doc["sd"] = sdReady; doc["sd_bytes"] = sdReady ? SD_MMC.totalBytes() : 0;
  doc["debug"] = config.debug; String out; serializeJson(doc, out); return out;
}
void sendCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "camera capture failed"); return; }
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len); esp_camera_fb_return(fb);
}
void stream() {
  WiFiClient client = streamServer.client();
  client.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Cache-Control: no-cache\r\nConnection: close\r\n\r\n");
  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get(); if (!fb) break;
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len); client.print("\r\n"); esp_camera_fb_return(fb); delay(10);
  }
  DBG("WEB", "stream client disconnected");
}
bool saveSnapshot() {
  if (!sdReady) return false;
  String path = "/snapshot-" + String(millis()) + ".jpg";
  File file = SD_MMC.open(path, FILE_WRITE); if (!file) return false;
  camera_fb_t *fb = esp_camera_fb_get(); bool ok = fb && file.write(fb->buf, fb->len) == fb->len;
  if (fb) esp_camera_fb_return(fb); file.close(); DBG("SD", "snapshot %s: %s", path.c_str(), ok ? "saved" : "failed"); return ok;
}
String fileList() {
  JsonDocument doc; JsonArray files = doc["files"].to<JsonArray>();
  if (sdReady) { File root = SD_MMC.open("/"); File f = root.openNextFile(); while (f) {
    JsonObject item = files.add<JsonObject>(); item["name"] = f.name(); item["size"] = f.size(); f = root.openNextFile();
  }}
  String out; serializeJson(doc, out); return out;
}
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><meta name=viewport content="width=device-width"><title>Vibrant ESP32-CAM</title>
<style>body{font:16px sans-serif;max-width:760px;margin:auto}input{margin:.25em;padding:.35em}fieldset{margin:.7em 0}img{max-width:100%}button{padding:.5em}</style>
<h1>Vibrant ESP32-CAM <small id=v></small></h1><img id=cam><p><a href=/capture target=_blank>Still capture</a> · <button onclick="snap()">Save snapshot</button> · <a href=/files>SD files</a></p>
<fieldset><legend>Wi-Fi / OTA</legend><form method=post action=/api/config>
SSID <input name=ssid><br>Password <input name=password type=password><br>Hostname <input name=hostname><br>OTA password <input name=otaPassword type=password></fieldset>
<fieldset><legend>Camera</legend>Framesize <select name=framesize><option value=5>QVGA</option><option value=10>VGA</option><option value=11>SVGA</option><option value=12>XGA</option><option value=13>HD</option><option value=14>UXGA</option></select>
JPEG quality <input name=jpegQuality type=number min=10 max=63 value=12> Brightness <input name=brightness type=number min=-2 max=2 value=0>
Contrast <input name=contrast type=number min=-2 max=2 value=0> Saturation <input name=saturation type=number min=-2 max=2 value=0><br>
<label><input name=vflip type=checkbox> Vertical flip</label> <label><input name=hmirror type=checkbox> Horizontal mirror</label></fieldset>
<fieldset><legend>LED / diagnostics</legend><label><input name=flash type=checkbox> Flash on</label> <label><input name=debug type=checkbox> Serial debug logging</label></fieldset>
<button>Save configuration</button></form><pre id=s></pre><script>
cam.src='http://'+location.hostname+':81/stream'; async function status(){let x=await fetch('/api/status').then(r=>r.json());v.textContent='v'+x.version;s.textContent=JSON.stringify(x,null,2)} async function snap(){await fetch('/api/snapshot',{method:'POST'});status()} status();setInterval(status,5000)
</script>)HTML";
void handleConfig() {
  bool jsonBody = server.hasArg("plain");
  if (jsonBody) {
    JsonDocument body;
    if (deserializeJson(body, server.arg("plain")) == DeserializationError::Ok) {
      if (body["ssid"].is<const char *>()) config.ssid = body["ssid"].as<String>();
      if (body["password"].is<const char *>()) config.password = body["password"].as<String>();
      if (body["hostname"].is<const char *>()) config.hostname = body["hostname"].as<String>();
      if (body["otaPassword"].is<const char *>()) config.otaPassword = body["otaPassword"].as<String>();
      if (body["framesize"].is<int>()) config.framesize = (framesize_t)constrain(body["framesize"].as<int>(), 0, 21);
      if (body["jpegQuality"].is<int>()) config.jpegQuality = constrain(body["jpegQuality"].as<int>(), 10, 63);
      if (body["brightness"].is<int>()) config.brightness = constrain(body["brightness"].as<int>(), -2, 2);
      if (body["contrast"].is<int>()) config.contrast = constrain(body["contrast"].as<int>(), -2, 2);
      if (body["saturation"].is<int>()) config.saturation = constrain(body["saturation"].as<int>(), -2, 2);
      if (body["vflip"].is<bool>()) config.vflip = body["vflip"].as<bool>();
      if (body["hmirror"].is<bool>()) config.hmirror = body["hmirror"].as<bool>();
      if (body["flash"].is<bool>()) config.flash = body["flash"].as<bool>();
      if (body["debug"].is<bool>()) config.debug = body["debug"].as<bool>();
    }
  }
  if (!jsonBody) {
    if (server.hasArg("ssid")) config.ssid = server.arg("ssid"); if (server.hasArg("password")) config.password = server.arg("password");
    if (server.hasArg("hostname")) config.hostname = server.arg("hostname"); if (server.hasArg("otaPassword")) config.otaPassword = server.arg("otaPassword");
    if (server.hasArg("framesize")) config.framesize = (framesize_t)constrain(server.arg("framesize").toInt(), 0, 21);
    if (server.hasArg("jpegQuality")) config.jpegQuality = constrain(server.arg("jpegQuality").toInt(), 10, 63);
    if (server.hasArg("brightness")) config.brightness = constrain(server.arg("brightness").toInt(), -2, 2);
    if (server.hasArg("contrast")) config.contrast = constrain(server.arg("contrast").toInt(), -2, 2);
    if (server.hasArg("saturation")) config.saturation = constrain(server.arg("saturation").toInt(), -2, 2);
    config.vflip = server.hasArg("vflip"); config.hmirror = server.hasArg("hmirror"); config.flash = server.hasArg("flash"); config.debug = server.hasArg("debug");
  }
  saveConfig(); applyCamera(); ledcWrite(FLASH_LED, config.flash ? 255 : 0); DBG("CONFIG", "web configuration changed");
  server.sendHeader("Location", "/"); server.send(303);
}
void setupWeb() {
  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/capture", HTTP_GET, sendCapture); server.on("/api/status", HTTP_GET, [](){ server.send(200, "application/json", jsonStatus()); });
  server.on("/api/config", HTTP_POST, handleConfig); server.on("/api/snapshot", HTTP_POST, [](){ server.send(200, "text/plain", saveSnapshot() ? "saved" : "SD unavailable"); });
  server.on("/files", HTTP_GET, [](){ server.send(200, "application/json", fileList()); });
  server.on("/api/config", HTTP_GET, [](){ JsonDocument d; d["ssid"] = config.ssid; d["hostname"] = config.hostname; d["debug"] = config.debug; String o; serializeJson(d,o); server.send(200,"application/json",o); });
  server.on("/firmware", HTTP_POST, [](){ server.send(200, "text/plain", Update.hasError() ? "update failed" : "update complete; rebooting"); delay(300); ESP.restart(); },
    [](){ HTTPUpload &u = server.upload(); if (u.status == UPLOAD_FILE_START) Update.begin(UPDATE_SIZE_UNKNOWN); else if (u.status == UPLOAD_FILE_WRITE) Update.write(u.buf, u.currentSize); else if (u.status == UPLOAD_FILE_END) Update.end(true); });
  server.begin(); streamServer.on("/stream", HTTP_GET, stream); streamServer.begin(); INFO("web server ready");
}
void setupOta() {
  ArduinoOTA.setHostname(config.hostname.c_str()); if (config.otaPassword.length()) ArduinoOTA.setPassword(config.otaPassword.c_str());
  ArduinoOTA.onStart([](){ otaUpdating = true; DBG("OTA", "update started"); }).onEnd([](){ setStatus(false); })
    .onProgress([](unsigned int p, unsigned int t){ if (config.debug) Serial.printf("[DEBUG][OTA] %u%%\n", p * 100 / t); })
    .onError([](ota_error_t e){ otaUpdating = false; INFO("OTA error %u", e); }); ArduinoOTA.begin();
}
void setupWifi() {
  WiFi.mode(WIFI_STA); WiFi.setHostname(config.hostname.c_str());
  if (config.ssid.length()) { WiFi.begin(config.ssid.c_str(), config.password.c_str()); unsigned long start = millis(); while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) { delay(250); Serial.print("."); } }
  if (WiFi.status() != WL_CONNECTED) { WiFi.mode(WIFI_AP_STA); WiFi.softAP(config.hostname.c_str()); INFO("STA failed; AP %s at %s", config.hostname.c_str(), WiFi.softAPIP().toString().c_str()); }
  else INFO("Wi-Fi connected: %s", WiFi.localIP().toString().c_str());
}
void setup() {
  Serial.begin(115200); bootMillis = millis(); pinMode(STATUS_LED, OUTPUT); setStatus(false);
  ledcAttach(FLASH_LED, 5000, 8); ledcWrite(FLASH_LED, 0); loadConfig();
  INFO("Vibrant ESP32-CAM v%s", FIRMWARE_VERSION); setupWifi(); initCamera(); initSd(); setupOta(); setupWeb();
}
void loop() {
  ArduinoOTA.handle(); server.handleClient(); streamServer.handleClient();
  if (otaUpdating && millis() - lastLed > 100) { lastLed = millis(); ledState = !ledState; setStatus(ledState); }
}
