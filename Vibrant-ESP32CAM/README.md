# Vibrant-ESP32CAM

Example firmware for an AI-Thinker-style ESP32-CAM (ESP-32S) with TY-OV2/OV2640 camera, microSD, Arduino OTA, and a configuration web server.

## Hardware and pinout

| Function | GPIO |
|---|---:|
| Camera PWDN, RESET | 32, -1 |
| Camera XCLK, SIOD, SIOC | 0, 26, 27 |
| Camera Y2..Y9 | 5, 18, 19, 21, 36, 39, 34, 35 |
| Camera VSYNC, HREF, PCLK | 25, 23, 22 |
| Red status LED (active low) | 33 |
| White flash LED (PWM) | 4 |

SD uses `SD_MMC` in 1-bit mode. A missing card is reported but does not prevent boot.

## Build and flash

Install the ESP32 Arduino core 3.x and ArduinoJson 7.x, select `esp32:esp32:esp32cam`, and use an OTA-capable partition scheme such as **Default 4MB with spiffs** (do not use `huge_app`). Then:

```sh
cd Vibrant-ESP32CAM
arduino-cli compile --fqbn esp32:esp32:esp32cam --export-binaries .
arduino-cli upload -p PORT --fqbn esp32:esp32:esp32cam .
```

Connect at 115200 baud. The first boot starts an access point named `vibrant-esp32cam` if the saved station settings cannot connect. Configure station Wi-Fi at `/`, then restart if needed.

## OTA and endpoints

The hostname and optional ArduinoOTA password are configured on the web page. ArduinoOTA is available after Wi-Fi starts. The `/firmware` HTTP POST endpoint accepts a compiled firmware binary upload. During an OTA update the red LED blinks.

* `/` - configuration, live preview, and status
* `/stream` on port 81 - MJPEG stream
* `/capture` - JPEG still
* `GET /api/status` - uptime, network, memory, SD, and version JSON
* `GET /api/config`, `POST /api/config` - configuration JSON/form data
* `POST /api/snapshot` - save a JPEG to SD
* `/files` - JSON list of SD files

## Configuration and debugging

Camera framesize, JPEG quality, brightness, contrast, saturation, flips, flash, station credentials, hostname, and OTA password are stored in ESP32 Preferences/NVS. Enable **Serial debug logging** on the web page to log Wi-Fi, camera, SD, OTA, web, and configuration events; it is persisted and can be disabled from the same page. Debug output never includes passwords.

## Changelog

* 1.0.3 - Use shared ArduinoOTA setup and HTTP OTA upload sequencing.
* 1.0.2 - Retain GUI-controlled debug output and shared diagnostic gating.
* 1.0.1 - Use the shared GUI-controlled debug-output gate; preserve the persisted web setting.
* 1.0.0 - Initial ESP32-CAM camera, SD, web, persistence, debug, and OTA example.

The repository-managed `libraries/EmcommCommon` library provides the OTA upload
state machine and ArduinoOTA setup helper. Camera routes, HTTP response handling,
and the ESP32 `Update` backend remain local to this sketch.
