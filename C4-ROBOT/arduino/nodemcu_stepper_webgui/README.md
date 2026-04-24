# nodemcu_stepper_webgui

ESP8266 (NodeMCU) firmware that exposes a responsive web UI for controlling a stepper motor driver and, in **Horizontal** axis mode, two servo-actuated push-buttons.

## Features

| Feature | Details |
|---------|---------|
| Web UI | Home page + Config page |
| Config persistence | LittleFS `/config.json` |
| Save button | Top **and** bottom of Config page |
| Backup / Restore | Download & upload `config.json` via GUI |
| Stepper pins | EN / STEP / DIR – configurable pin, label, colour, active-high/low logic |
| Pin test | Force EN / STEP / DIR high or low from Home page |
| LED feedback | NodeMCU blue LED (GPIO2) on while servo is clicking |
| Axis modes | **Vertical** (default) / **Horizontal** |
| Horizontal mode | Top & Bottom button servos: pin + min/max µs |
| Click buttons | 1 × / 3 × / 4 × / 9 × / 13 × per servo |
| HTTP endpoints | See table below |

### HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Home page |
| GET | `/config` | Config page |
| POST | `/save` | Save configuration |
| GET | `/backup` | Download `config.json` |
| POST | `/restore` | Upload & restore `config.json` |
| GET | `/test/en?state=0\|1` | Force EN pin low/high |
| GET | `/test/step?state=0\|1` | Force STEP pin low/high |
| GET | `/test/dir?state=0\|1` | Force DIR pin low/high |
| GET | `/servo/top?clicks=N` | Click Top button N times (1–50) |
| GET | `/servo/bottom?clicks=N` | Click Bottom button N times (1–50) |
| GET | `/servo/top?pos=min\|max` | Move Top servo to MIN or MAX position (test) |
| GET | `/servo/bottom?pos=min\|max` | Move Bottom servo to MIN or MAX position (test) |

## Build & Upload

### Requirements

* [Arduino IDE](https://www.arduino.cc/en/software) ≥ 1.8 or Arduino IDE 2.x
* **ESP8266 Arduino core** – install via Boards Manager:  
  URL: `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
* **ArduinoJson** library (v6.x) – install via Library Manager
* **Servo** library – bundled with the ESP8266 core (no extra install needed)

### Board Settings (Arduino IDE)

| Setting | Value |
|---------|-------|
| Board | NodeMCU 1.0 (ESP-12E Module) |
| Flash size | 4MB (FS: 1MB – LittleFS) |
| Upload speed | 115200 |
| Port | your COM / `/dev/ttyUSB*` port |

### Steps

1. Open `nodemcu_stepper_webgui.ino` in Arduino IDE.
2. Select the board and port as above.
3. Click **Upload**.
4. After upload, open **Tools → ESP8266 LittleFS Data Upload** once to initialise the filesystem (optional – the firmware creates `config.json` automatically on first save).
5. Connect to the WiFi AP `NodeMCU-Stepper` (password `12345678`).
6. Browse to `http://192.168.4.1/`.

### Changing WiFi Credentials

Edit the defaults in the `AppConfig` struct at the top of the `.ino`, **or** use the Config page in the web UI and click *Save*.

## Default Pin Assignments

| Signal | GPIO | NodeMCU label |
|--------|------|---------------|
| EN | GPIO4 | D2 |
| STEP | GPIO5 | D1 |
| DIR | GPIO0 | D3 |
| Top servo | GPIO14 | D5 |
| Bottom servo | GPIO12 | D6 |
| LED | GPIO2 | D4 (built-in) |

All pins are reconfigurable via the Config page.
