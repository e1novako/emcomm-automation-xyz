# C4-ROBOT

NodeMCU (ESP8266) stepper-motor web controller with optional horizontal-mode button servos.

## Firmware

**File:** `nodemcu_webgui.ino` — FW **0.89**

### Hardware supported

- **Stepper motor driver** (EN / DIR / STEP, configurable pins, active-high/low logic)
- **Endstops** × 2 (BEGIN / END, active-low INPUT_PULLUP)
- **Button servos** × 2 (Top / Bottom) — **horizontal axis mode only**
- NodeMCU blue LED (GPIO2) lights while the motor is moving
- OTA firmware update (password = WiFi password)
- MQTT commands for remote control

### Web GUI pages

| URL | Description |
|-----|-------------|
| `/` | Home — control, move, status |
| `/config` | Config — all settings, pin test, servo test |
| `/status` | Plain-text status (polled by home page) |

### Config page features (v0.89)

- **Save config** button at **top and bottom** of the form
- **Backup / Restore** config via browser:
  - *Download config.json* — saves current `/config.json` to your PC
  - *Upload & Restore* — uploads a JSON file, overwrites `/config.json`, reboots

### Horizontal mode — Button Servos

When axis is set to **Horizontal (H)**, a *Button Servos* section appears on the Config page:

| Setting | Description |
|---------|-------------|
| Pin | GPIO pin for the servo signal wire |
| Min µs | Pulse width in released (idle) position |
| Max µs | Pulse width in pressed position |
| Press hold (ms) | How long to hold the pressed position |
| Release hold (ms) | How long to hold the released position |
| Pause between clicks (ms) | Gap between consecutive clicks |

**Servo endpoints:**

| Endpoint | Description |
|----------|-------------|
| `/servo/top/min` | Move top servo to min (release) |
| `/servo/top/max` | Move top servo to max (press) |
| `/servo/top/click?n=N` | Click top servo N times |
| `/servo/bot/min` | Move bottom servo to min (release) |
| `/servo/bot/max` | Move bottom servo to max (press) |
| `/servo/bot/click?n=N` | Click bottom servo N times |

GUI click buttons are provided for **1, 3, 4, 9, 13** clicks for each servo.

Servo pulse widths are clamped to **400 – 2600 µs** for safety.

### Config backup endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/config/download` | GET | Download `/config.json` |
| `/config/upload` | POST (multipart) | Upload and restore `/config.json`, then reboot |

### Config file (`/config.json` in LittleFS)

```json
{
  "axis": "H",
  "wifi": { "ssid": "…", "pass": "…" },
  "tune": { "max_sps": 800, "accel": 2500 },
  "pins": {
    "motor": { "en": 16, "dir": 5, "step": 4 },
    "endstops": { "begin": 12, "end": 13 }
  },
  "logic": {
    "en_active_high": true,
    "dir_active_high": true,
    "step_active_high": true
  },
  "servos": {
    "top": { "pin": 14, "min_us": 1000, "max_us": 2000 },
    "bot": { "pin": 0,  "min_us": 1000, "max_us": 2000 },
    "press_ms": 250,
    "release_ms": 250,
    "pause_ms": 250
  },
  "positions": [0, 0, …]
}
```

### Libraries required

- `ESP8266WiFi`, `ESP8266WebServer`, `ArduinoOTA` (ESP8266 core)
- `PubSubClient` (MQTT)
- `LittleFS` (ESP8266 core)
- `ArduinoJson` ≥ v7
- `Servo` / `ESP8266Servo`
