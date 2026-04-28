# C4-ROBOT

NodeMCU (ESP8266) stepper-motor web controller with optional horizontal-mode button servos.

## Firmware

**File:** `nodemcu_webgui.ino` — FW **0.92**

### Hardware supported

- **Stepper motor driver** (EN / DIR / STEP, configurable pins, active-high/low logic)
- **Endstops** × 2 (BEGIN / END, active-low INPUT_PULLUP)
- **Button servos** × 2 (Top / Bottom) — **horizontal axis mode only**
- NodeMCU blue LED (GPIO2) lights while the motor is moving
- OTA firmware update (password = WiFi password)
- MQTT commands for remote control and JSON event telemetry

### Web GUI pages

| URL | Description |
|-----|-------------|
| `/` | Home — control, move, status |
| `/config` | Config — all settings, pin test, servo test |
| `/status` | Plain-text status (polled by home page) |

### Config page features

- **Save config** button at **top and bottom** of the form
- **Backup / Restore** config via browser:
  - *Download config.json* — saves current `/config.json` to your PC
  - *Upload & Restore* — uploads a JSON file, overwrites `/config.json`, reboots

---

## MQTT

### Broker settings

The broker address and port are compiled in. Edit these constants near the top
of `nodemcu_webgui.ino` before flashing:

| Constant | Default | Description |
|----------|---------|-------------|
| `MQTT_HOST` | `192.168.1.6` | MQTT broker IP address |
| `MQTT_PORT` | `1883` | MQTT broker port |

The device connects without authentication. Adjust `mqttEnsureConnected()` if
your broker requires credentials.

### Topic scheme

All topics are prefixed with `nodemcu/axis/{axis}/` where `{axis}` is `vertical`
or `horizontal` depending on the configured axis type.

#### Command topics (subscribed, plain-text payload)

| Topic | Payload | Description |
|-------|---------|-------------|
| `…/cmd/move` | integer steps (signed) | Move N steps |
| `…/cmd/goto` | integer index 1–40 | Go to stored position |
| `…/cmd/speed` | integer steps/s | Set max speed |
| `…/cmd/accel` | float steps/s² | Set acceleration |
| `…/cmd/stop` | any | Request stop |

#### State topic (published, retained plain-text)

| Topic | Description |
|-------|-------------|
| `…/state` | Plain-text status snapshot, published after every command and every 2 s |

#### Event topic (published, JSON, **new in 0.92**)

| Topic | Description |
|-------|-------------|
| `…/event` | JSON telemetry event published on motion start and endstop detection |

### Event payload format

All event messages share the same JSON schema:

```json
{
  "event":     "<event_name>",
  "command":   "<command_string>",
  "steps":     <integer>,
  "timestamp": "<ISO-8601 UTC string>"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `event` | string | `"motion_started"` or `"endstop_reached"` |
| `command` | string | Command that triggered the motion (see table below) |
| `steps` | integer | Current step position counter at the time of the event |
| `timestamp` | string | UTC ISO-8601 (e.g. `"2025-01-15T14:32:07Z"`). Falls back to `"T+<millis>ms"` before NTP sync. |

#### `command` values

| Trigger | `command` value example |
|---------|------------------------|
| HTTP `/move?steps=500` | `"move:500"` |
| HTTP `/home?dir=begin` | `"home:begin"` |
| HTTP `/home?dir=end` | `"home:end"` |
| HTTP `/goto?i=3` | `"goto:3"` |
| MQTT `…/cmd/move` payload `500` | `"mqtt-move:500"` |
| MQTT `…/cmd/goto` payload `3` | `"mqtt-goto:3"` |
| Test mode (endstop-to-endstop cycles) | `"test:end"` or `"test:begin"` |

### Example payloads

**motion_started** — emitted immediately before the step loop begins:

```json
{
  "event": "motion_started",
  "command": "move:500",
  "steps": 1240,
  "timestamp": "2025-01-15T14:32:07Z"
}
```

**endstop_reached** — emitted after the step loop exits because an endstop was
detected (BEGIN, END, or both):

```json
{
  "event": "endstop_reached",
  "command": "home:begin",
  "steps": 0,
  "timestamp": "2025-01-15T14:32:09Z"
}
```

### NTP / timestamp notes

`configTime(0, 0, "pool.ntp.org", "time.nist.gov")` is called at startup after
WiFi connects. The SNTP client syncs in the background. Until the first sync
the `timestamp` field uses an uptime fallback (`"T+<millis>ms"`). Once synced
(typically within a few seconds of boot) all timestamps are proper ISO-8601 UTC.

---

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
