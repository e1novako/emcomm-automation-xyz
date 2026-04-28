# C4-ROBOT

NodeMCU (ESP8266) stepper-motor web controller with optional horizontal-mode button servos.

## Firmware

**File:** `nodemcu_webgui.ino` — FW **0.93**

### Hardware supported

- **Stepper motor driver** (EN / DIR / STEP, configurable pins, active-high/low logic)
- **Endstops** × 2 (BEGIN / END, active-low INPUT_PULLUP)
- **Button servos** × 2 (Top / Bottom) — **horizontal axis mode only**
- NodeMCU blue LED (GPIO2) lights while the motor is moving
- OTA firmware update (configurable password; falls back to WiFi password)
- MQTT commands for remote control, JSON event telemetry, and JSON state snapshots

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

## First-time setup — WiFi credentials

WiFi credentials are **not** compiled in. They use the `secrets.h` pattern:

1. Copy `secrets.h.template` to `secrets.h` inside the `C4-ROBOT/` folder.
2. Fill in your SSID and password in `secrets.h`:
   ```cpp
   #define SECRET_WIFI_SSID "your-wifi-ssid"
   #define SECRET_WIFI_PASS "your-wifi-password"
   ```
3. `secrets.h` is listed in `.gitignore` — it will **not** be committed.

> **Alternative:** leave `secrets.h` with empty strings (the default). The device
> will fail to connect to WiFi on first boot, but you can then set the SSID / password
> through the Config page once connected (e.g. via AP fallback or serial flash).

---

## MQTT

### Broker settings

MQTT broker host and port are configurable through the **Config** web page — no
reflashing required. Defaults are preserved.

| Config field | Default | Description |
|--------------|---------|-------------|
| MQTT Host | `192.168.1.6` | Broker hostname or IP address |
| MQTT Port | `1883` | Broker TCP port (1–65535) |

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

#### State topic (published, retained JSON, **new in 0.93**)

| Topic | Description |
|-------|-------------|
| `…/state` | JSON status snapshot, published after every command and every 2 s |

#### Event topic (published, JSON)

| Topic | Description |
|-------|-------------|
| `…/event` | JSON telemetry event published on motion start and endstop detection |

### State payload format (new in 0.93)

```json
{
  "fw":        "0.93",
  "axis":      "vertical",
  "moving":    false,
  "pos":       1240,
  "direction": "positive",
  "endstops": {
    "begin":      false,
    "end":        false,
    "both_error": false
  },
  "test_mode": {
    "active": false,
    "cycle":  0,
    "target": "end"
  },
  "steps_done": 1240,
  "timestamp":  "2025-01-15T14:32:07Z"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `fw` | string | Firmware version |
| `axis` | string | `"vertical"` or `"horizontal"` |
| `moving` | bool | `true` while step loop is active |
| `pos` | integer | Absolute step position (0 = BEGIN endstop) |
| `direction` | string | `"positive"` or `"negative"` — direction of last/active move |
| `endstops.begin` | bool | BEGIN endstop pressed |
| `endstops.end` | bool | END endstop pressed |
| `endstops.both_error` | bool | Both endstops pressed simultaneously (movement disabled) |
| `test_mode.active` | bool | Test mode running |
| `test_mode.cycle` | integer | Current cycle (0–10) |
| `test_mode.target` | string | `"end"` or `"begin"` — current test direction |
| `steps_done` | integer | Live counter while moving; final count when idle |
| `timestamp` | string | UTC ISO-8601. Falls back to `"T+<millis>ms"` before NTP sync. |

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
  "mqtt": { "host": "192.168.1.6", "port": 1883 },
  "ota_pass": "",
  "servos": {
    "top": { "pin": 14, "min_us": 1000, "max_us": 2000 },
    "bot": { "pin": 0,  "min_us": 1000, "max_us": 2000 },
    "press_ms": 250,
    "release_ms": 250,
    "pause_ms": 250
  },
  "positions": [0, 0, "…"]
}
```

### Libraries required

- `ESP8266WiFi`, `ESP8266WebServer`, `ArduinoOTA` (ESP8266 core)
- `PubSubClient` (MQTT)
- `LittleFS` (ESP8266 core)
- `ArduinoJson` ≥ v7
- `Servo` / `ESP8266Servo`
