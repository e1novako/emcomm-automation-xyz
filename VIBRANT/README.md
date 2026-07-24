# VIBRANT (NodeMCU v3 / ESP8266)

Arduino project for NodeMCU v3 with a web interface for controlling up to 16 outputs.

## WARNING

Factory Wi-Fi credentials are publicly known:

- SSID: `Z-Wave Automation`
- Password: `Fiber714Cvet`

Change the password immediately in **Settings** after the first boot.

## Highlights

- All configuration is persisted in LittleFS JSON file: `/vibrant_config.json`
- English-only UI labels and source comments
- Main page lists configured devices (model, name, status) and provides checkbox toggles and per-output load action buttons
- Settings, toggle actions, and config maintenance endpoints are protected with HTTP Basic Auth (`admin` / current Wi-Fi password)
- Settings page supports:
  - MAC address
  - DHCP hostname
  - SSID
  - Password
  - Wi-Fi output power/strength
  - MQTT broker host, port, username, and password (enable/disable toggle)
  - Bulk-copying the first Model/Name to all visible rows (`#<number>` in the first name continues from the parsed starting number)
  - Reversing GPIO assignments across the currently configured output rows
  - Up to 16 device entries (`model`, `name`, output `D0`–`D8`, `RX`, `TX`, or `none`)
  - MQTT server/host, port, user, password, and enable toggle
- Configuration maintenance routes:
  - Export backup (`/config/export`)
  - Import backup (`/config/import`)
  - Factory reset to defaults (`/config/factory-reset`)
- Web-based OTA firmware update (`/firmware/update`): upload a compiled `.bin` directly from the browser; the device reboots automatically after a successful flash
- Optional ArduinoOTA support (disabled by default): developer/service OTA uploads via Arduino IDE or OTA-capable tooling when explicitly enabled in Settings
- Serial diagnostics print boot progress, Wi-Fi state, configured outputs, and important error/status messages
- Output pins are configured/driven only after a 1-second post-boot delay
- Firmware automatically attempts Wi-Fi reconnect after disconnects
- Firmware performs a controlled restart for unrecoverable conditions after logging the reason to serial
- FLASH/GPIO0 factory reset is checked only during a short boot-time sampling window

## Default factory GPIO assignment

On first boot (or after factory reset), outputs 1–8 are mapped to **D0–D7** in order:

| Output | NodeMCU label | GPIO |
|--------|--------------|------|
| 1 | D0 | GPIO16 |
| 2 | D1 | GPIO5 |
| 3 | D2 | GPIO4 |
| 4 | D3 | GPIO0 |
| 5 | D4 | GPIO2 |
| 6 | D5 | GPIO14 |
| 7 | D6 | GPIO12 |
| 8 | D7 | GPIO13 |

Outputs 9–16 default to unassigned (`none`).

## MQTT

### Configuration

Enable MQTT and set the broker host/port in **Settings → MQTT**.  Fields:

| Field | Description |
|-------|-------------|
| Enable MQTT | Enables MQTT connectivity |
| MQTT server host | Broker IP or hostname (e.g. `192.168.1.6`) |
| MQTT port | Broker port (default `1883`) |
| MQTT user | Optional broker username |
| MQTT password | Optional broker password |

### Topics

Base: `vibrant/<hostname>/`

| Topic | Direction | Payload | Description |
|-------|-----------|---------|-------------|
| `vibrant/<hostname>/out/<N>/set` | Subscribe | `ON` or `OFF` | Set output N state |
| `vibrant/<hostname>/out/<N>/action` | Subscribe | command name | Run a load action on output N |
| `vibrant/<hostname>/out/<N>/state` | Publish (retained) | `ON` or `OFF` | Current output N state |

N is the zero-based output index (0 = output 1, 1 = output 2, …).

Action commands accepted via the `action` topic: `power_on`, `power_off`, `leave_mesh`, `factory_reset`.

## Load action commands

The main page exposes per-output action buttons. The same commands are accepted via MQTT and the `/action` HTTP endpoint.

| Command | Description |
|---------|-------------|
| `power_on` | Turn the output ON immediately |
| `power_off` | Turn the output OFF immediately |
| `leave_mesh` | Run the leave-mesh power-cycling sequence |
| `factory_reset` | Run the factory-reset power-cycling sequence for the connected bulb |

### Leave mesh sequence

Starting with the bulb powered on:
1. Cycle power **5 times**: 5 s OFF → 1 s ON per cycle
2. Wait 5 s (bulb turns green)
3. Trigger: 2 s OFF → 1 s ON (cycles power while bulb is green)

Total sequence duration: ~38 s (5×6 s + 5 s + 3 s)

### Factory reset sequence (connected bulb)

Starting with the bulb powered on:
1. Cycle power **13 times**: 5 s OFF → 1 s ON per cycle
2. Wait 5 s (bulb transitions from 1800 K/red to blue)
3. Trigger: 2 s OFF → 1 s ON (cycles power while bulb is blue)

Total sequence duration: ~86 s (13×6 s + 5 s + 3 s)

### Non-blocking execution

All GPIO activity (including the timed cycling sequences) runs in the background via a `millis()`-based state machine in the main loop. The web UI and MQTT connection remain fully responsive during any running sequence. The running action is shown in a banner on the main page (auto-refreshes every 3 s) and can be cancelled at any time.

## Default factory values

- SSID: `Z-Wave Automation`
- Password: `Fiber714Cvet`
- Hostname format: `C4-VIBRANT-<last three octets of MAC>`
- Wi-Fi power range: `5.0 - 20.5 dBm`
- MQTT: disabled by default
- Default GPIO assignments: outputs 1–8 mapped to D0–D7 (GPIO16, GPIO5, GPIO4, GPIO0, GPIO2, GPIO14, GPIO12, GPIO13)

Security note: factory credentials are public and meant only for first setup.

Additional security note: HTTP Basic Auth is not encrypted on plain HTTP. Use this firmware only on trusted local networks/AP access.

## MQTT

When MQTT is enabled in Settings, the firmware:

- Connects to the configured broker on boot and reconnects automatically every 15 seconds if the connection is lost.
- Publishes the retained state of each output to `vibrant/<hostname>/output/<N>/state` (`1` = ON, `0` = OFF) whenever a toggle is applied (from the web UI or MQTT).
- Subscribes to `vibrant/<hostname>/output/<N>/set` for each output. Send `1`, `ON` (case-insensitive), or `true` (case-insensitive) to turn on; `0` or any other value to turn off.

MQTT username and password are optional (leave blank for anonymous access). The MQTT password field is never pre-filled in the form; leave it empty to keep the current stored password.

## Firmware update (OTA)

VIBRANT supports two OTA firmware update paths:

- **Web UI OTA upload (primary/recommended)**
- **ArduinoOTA (secondary developer/service path, disabled by default)**

### Web UI firmware upload (primary method)

1. Compile `VIBRANT/VIBRANT.ino` for your NodeMCU board to produce a `.bin` file.
   - Arduino IDE: **Sketch → Export Compiled Binary**
   - `arduino-cli`: `arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries VIBRANT/VIBRANT.ino`
2. Open the device web UI and log in.
3. Go to **Settings → Firmware update** and click **Open firmware update page**, or navigate directly to `http://<device-ip>/firmware/update`.
4. Select the compiled `.bin` file and click **Upload and flash**.
5. Wait for the upload to complete. The device reboots automatically.
6. The page reloads after 15 seconds. Verify the new firmware is running.

> **Warning:** Do not power off the device during an update. A power loss mid-flash may require USB reflashing to recover.

### ArduinoOTA (secondary method)

ArduinoOTA is available for developer/service workflows and is **disabled by default**.

Enable it in **Settings → Diagnostics → Enable ArduinoOTA service**.

Behavior:

- Uses the configured device hostname (`hostname`) as the ArduinoOTA hostname.
- Uses the current admin password (same password used for HTTP Basic Auth) as the ArduinoOTA password.
- Is serviced in the main loop, so background output actions, web UI, and MQTT continue to run.

Typical flow:

1. Enable ArduinoOTA in Settings and save.
2. Ensure your development machine is on the same network.
3. Select the device's network OTA target in Arduino IDE/tooling.
4. Upload firmware over Wi-Fi; the device reboots automatically on success.

## Arduino libraries

- ESP8266 core libraries (`ESP8266WiFi`, `ESP8266WebServer`, `Updater`) from ESP8266 board package 3.x
- `LittleFS`
- `ArduinoJson` 7.x
- `PubSubClient` 2.x (for MQTT)

## File layout

- `VIBRANT.ino` — complete firmware sketch

## Build notes

1. Open `VIBRANT/VIBRANT.ino` in Arduino IDE (or compile with `arduino-cli` for an ESP8266 board).
2. Select a NodeMCU v3 compatible ESP8266 board profile.
3. Ensure `ArduinoJson` and `PubSubClient` are installed.
4. Flash the firmware.

After boot, join the configured AP and open the device IP in a browser.

## Current networking behavior

- The configured `SSID`/`Password` are used for both SoftAP and Station connect attempts.
- Edited MAC address is applied to both SoftAP and Station interfaces.
- Wi-Fi power is clamped to a minimum of `5.0 dBm`.
- If station connectivity drops, the firmware periodically attempts reconnect.
- If storage or runtime recovery fails irrecoverably, the device logs the reason and restarts.
- MQTT reconnects automatically (every 10 s) when the broker is unreachable.
