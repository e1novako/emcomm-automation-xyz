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
- Main page lists configured devices (model, name, status) and provides checkbox toggles
- Settings, toggle actions, and config maintenance endpoints are protected with HTTP Basic Auth (`admin` / current Wi-Fi password)
- Settings page supports:
  - MAC address
  - DHCP hostname
  - SSID
  - Password
  - Wi-Fi output power/strength
  - Up to 16 device entries (`model`, `name`, output `D0`–`D15` or `none`)
- Configuration maintenance routes:
  - Export backup (`/config/export`)
  - Import backup (`/config/import`)
  - Factory reset to defaults (`/config/factory-reset`)
- Serial diagnostics print boot progress, Wi-Fi state, configured outputs, and important error/status messages
- Firmware automatically attempts Wi-Fi reconnect after disconnects
- Firmware performs a controlled restart for unrecoverable conditions after logging the reason to serial

## Default factory values

- SSID: `Z-Wave Automation`
- Password: `Fiber714Cvet`
- Hostname format: `C4-VIBRANT-<last three octets of MAC>`
- Wi-Fi power range: `5.0 - 20.5 dBm`

Security note: factory credentials are public and meant only for first setup.

Additional security note: HTTP Basic Auth is not encrypted on plain HTTP. Use this firmware only on trusted local networks/AP access.

## Arduino libraries

- ESP8266 core libraries (`ESP8266WiFi`, `ESP8266WebServer`) from ESP8266 board package 3.x
- `LittleFS`
- `ArduinoJson` 7.x

## File layout

- `VIBRANT.ino` — complete firmware sketch

## Build notes

1. Open `VIBRANT/VIBRANT.ino` in Arduino IDE (or compile with `arduino-cli` for an ESP8266 board).
2. Select a NodeMCU v3 compatible ESP8266 board profile.
3. Ensure `ArduinoJson` is installed.
4. Flash the firmware.

After boot, join the configured AP and open the device IP in a browser.

## Current networking behavior

- The configured `SSID`/`Password` are used for both SoftAP and Station connect attempts.
- Edited MAC address is applied to both SoftAP and Station interfaces.
- Wi-Fi power is clamped to a minimum of `5.0 dBm`.
- If station connectivity drops, the firmware periodically attempts reconnect.
- If storage or runtime recovery fails irrecoverably, the device logs the reason and restarts.
