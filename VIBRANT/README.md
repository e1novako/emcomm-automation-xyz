# VIBRANT (NodeMCU v3 / ESP8266)

Arduino project for NodeMCU v3 with a web interface for controlling up to 16 outputs.

## Highlights

- All configuration is persisted in LittleFS JSON file: `/vibrant_config.json`
- English-only UI labels and source comments
- Main page lists configured devices (model, name, status) and provides checkbox toggles
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

## Default factory values

- SSID: `Z-Wave Automation`
- Password: `KoToTamoPeva2016`
- Hostname format: `C4-VIBRANT-<last three octets of MAC>`

## Arduino libraries

- ESP8266 core libraries (`ESP8266WiFi`, `ESP8266WebServer`)
- `LittleFS`
- `ArduinoJson`

## File layout

- `VIBRANT.ino` — complete firmware sketch

## Build notes

1. Open `VIBRANT/VIBRANT.ino` in Arduino IDE (or compile with `arduino-cli` for an ESP8266 board).
2. Select a NodeMCU v3 compatible ESP8266 board profile.
3. Ensure `ArduinoJson` is installed.
4. Flash the firmware.

After boot, join the configured AP and open the device IP in a browser.
