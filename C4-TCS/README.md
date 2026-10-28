# C4-TCS (TCS_Intercom)

ESP8266 sniffer/transmitter for the TCS two-wire intercom bus, bridged to MQTT and Control4.

- Input GPIO14 (decoded in an ISR), output GPIO12 (bit-banged).
- Publishes decoded frames on `mqtt_topic`; accepts commands on `tcs_command_topic`; opens the door on `control4_command_topic`.
- Config is stored in NVRAM (EEPROM emulation, its own flash sector with CRC; survives firmware/FS updates). A legacy LittleFS `/config.json` is migrated on first boot. Keys: `mqtt_server`, `mqtt_port`, `mqtt_topic`, `mqtt_client`, `mqtt_user`, `mqtt_pass`, `control4_command_topic`, `control4_notify_topic`, `tcs_command_topic`, `tz`, `ntp_server`, `debug`, `serial_number` (hex), `wifi_ssid`, `password`. Users stay in LittleFS `/tcs_users.json`.

## Build

Sketch folder name must match the `.ino` name:

    arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries TCS_Intercom/

Libraries: ArduinoJson 7.x, PubSubClient, WiFiManager 2.0.17.

## Password, WiFi and OTA

The WiFi password is the one password: it protects the web GUI (HTTP basic auth, any user name, only the password is checked), OTA, and is used to join WiFi. By default (`password` empty) the device uses the WiFi password already stored in it. Set `wifi_ssid` plus a password (8-63 chars) in Settings to join another network, or tick "Use the WiFi password already stored" to clear the override. If joining the configured SSID fails (or it is blank) the device falls back to WiFiManager and its config AP `TCS-<chipid>` (password = the same, or `intercom` if none is known). OTA upload: `espota.py -i <ip> -p 8266 -a <wifi password> -f <bin>`. Basic auth is cleartext over HTTP; an open (password-less) network leaves GUI and OTA unprotected.

## Backup / restore

Web GUI pages, reached from a rounded-button menu at the top: Status (`/`), WiFi (`/wifi`), MQTT (`/mqtt`), Time (`/time`), Backup/Restore (`/backup`). Saving a page writes NVRAM and reboots. Backup/Restore: download the JSON (`/backup/download`, includes passwords) or upload one to `/restore`; it is applied, saved and the device reboots. All pages need the password.

## Door-open address

The door station address used for the Control4 open-door toggle is tracked dynamically from door-related bus frames (door call, talking, door opened/closed, end of ringtone, door readiness, init). If the controller changes address under bus congestion, the next such frame updates it. `tcs_command` messages without an `address` also use it. `address`, `payload` and `serial_number` in command messages are hex strings (or numbers).

## Web GUI / debug

`http://<device>/` shows status (MQTT, tracked door address, dropped frames) and a button to turn serial debug logging on/off; the setting persists in `config.json` (`debug`). `tz` in `config.json` is a POSIX TZ string (default CET/CEST).

## MQTT topics

- `<mqtt_topic>`: decoded/sent frames and events (`sniffer_online`, `control4_notify`).
- `<mqtt_topic>/availability`: retained `online` / `offline` (last will).
- `<mqtt_topic>/status`: every 60 s: version, uptime, RSSI, free heap, RX overruns, TX dropped/queued, door address.
- Control4 IAS zone: Alarm1 on an internal call to this device's serial, Alarm2 on a door call to it.

## Transmit

Frames are queued (4 deep, 20 ms gap) and bit-banged by hardware timer1, so `loop()` never blocks. RX is ignored while transmitting.

## Tests

Host-side protocol round-trip tests (no hardware needed):

    g++ -std=c++17 -Itest/stubs -I. test/test_protocol.cpp tcs_protocol.cpp -o /tmp/tcs_test && /tmp/tcs_test

## Changelog

- 1.0.9: web GUI split into WiFi, MQTT, Time and Backup/Restore pages with a rounded-button top menu; `ntp_server` setting; `/config` removed, download moved to `/backup/download`.
- 1.0.8: the password is the WiFi password (default: the stored one), no user name; the `intercom` default is only the fallback AP password.
- 1.0.7: single password for GUI/OTA/WiFi; configurable SSID; all settings in NVRAM; settings backup/restore; GUI behind basic auth; `serial_number` setting; AP name `TCS-<chipid>`.
- 1.0.6: restored `stop_talking(address)` helper.
- 1.0.5: timer-driven non-blocking TX with queue; bounded JSON building and 512-byte MQTT buffer; availability (last will) and periodic status topics; stable MQTT client ID; `DEVICE_TEST` plus buildable `FOUND_DEVICE`/`FOUND_DEVICE_SUBSYSTEM`; `FLOOR_CALL` carries an address; Control4 notify carries alarm/address/serial and Alarm2 on door calls; `/config` web page for all settings (saves and reboots; no auth); dead code removed; host protocol tests.

- 1.0.4: dynamic door address; non-blocking WiFi/MQTT reconnect and NTP; ISR frame snapshot with overrun counter; frame length taken from the bus (long/short) and used when sending; `OPEN_DOOR_LONG` parsed distinctly; hex-only payload parsing, no malloc in callback; `address`/`payload` always published; configurable TZ; debug toggle web GUI; users buffer 8 KB; header/implementation overloads fixed; HA discovery uses `value_template` and list identifiers.

- 1.0.3: re-subscribe to MQTT topics after a WiFi reconnect; bounds-checked config/MQTT buffers (`strlcpy`, `snprintf`); NUL-terminated file reads, missing JSON keys no longer crash; control4/command topics are now saved and loaded from `config.json`.
