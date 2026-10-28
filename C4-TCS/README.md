# C4-TCS (TCS_Intercom)

ESP8266 sniffer/transmitter for the TCS two-wire intercom bus, bridged to MQTT and Control4.

- Input GPIO14 (decoded in an ISR), output GPIO12 (bit-banged).
- Publishes decoded frames on `mqtt_topic`; accepts commands on `tcs_command_topic`; opens the door on `control4_command_topic`.
- Config in LittleFS `/config.json` (`mqtt_server`, `mqtt_port`, `mqtt_topic`, `mqtt_client`, `mqtt_user`, `mqtt_pass`, `control4_command_topic`, `control4_notify_topic`, `tcs_command_topic`); users in `/tcs_users.json`.

## Build

Sketch folder name must match the `.ino` name:

    arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries TCS_Intercom/

Libraries: ArduinoJson 7.x, PubSubClient, WiFiManager 2.0.17.

## Door-open address

The door station address used for the Control4 open-door toggle is tracked dynamically from door-related bus frames (door call, talking, door opened/closed, end of ringtone, door readiness, init). If the controller changes address under bus congestion, the next such frame updates it. `tcs_command` messages without an `address` also use it. `address`, `payload` and `serial_number` in command messages are hex strings (or numbers).

## Web GUI / debug

`http://<device>/` shows status (MQTT, tracked door address, dropped frames) and a button to turn serial debug logging on/off; the setting persists in `config.json` (`debug`). `tz` in `config.json` is a POSIX TZ string (default CET/CEST).

## Changelog

- 1.0.4: dynamic door address; non-blocking WiFi/MQTT reconnect and NTP; ISR frame snapshot with overrun counter; frame length taken from the bus (long/short) and used when sending; `OPEN_DOOR_LONG` parsed distinctly; hex-only payload parsing, no malloc in callback; `address`/`payload` always published; configurable TZ; debug toggle web GUI; users buffer 8 KB; header/implementation overloads fixed; HA discovery uses `value_template` and list identifiers.

- 1.0.3: re-subscribe to MQTT topics after a WiFi reconnect; bounds-checked config/MQTT buffers (`strlcpy`, `snprintf`); NUL-terminated file reads, missing JSON keys no longer crash; control4/command topics are now saved and loaded from `config.json`.
