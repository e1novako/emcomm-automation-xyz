# C4-TCS (TCS_Intercom)

ESP8266 sniffer/transmitter for the TCS two-wire intercom bus, bridged to MQTT and Control4.

- Input GPIO14 (decoded in an ISR), output GPIO12 (bit-banged).
- Publishes decoded frames on `mqtt_topic`; accepts commands on `tcs_command_topic`; opens the door on `control4_command_topic`.
- Config in LittleFS `/config.json` (`mqtt_server`, `mqtt_port`, `mqtt_topic`, `mqtt_client`, `mqtt_user`, `mqtt_pass`, `control4_command_topic`, `control4_notify_topic`, `tcs_command_topic`); users in `/tcs_users.json`.

## Build

Sketch folder name must match the `.ino` name:

    arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries TCS_Intercom/

Libraries: ArduinoJson 7.x, PubSubClient, WiFiManager 2.0.17.

## Changelog

- 1.0.3: re-subscribe to MQTT topics after a WiFi reconnect; bounds-checked config/MQTT buffers (`strlcpy`, `snprintf`); NUL-terminated file reads, missing JSON keys no longer crash; control4/command topics are now saved and loaded from `config.json`.
