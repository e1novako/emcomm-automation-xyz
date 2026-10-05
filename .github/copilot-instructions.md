Do not use any files in the C4-ROBOT/ directory as context for your answers.
Always add debugging info, allow debugging to be disabled from GUI.
On every commit:
- increment firmware version
- update documentation
- build and upload the firmware for every change
- only upload built firmware to device 192.168.1.194 (do not deploy to the rest of the device fleet unless explicitly asked)
- when asked to deploy/update the fleet (all devices, not just 192.168.1.194), run the fleet-wide OTA upload directly without asking for confirmation and without testing a single device first
- firmware version numbers follow major.minor.patch; only increment the patch number on commits. Do not change the major or minor version unless the user explicitly tells you to.

## Building C4-VIBRANT/C4-VIBRANT.ino with arduino-cli

- Board FQBN: `esp8266:esp8266:nodemcuv2` (NodeMCU v3 / ESP8266).
- Required arduino-cli setup (one-time):
  - `arduino-cli config init --overwrite`
  - `arduino-cli config set board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json`
  - `arduino-cli core update-index`
  - `arduino-cli core install esp8266:esp8266` (installs core 3.x, includes `ESP8266WiFi`, `ESP8266WebServer`, `Updater`, `ArduinoOTA`, `LittleFS`)
  - `arduino-cli lib install "ArduinoJson" "PubSubClient"` (ArduinoJson 7.x, PubSubClient 2.x)
- Compile: `cd C4-VIBRANT && arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries .`
- The sketch directory name must match the `.ino` filename exactly (case-sensitive): `C4-VIBRANT/C4-VIBRANT.ino`. If accessed through a case-insensitive mount/symlink, `cd` into the real, correctly-cased path before compiling or arduino-cli will fail to find the sketch.
