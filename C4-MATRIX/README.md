# C4-MATRIX

ESP8266 NodeMCU v3 firmware for an 8x32 WS2812 LED matrix with a text-entry
web page with solid-color LED controls, LittleFS-persisted settings, SoftAP +
station Wi-Fi, ArduinoOTA, and
web firmware updates.

## Wiring

- Matrix data input to NodeMCU **D0 (GPIO16)** by default. The data pin can be
  changed to D0-D8 on `/config`.
- Power the 256-pixel matrix from an appropriately rated external **5 V**
  supply. Do not power the whole matrix from the NodeMCU 3.3 V pin.
- Connect the matrix supply ground and NodeMCU ground together.
- Add an approximately **330 Ω** resistor in series with the data line, near
  the first LED, and a suitably rated bulk capacitor (for example, 1000 µF,
  6.3 V or higher) across the matrix 5 V and ground.
- GPIO16/D0 uses the NeoPixel library's bit-banged output mode.

## Libraries and build

Install the ESP8266 board core and required libraries:

```sh
arduino-cli config init --overwrite
arduino-cli config set board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli lib install "ArduinoJson" "PubSubClient" "Adafruit NeoPixel"
```

Build from the directory whose name exactly matches the sketch:

```sh
cd C4-MATRIX
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries .
```

The firmware uses ArduinoJson 7, PubSubClient (installed to match the shared
VIBRANT toolchain), and Adafruit NeoPixel. PubSubClient is not used by this
project's sketch.

## Web interface

After boot, connect to the `C4-MATRIX-XXXXXX` SoftAP (where `XXXXXX` is the
last three MAC octets) or open the device's station IP. The default AP and
station credentials are `Fiber714Cvet` and the default station SSID is
`Z-Wave Automation`.

- `/` provides the text input. Submitted text is displayed and saved in
  LittleFS across reboots, returning the display to **text** mode.
  **All ON** fills all 256 LEDs with the last applied fill color (white by
  default); **All OFF** blanks the matrix without forgetting that color.
  **Red**, **Green**, and **Blue** fill the matrix immediately. Choose a
  **Custom RGB color** with the color picker and press **Apply** to fill it.
  These controls use `fetch` without reloading the page and show success/error
  feedback. Fill/off overrides text and scrolling without changing the saved
  text or scroll settings; all modes respect the configured brightness.
  Mode and fill color are restored after reboot.
- `/config` configures hostname, Wi-Fi power, station credentials, optional
  custom MAC, display pin/brightness/color/orientation, scroll direction and
  speed, ArduinoOTA, and debug logging. It also shows firmware version, IP,
  MAC, free heap, uptime, text, and scroll state. Configuration and update
  pages use HTTP Basic Authentication (`admin` / the configured AP password).
- `/update` accepts a compiled firmware `.bin` upload; the device reboots when
  the update succeeds.

## OTA

- **ArduinoOTA:** enabled by default. Select the configured hostname in the
  Arduino IDE network ports or upload with `espota.py`. The configured AP
  password is the OTA password. It can be disabled on `/config`.
- **Web OTA:** from `/config`, follow the **Web firmware update** link and
  upload the exported `.bin` firmware. Keep power connected until the device
  reboots.

## API

- `GET /api/status` returns version, active IP, MAC, free heap, uptime, current
  text, scroll state, `mode` (`text`, `fill`, or `off`), and `fillColor`
  (a `#RRGGBB` string).
- `POST /api/text` accepts JSON such as `{"text":"Hello"}`; text is limited to
  128 characters and saved to LittleFS.
- `POST /api/scroll` accepts JSON such as
  `{"enabled":true,"direction":"left","speed":80}`. Direction is `left` or
  `right`; speed is milliseconds per column from 10 through 1000.
- `POST /api/leds` accepts exactly one command: `{"state":"on"}`,
  `{"state":"off"}`, `{"color":"red"}`, `{"color":"green"}`,
  `{"color":"blue"}`, or `{"r":12,"g":200,"b":90}`. RGB values must be
  integers from 0 through 255. Unknown names, missing channels, mixed
  commands, and invalid values return HTTP 400 without changing the display.
  Successful commands return `{"ok":true}` and persist mode and fill color.
  Bodies larger than 512 bytes return HTTP 413; storage failures return 500.
  For example (replace the IP with your device's address):

  ```sh
  curl -H 'Content-Type: application/json' -d '{"state":"on"}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"color":"red"}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"r":12,"g":200,"b":90}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"state":"off"}' http://192.168.1.194/api/leds
  ```

## Configuration

Settings are stored as JSON at `/matrix_config.json` in LittleFS. Factory
reset is available on `/config`. Firmware version is defined in `Version.h`.
Firmware **1.0.1** adds the LED controls, persisted display modes, and LED API.
Additional LED command diagnostics follow the **Serial debug logging** toggle
on `/config`, so they can be disabled from the GUI.
