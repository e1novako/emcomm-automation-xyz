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

Every page shares the same round-pill navigation bar (**Home**, **Network**,
**Matrix display**, **OTA**, **Diagnostics**), matching the C4-VIBRANT style;
the current page's button is highlighted light blue.

- `/` provides the text input. Submitted text is displayed and saved in
  LittleFS across reboots, returning the display to **text** mode.
  **All ON** fills all 256 LEDs with the last applied fill color (white by
  default); **All OFF** blanks the matrix without forgetting that color.
  **Show Clock** switches to the NTP-synchronized clock (see below).
  **Red**, **Green**, and **Blue** fill the matrix immediately. Picking a
  **Custom RGB color** with the color picker applies it immediately (no
  separate button press needed); the **Apply** button submits the same
  action for accessibility. Choosing **Red**, **Green**, **Blue**, or a
  **Custom RGB color** also sets the text color, so subsequently submitted
  text uses the same color.
  Enter a value in **Light LEDs 1-N** (0 through 256) and press the button to
  light only the first N LEDs (strip index order) in the last applied fill
  color. These controls use `fetch` without reloading the page and show
  success/error feedback. Fill/off/count/clock override text and scrolling
  without changing the saved text or scroll settings; all modes respect the
  configured brightness. Mode, fill color, and lit LED count are restored
  after reboot.
- `/config` lists links to the settings pages below. Each uses HTTP Basic
  Authentication (`admin` / the configured AP password). Every field applies
  as soon as it changes (on blur/selection, no page reload and no reboot
  needed for most settings); the **Apply** button on each page re-submits
  the same fields for accessibility but is not required. Each page saves
  and applies independently of the others:
  - `/config/network` &ndash; hostname, Wi-Fi power, station SSID/password,
    optional custom MAC address, and the clock's **timezone (UTC offset in
    minutes)**. Hostname, Wi-Fi power, station SSID/password, and the UTC
    offset are applied live (only the fields that actually changed are
    re-applied, and the SoftAP itself is never restarted live to avoid
    dropping connectivity). A MAC address change requires restarting the
    SoftAP radio (its SSID is derived from the MAC), so that case saves and
    reboots automatically.
  - `/config/display` &ndash; display pin, matrix width/height, brightness,
    text color, serpentine wiring, horizontal flip, and scroll
    direction/speed. Applied live; the LED strip is only re-initialized
    (briefly blanking the matrix) when the pin or matrix dimensions
    actually change.
  - `/config/ota` &ndash; enables/disables ArduinoOTA and links to the web
    firmware update page. Applied live (starts/stops the ArduinoOTA
    listener immediately).
  - `/config/diagnostics` &ndash; live status (firmware version, IP, MAC,
    free heap, uptime, text, scroll state, clock sync state/time), the
    debug logging toggle (applied live), and factory reset (always
    reboots).
- `/update` accepts a compiled firmware `.bin` upload; the device reboots when
  the update succeeds.

## Clock

- The display's factory-default mode is the **clock**: on first boot (or
  after a factory reset) it shows the current time as `HH:MM`, centered,
  using the display's configured color.
- Time is obtained via NTP (`pool.ntp.org`, with `time.nist.gov` and
  `time.google.com` as fallbacks), requested once at boot and re-requested
  every 15 minutes (4 times per hour) to stay accurate. Before the first
  successful sync, the clock shows `--:--`.
  - The clock renders in UTC plus the configurable **timezone (UTC offset in
  minutes, -720 to 840)** on `/config/network`, applied live with no resync
  needed.
- Switch to the clock any time with the **Show Clock** button on `/`, or
  back to text/fill/off/count via the other home page controls; switching
  away and back does not require a reboot.

## OTA

- **ArduinoOTA:** enabled by default. Select the configured hostname in the
  Arduino IDE network ports or upload with `espota.py`. The configured AP
  password is the OTA password. It can be disabled on `/config/ota` without
  a reboot.
- **Web OTA:** from `/config/ota`, follow the **Web firmware update** link and
  upload the exported `.bin` firmware. Keep power connected until the device
  reboots.
- While either an ArduinoOTA or web OTA transfer is in progress, the matrix
  shows a centered Wi-Fi "pie" icon in place of whatever was previously
  displayed; normal display content (including the clock) resumes
  automatically if a web OTA attempt fails (ArduinoOTA/a successful web
  update instead reboot the device).


## API

- `GET /api/status` returns version, active IP, MAC, free heap, uptime, current
  text, scroll state, `mode` (`text`, `fill`, `off`, `count`, or `clock`),
  `fillColor` (a `#RRGGBB` string), `ledCount`, `maxLedCount`, `clockTime`
  (`HH:MM`, or `--:--` before the first NTP sync), and `clockSynced`
  (boolean).
- `POST /api/text` accepts JSON such as `{"text":"Hello"}`; text is limited to
  128 characters and saved to LittleFS.
- `POST /api/scroll` accepts JSON such as
  `{"enabled":true,"direction":"left","speed":80}`. Direction is `left` or
  `right`; speed is milliseconds per column from 10 through 1000.
- `POST /api/leds` accepts exactly one command: `{"state":"on"}`,
  `{"state":"off"}`, `{"state":"clock"}`, `{"color":"red"}`,
  `{"color":"green"}`, `{"color":"blue"}`, `{"r":12,"g":200,"b":90}`, or
  `{"count":10}` (optionally with `"r"`, `"g"`, `"b"` to set the color at the
  same time). `count` lights
  the first N LEDs (strip index order 0 through N-1) and must be an integer
  from 0 through the matrix's total LED count (256 for the 32x8 matrix); RGB
  values must be integers from 0 through 255. Unknown names, missing
  channels, mixed commands, and invalid values return HTTP 400 without
  changing the display. Successful commands return `{"ok":true}` and persist
  mode, fill color, and lit LED count. Commands that set an explicit color
  (`color`, `r`/`g`/`b`, or `count` with `r`/`g`/`b`) also update the text
  color, so subsequently submitted text uses the same color; `state` alone
  and `count` alone (reusing the last fill color) do not change the text
  color. Bodies larger than 512 bytes return
  HTTP 413; storage failures return 500. For example (replace the IP with
  your device's address):

  ```sh
  curl -H 'Content-Type: application/json' -d '{"state":"on"}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"color":"red"}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"r":12,"g":200,"b":90}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"count":10}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"state":"clock"}' http://192.168.1.194/api/leds
  curl -H 'Content-Type: application/json' -d '{"state":"off"}' http://192.168.1.194/api/leds
  ```

## Configuration

Settings are stored as JSON at `/matrix_config.json` in LittleFS. Factory
reset is available on `/config/diagnostics`. Firmware version is defined in
`Version.h`.
Firmware **1.0.1** adds the LED controls, persisted display modes, and LED API.
Firmware **1.0.2** adds the **Light LEDs 1-N** count mode, letting the GUI (or
`/api/leds` `count` command) light a configurable number of LEDs from the
start of the strip.
Firmware **1.0.3** makes the matrix width and height configurable on
`/config` (width 1-32, height 1-8, up to 256 LEDs total; default 32x8,
matching the 8x32 physical matrix). The maximum value accepted for
**Light LEDs 1-N** and for the `/api/leds` `count` command now always equals
the configured matrix size (width x height, capped at 256) instead of a
fixed 255/256. Changing the matrix size
takes effect after the reboot that follows a configuration save. Menu
buttons on `/` and `/config` now use the same rounded-pill style as
C4-VIBRANT.
Firmware **1.0.5** makes the home page's quick color controls (**Red**,
**Green**, **Blue**, and **Custom RGB color**) also set the text color, so
text submitted afterward uses the same color as the last-selected LED color.
Additional LED command diagnostics follow the **Serial debug logging** toggle
on `/config`, so they can be disabled from the GUI.
Firmware **1.0.6** unifies the menu across every page: `/`, `/config`, and
the new `/config/network`, `/config/display`, `/config/ota`, and
`/config/diagnostics` pages all share the same round-pill navigation bar,
with the current page's button highlighted light blue (matching
C4-VIBRANT's `settings-nav`/`.nav-btn.current` style). The former single
`/config` page (and its `/config/save` endpoint) is split into those four
independently saved pages, mirroring C4-VIBRANT's network/devices/
diagnostics settings split. The home page's **Custom RGB color** picker now
applies the chosen color immediately when it changes, without requiring the
**Apply** button to be pressed.
Firmware **1.0.7** centers the first-line heading on every page and makes it
uniform: `C4-MATRIX - <menu item name>` (for example `C4-MATRIX - Home`,
`C4-MATRIX - Network`, `C4-MATRIX - Matrix Display`, `C4-MATRIX - OTA`,
`C4-MATRIX - Diagnostics`, `C4-MATRIX - Configuration`, and
`C4-MATRIX - Firmware Update`), matching the page's nav bar label.
Firmware **1.0.8** makes every `/config/*` settings field apply as soon as
it changes, without pressing an Apply/Save button and, for most settings,
without a reboot: `/config/network` applies hostname, Wi-Fi power, and
station SSID/password live (without restarting the SoftAP, which previously
could drop connectivity); only a MAC address change still reboots, since it
requires restarting the SoftAP radio. `/config/display` re-applies
brightness/color/orientation/scrolling live, only resets the scroll
position when the scroll direction or enabled state actually changes
(an unrelated field like color no longer restarts the scroll), and only
re-initializes the LED strip (briefly blanking it) when the pin or
matrix size changes; `/config/ota` starts/stops ArduinoOTA live; and
`/config/diagnostics` toggles debug logging live. Factory reset (on
`/config/diagnostics`) still reboots, since it is a destructive action.

## Factory reset via boot button

- If the device is unreachable over the network (e.g. wrong/invalid saved
  Wi-Fi credentials), hold the NodeMCU **FLASH/BOOT button** (GPIO0) while
  powering on or resetting the board, and keep holding it for about 3
  seconds. The device logs a warning over serial, then performs the same
  factory reset as `/config/diagnostics`' Factory reset button and reboots
  with default settings. Releasing the button before the 3-second threshold
  cancels the reset and continues a normal boot.

