# C4-VIBRANT (NodeMCU v3 / ESP8266)

Arduino project for NodeMCU v3 with a web interface for controlling up to 16 outputs.

## WARNING

Factory Wi-Fi credentials are publicly known:

- SSID: `Z-Wave Automation`
- Password: `Fiber714Cvet`

Change the password immediately in **Network, Wi-Fi & MQTT** after the first
boot.

## Highlights

- All configuration is persisted in LittleFS JSON file: `/vibrant_config.json`
- English-only UI labels and source comments
- Main page lists configured devices (model, name, status) and provides rounded state buttons (gray when ON, pastel yellow when OFF), per-output load action buttons, and bulk actions (`Leave Mesh All`, `Factory Reset All`)
- Settings, toggle actions, and config maintenance endpoints are protected with HTTP Basic Auth (`admin` / current Wi-Fi password)
- Settings are split into three navigable pages linked from the main page and from each other:
  - **Network, Wi-Fi & MQTT** (`/settings/network`): station/AP credentials, MAC, hostname, Wi-Fi power, and MQTT broker settings
  - **Devices & output configuration** (`/settings/devices`): output count, device names/models/manufacturers, and GPIO assignments
  - **Diagnostics & OTA** (`/settings/diagnostics`): ArduinoOTA, verbose serial debug, configuration import/export/reset, and firmware update access
- Saving any page only updates that page's settings; it does not clear checkboxes, credentials, or values on other pages. Station/AP and MQTT passwords are never pre-filled: leave them blank to keep the existing password; use the MQTT clear-password checkbox to remove broker authentication.
- The Network page supports:
  - MAC address with an opt-in **Use custom MAC address** checkbox (off by default, including older saved configurations)
  - DHCP hostname (up to 32 letters, digits or hyphens; empty uses the MAC-based default)
  - MQTT broker host, port, username, and password (enable/disable toggle)
- The Devices page supports:
  - Bulk-copying the first Manufacturer/Model/Name to all visible rows (`#<number>` in the first name continues from the parsed starting number)
  - Clearing all Manufacturer, Model, and Name fields without changing Control output selections
  - Reversing GPIO assignments across the currently configured output rows
  - Up to 16 device entries (`manufacturer`, `model`, `name`, output `D0`–`D8`, `RX`, `TX`, or `none`)
  - Duplicate GPIO assignments among active outputs are rejected. TX/RX disable serial communication; GPIO0 (FLASH) and GPIO15 can affect boot, and remain selectable with warnings.
  - Unspecified device identity fields default to manufacturer `Control4` and model `Vibrant`
- Configuration maintenance routes:
  - Export backup (`/config/export`)
  - Import backup (`/config/import`)
  - Factory reset to defaults (`/config/factory-reset`)
- Web-based OTA firmware update (`/firmware/update`): upload a compiled `.bin` directly from the browser; the device reboots automatically after a successful flash
- ArduinoOTA support (enabled by default): developer/service OTA uploads via Arduino IDE or OTA-capable tooling; it can be disabled on **Diagnostics & OTA**
- Serial diagnostics print boot progress, Wi-Fi state, configured outputs, and important error/status messages
- Verbose `[DEBUG]` MQTT, Stickserver, ArduinoOTA, and settings-save logs can be disabled with **Enable verbose serial debug logging** on **Diagnostics & OTA** (off by default); boot Wi-Fi scan and detailed loaded Wi-Fi logging run only when debug is enabled
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

### Stickserver reservations

Stickserver `reserve` requests match the requested `ntype` case-insensitively against each comma-separated entry in the output's configured Model field. Whitespace around each entry is ignored, so a value such as `ABC123, XYZ-9, foo_bar` supports all three model names. Device information reports the configured Manufacturer and raw Model value, and the reported `ntype` is the Model value.

### Configuration

Enable MQTT and set the broker host/port in **Network, Wi-Fi & MQTT**. Fields:

| Field | Description |
|-------|-------------|
| Enable MQTT | Enables MQTT connectivity |
| MQTT server host | Broker IP or hostname (e.g. `192.168.1.6`) |
| MQTT port | Broker port (default `1883`) |
| MQTT user | Optional broker username |
| MQTT password | Optional broker password |

### Topics

Base: `<STICKSERVER_ROOT_TOPIC>` (see `Stickserver.cpp`)

Device control is handled exclusively via the Stickserver protocol: `hello`, `list`, `reserve`, `release`, `status`, `join`, `leave`, `power_on`, `power_off`, `leave_mesh`/`factory_reset`, and `reboot`. The legacy native `vibrant/<hostname>/out/<N>/set|action|state` topics were removed in 1.4.10 since the Stickserver protocol already covers identical functionality.

### Fleet discovery

The device passively discovers other stickserver instances on the broker by periodically broadcasting `hello` on the root topic and requesting `list` from each instance it learns about. Discovered instances and their outputs are shown on the **All Outputs** page (`/fleet`), one column per stickserver, one row per output. Clicking an output's button sends a `power_on`/`power_off` command directly to that stickserver's instance topic to toggle it.

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
1. If initially OFF, turn ON for 8 s of preparation.
2. Cycle power **5 times**: 1 s OFF → 1.5 s ON per cycle.
3. Wait 8 s (bulb turns green).
4. Trigger: 1 s OFF → 10 s ON (cycles power while bulb is green).

Total sequence duration: ~31.5 s if initially ON, ~39.5 s if initially OFF.

### Factory reset sequence (connected bulb)

Starting with the bulb powered on:
1. If initially OFF, turn ON for 8 s of preparation.
2. Cycle power **13 times**: 1 s OFF → 1.5 s ON per cycle.
3. Wait 8 s (bulb transitions from 1800 K/red to blue).
4. Trigger: 1 s OFF → 10 s ON (cycles power while bulb is blue).

Total sequence duration: ~51.5 s if initially ON, ~59.5 s if initially OFF.

### Non-blocking execution

All GPIO activity (including the timed cycling sequences) runs in the background via a `millis()`-based state machine in the main loop. The web UI and MQTT connection remain fully responsive during any running sequence. The running action is shown in a banner on the main page (auto-refreshes every 3 s) and can be cancelled at any time.

For `Leave Mesh All` and `Factory Reset All`, all managed outputs prepare and cycle in parallel. Direct ON/OFF commands cancel the sequence when they target an owned output; bulk ON/OFF requests are rejected while an action runs.

## Default factory values

- SSID: `Z-Wave Automation`
- Password: `Fiber714Cvet`
- Hostname format: `C4-VIBRANT-<last three octets of MAC>`
- Wi-Fi power range: `5.0 - 20.5 dBm`
- MQTT: disabled by default
- Default GPIO assignments: outputs 1–8 mapped to D0–D7 (GPIO16, GPIO5, GPIO4, GPIO0, GPIO2, GPIO14, GPIO12, GPIO13)
- ArduinoOTA: enabled by default; disable it from **Diagnostics & OTA** when not needed

Security note: factory credentials are public and meant only for first setup.

Additional security note: HTTP Basic Auth is not encrypted on plain HTTP. Use this firmware only on trusted local networks/AP access.

## Release notes

### 1.6.1

- Extended the hello broadcast-suppression idea from 1.6.0 to the per-server "list" request too: every stickserver already observes every "list" request/response addressed to any peer via the shared wildcard subscription, so a device now skips re-requesting "list" from a given peer if a request or response for that same peer was observed (by anyone, not just itself) within the last 10-second window, instead of blindly polling on its own fixed timer regardless of what others already asked.

### 1.6.0

- Reduced MQTT discovery traffic: previously every stickserver independently broadcast its own "hello" discovery request every 15 seconds, so an N-device fleet generated up to N redundant broadcasts (and N sets of responses) per cycle even though every device already observes every response on the shared bus regardless of who asked. Each device now tracks the last time *any* hello command was seen on the bus (its own or a peer's) and only issues its own broadcast if none has been observed in the last 5 seconds, collapsing the fleet down to one hello broadcast per 5-second window.
- All Outputs page: each column header is now a link to `http://<stick_server_ip>` (opens in a new tab) for stickservers with a known IP, for quick access to that device's own UI.
- Replaced plain text menu links with button-styled links (`.nav-btn`) on every page (main output control, all settings pages, All Outputs), keeping the existing `|` separators between them.

### 1.5.2

- Fixed ArduinoOTA (network/IDE firmware upload) typically needing 2-3 retries to succeed. Root cause: `loop()` called the web server, MQTT, and stickserver discovery on every iteration regardless of an in-flight OTA transfer, and the web server's chunked-write retry loop can legitimately block for up to several seconds under backpressure -- either of these could starve `ArduinoOTA.handle()` of CPU time long enough for `espota`'s upload to time out mid-transfer. `loop()` now pauses the web server/MQTT/discovery entirely while an OTA transfer is active (from `onStart` to `onEnd`/`onError`), and the web server's blocking write-retry loop now opportunistically services `ArduinoOTA.handle()` too, so an OTA invitation/transfer in progress is never starved even by a slow concurrent page load.

### 1.5.1

- All Outputs page columns are now sorted by ascending IP address (numeric, so `.9` sorts before `.10`), instead of arbitrary discovery/slot order. This firmware's own `hello`/`list` responses now include an `"ip"` field (`WiFi.localIP()`); the field is parsed into each discovered server's record when present. Third-party stickservers that don't report an IP sort after all known-IP columns, falling back to hostname ordering among themselves.

### 1.5.0

- Found why output state color never showed on the All Outputs page: the `.output-toggle`/`.output-on`/`.output-off` button classes were only ever defined in the home page's stylesheet, not the settings-page stylesheet that `/fleet` (and all other `/settings/*` pages) actually use, so the buttons rendered with no state styling at all. Those classes (and centered `th`/`td` text) are now part of the shared settings-page stylesheet.
- Removed the "Output #" row-number column from the All Outputs table; all cells are centered.

### 1.4.20

- The hide-until-loaded change in 1.4.19 ruled out progressive-rendering as the cause -- the user still saw missing data on a fully-loaded page, with a consistent amount missing per row on some loads. The likely real cause: building a whole table row (up to 16 columns of `<form>` markup, several KB) as one `String` via repeated `+=` concatenation is exactly the kind of allocation pattern that can hit heap fragmentation on this memory-constrained device, where a failed/short reallocation can silently truncate the string. Each table cell is now built and sent as its own small, independently-written chunk (with a small `reserve()` up front) instead of accumulating a whole row first, cutting the peak String size needed per chunk from several KB down to under ~400 bytes.
- Added free heap and heap fragmentation percentage to the Diagnostics & OTA page, to help diagnose memory-pressure-related issues going forward.

### 1.4.19

- Found the actual source of the still-reported "missing/corrupted data" on the All Outputs page: it wasn't corruption at all. The page streams as many HTTP chunks, and with `table-layout:fixed` a browser can render the table progressively as each chunk arrives; on a slow or weak WiFi link, a user looking at the page mid-load would see rows/cells that simply hadn't arrived yet -- which looked exactly like randomly missing data, varying with network speed and timing. The table is now built hidden behind a "Loading discovered outputs…" message and only revealed (by a trailing inline script) once the entire table has been fully received, so the page can never be viewed in a partially-loaded state.

### 1.4.18

- The 1.4.17 mitigation (longer write timeout, `TCP_NODELAY`, fewer chunk writes) reduced but did not eliminate the chunked-encoding corruption on busy pages: testing still showed an occasional short write silently truncating a chunk's actual byte count below its already-declared size. Chunked table/page responses no longer use `ESP8266WebServer::sendContent()` for the body at all; each chunk (size header, payload, and trailer) is now written directly to the client through a small retry loop that keeps writing until every byte is confirmed sent (or the socket is truly disconnected/stalled), so a declared chunk size can never again mismatch what was actually delivered.

### 1.4.17

- Fixed the real root cause of the intermittent blank/missing cells on the All Outputs page: `ESP8266WebServer::sendContent()` has only a 1-second default write timeout and does not retry on a short write, so under momentary WiFi/TCP congestion it could write fewer bytes than the already-declared HTTP chunk size, desyncing the chunked-transfer framing (symptoms: well-formed but randomly truncated/merged table rows, worse on a busy network). The chunked response helpers now raise the client's write timeout to 8s and enable `TCP_NODELAY` before streaming a page, and each table row/piece is sent as a single `sendContent()` call instead of being needlessly re-split into 256-byte sub-writes (which only multiplied the number of at-risk operations).

### 1.4.16

- Fixed a bug where the All Outputs page intermittently rendered blank `<td>` cells for outputs that had previously been discovered. The discovery output table is now merged by `euid` instead of being wholesale overwritten on each `hello`/`list` response, so a short/partial reply from one stickserver instance no longer erases already-known outputs for that column. Outputs are only fully cleared when their parent server entry goes stale (no `hello`/`list` activity for ~90s) and is pruned.

### 1.4.15

- Stickserver discovery now parses the per-output `devices` array (euid/name/state) from `hello` responses, not just `list` responses, so a discovered output's state on the **All Outputs** page updates as soon as any hello reply carries it (some stickserver implementations include it there). Our own `hello` response now includes this `devices` array too, for parity.
- On the All Outputs page, each button's color (yellow = ON, gray = OFF) now reflects the most recently received `state` value from either `hello` or `list` discovery responses.

### 1.4.14

- Renamed the "Fleet outputs" page to "All Outputs" (route still `/fleet`); updated nav labels and page title.
- Added `|` separators between navigation links on every settings page, matching the main page's link style.
- All Outputs page buttons are now live controls: clicking a button publishes a Stickserver `power_on`/`power_off` command (toggling the output) directly to that device's MQTT instance topic, using the last known state (from passive `hello`/`list` discovery) to decide the toggle direction. New `POST /fleet/toggle` route.

### 1.4.13

- Chunked HTML responses (`/`, `/settings/devices`, `/fleet`) are now streamed section-by-section (and row-by-row for tables) directly to the network as each piece is built, instead of assembling the full page into one `String` and splitting it afterwards. This bounds peak RAM usage to roughly one row's worth of HTML regardless of how many outputs/stickservers are configured/discovered.

### 1.4.12

- Reservation column button now shows just the `<owner>` (no "Reserved by" prefix), is disabled when the output is not reserved, and is enabled when reserved. Clicking it releases that output's reservation (processed in-process via the Stickserver `release` command, with the response still published over MQTT when connected).
- Stickserver `release` command: sending it with no `owner` and an empty/omitted `euids` list now releases all currently-reserved managed outputs instead of returning an `invalid_member` error.
- All HTML page responses (`/`, `/settings/*`, `/fleet`, config export, firmware update page) are now sent using HTTP chunked transfer-encoding (`chunkedResponseModeStart`/`sendContent`/`chunkedResponseFinalize`) instead of a single buffered `Content-Length` response.

### 1.4.11

- Added a new "Fleet outputs" page (`/fleet`, linked from the main page and settings nav) showing all discovered stickserver instances in a table: each column is one stickserver, each row is one output slot, rendered as a read-only button (gray = off, yellow = on).
- Added passive stickserver discovery: the device now subscribes to the Stickserver root topic plus a wildcard on everything beneath it, periodically broadcasts `hello` on the root topic, and requests `list` from each discovered instance to populate its outputs. Entries are pruned after ~90s of inactivity.
- Hardened `handleStickserverMessage` so commands addressed to other instances (observed via the new wildcard subscription) are never answered on our own topic; only responses are used for discovery.

### 1.4.10

- Removed the legacy native `vibrant/<hostname>/out/<N>/set|action|state` MQTT topics (publish and subscribe). Device control now goes exclusively through the Stickserver protocol, which already implements identical functionality (confirmed in 1.4.9).

### 1.4.9

- Added a "Reboot device" button on the Diagnostics & OTA settings page.
- Added a live device uptime timer (hh:mm:ss, counted from boot) shown on the main page and the Diagnostics & OTA page.
- Verified MQTT command parity with `example.ino`: `hello`, `list`, `reserve`, `release`, `status`, `join`, `leave`, `power_on`, `power_off`, `leave_mesh`, `factory_reset`, and `reboot` were already fully implemented via the native per-output topics and the Stickserver protocol; no gaps found.

### 1.4.8

- Added a "Reservation" column to the main page showing "Reserved by &lt;owner&gt;" (gray when not reserved, yellow when reserved).
- When an output is reserved, its Output toggle, Leave Mesh, and Factory Reset buttons are disabled.
- When all managed outputs are reserved, the bulk Turn ON all/Turn OFF all/Leave Mesh All/Factory Reset All buttons are disabled.

### 1.4.7

- Main page table width is now 60% of the screen, capped at a maximum of 1200px.

### 1.4.6

- Main page table: all cell data is now centered; Manufacturer, Model, and Name columns share equal width, while Name, Output, and Actions columns are narrower.

### 1.4.5

- Unified the appearance of all buttons (Save, Cancel, bulk actions, Leave Mesh, Factory Reset, Output toggle) to the same pill-shaped style; only the Output toggle's ON/OFF background color still differs to indicate state.

### 1.4.4

- Fixed Output column button colors: OFF is now gray, ON is now yellow.

### 1.4.3

- Renamed the project folder and sketch from `VIBRANT` to `C4-VIBRANT`.

### 1.4.2

- Added a Manufacturer column to the main page, alongside Model and Name.

### 1.4.1

- Removed the redundant Status column (the Output column already shows ON/OFF) and centered its button.
- Removed the per-output On/Off action buttons, keeping only Leave Mesh and Factory Reset.
- Restyled the Leave Mesh and Factory Reset buttons to match the Output column's pill-button appearance.

### 1.4.0

- Split configuration into Network/Wi-Fi/MQTT, Devices/outputs, and Diagnostics/OTA pages with independent saves and shared navigation.
- Replaced main-page output checkboxes with accessible rounded buttons showing the current ON/OFF state and requesting its opposite.
- Added diagnostics-page access to verbose-debug control, ArduinoOTA, maintenance actions, and web firmware upload; documented the `arduino-cli` ESP8266 build setup.

### 1.3.0

- Split the firmware into focused source modules while preserving runtime behavior.

### 1.2.6

- Added opt-in custom MAC application, hostname/GPIO validation and pin warnings.
- Unified load-action timing profiles, fixed all-output ownership and factory-reset trigger wait.
- Applied Wi-Fi, output, MQTT and ArduinoOTA changes after settings updates, config imports and factory resets.
- Made verbose debug output and boot Wi-Fi scanning conditional on the Diagnostics switch.

### 1.2.5

- Defaulted unspecified device manufacturers to `Control4` and models to `Vibrant`.

### 1.2.4

- Added case-insensitive matching for comma-separated Model lists in Stickserver reservations.
- Enabled ArduinoOTA by default; it remains configurable from the Settings page.

## MQTT

When MQTT is enabled in **Network, Wi-Fi & MQTT**, the firmware:

- Connects to the configured broker on boot and reconnects automatically every 15 seconds if the connection is lost.
- Publishes the retained state of each output to `vibrant/<hostname>/output/<N>/state` (`1` = ON, `0` = OFF) whenever a toggle is applied (from the web UI or MQTT).
- Subscribes to `vibrant/<hostname>/output/<N>/set` for each output. Send `1`, `ON` (case-insensitive), or `true` (case-insensitive) to turn on; `0` or any other value to turn off.

MQTT username and password are optional (leave blank for anonymous access). The MQTT password field is never pre-filled in the form; leave it empty to keep the current stored password.

## Firmware update (OTA)

C4-VIBRANT supports two OTA firmware update paths:

- **Web UI OTA upload (primary/recommended)**
- **ArduinoOTA (secondary developer/service path, enabled by default)**

### Web UI firmware upload (primary method)

1. Compile `C4-VIBRANT/C4-VIBRANT.ino` for your NodeMCU board to produce a `.bin` file.
   - Arduino IDE: **Sketch → Export Compiled Binary**
   - `arduino-cli`: `arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries C4-VIBRANT/C4-VIBRANT.ino`
2. Open the device web UI and log in.
3. Go to **Diagnostics & OTA → Open firmware update page**, or navigate directly to `http://<device-ip>/firmware/update`.
4. Select the compiled `.bin` file and click **Upload and flash**.
5. Wait for the upload to complete. The device reboots automatically.
6. The page reloads after 15 seconds. Verify the new firmware is running.

> **Warning:** Do not power off the device during an update. A power loss mid-flash may require USB reflashing to recover.

### ArduinoOTA (secondary method)

ArduinoOTA is available for developer/service workflows and is **enabled by default**.

It can be disabled or re-enabled in **Diagnostics & OTA → Enable ArduinoOTA service**.

Behavior:

- Uses the configured device hostname (`hostname`) as the ArduinoOTA hostname.
- Uses the current admin password (same password used for HTTP Basic Auth) as the ArduinoOTA password.
- Is serviced in the main loop, so background output actions, web UI, and MQTT continue to run.

Typical flow:

1. Confirm ArduinoOTA is enabled on **Diagnostics & OTA** (it is enabled by default) and save if needed.
2. Ensure your development machine is on the same network.
3. Select the device's network OTA target in Arduino IDE/tooling.
4. Upload firmware over Wi-Fi; the device reboots automatically on success.

## Arduino libraries

- ESP8266 core libraries (`ESP8266WiFi`, `ESP8266WebServer`, `Updater`) from ESP8266 board package 3.x
- `LittleFS`
- `ArduinoJson` 7.x
- `PubSubClient` 2.x (for MQTT)

## Source layout

- `C4-VIBRANT.ino` — Arduino `setup()` and `loop()` entry points
- `Config` — device configuration, defaults, persistence, and MAC handling
- `Debug` — status, warning, error, and diagnostic logging
- `Outputs` and `Actions` — GPIO management and load-action sequences
- `WifiManager` and `MqttClient` — network connections and MQTT messaging
- `Reservations` and `Stickserver` — output reservations and the Stickserver protocol
- `WebServer` and `WebAssets` — HTTP routes, handlers, and embedded page assets
- `WebUpdate` and `BackupRestore` — browser firmware updates and configuration maintenance
- `OtaService` — ArduinoOTA setup and servicing
- `BootDiagnostics` — boot-time FLASH-button reset detection
- `Runtime` — applies updated settings across the runtime services
- `Version.h` — firmware version

## Build notes

To prepare `arduino-cli` (one-time setup), run:

```sh
arduino-cli config init --overwrite
arduino-cli config set board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli lib install "ArduinoJson" "PubSubClient"
```

Compile from the correctly-cased `C4-VIBRANT/` sketch directory (the folder and
`C4-VIBRANT.ino` filename must match):

```sh
cd C4-VIBRANT
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries .
```

Alternatively, open `C4-VIBRANT/C4-VIBRANT.ino` in Arduino IDE, select a NodeMCU v3
compatible ESP8266 board, ensure `ArduinoJson` and `PubSubClient` are installed,
and flash the firmware.

After boot, join the configured AP and open the device IP in a browser.

### Fleet OTA updates

To build and OTA-upload the firmware to the whole device fleet (or a subset)
in one step, use `scripts/ota_update_fleet.sh` from the repo root:

```sh
# Update the default fleet (192.168.1.194, 192.168.1.81-95)
scripts/ota_update_fleet.sh

# Update only specific devices
scripts/ota_update_fleet.sh 192.168.1.81 192.168.1.82

# Override the ArduinoOTA password (default: factory password Fiber714Cvet)
scripts/ota_update_fleet.sh -p MyPassword
```

The script cleans stale build artifacts, compiles the sketch, then uploads to
each IP over ArduinoOTA with automatic retries, printing a per-device summary.

## Current networking behavior

- The configured `SSID`/`Password` are used for both SoftAP and Station connect attempts.
- When **Use custom MAC address** is enabled, the edited MAC address is applied to both SoftAP and Station interfaces.
- Wi-Fi power is clamped to a minimum of `5.0 dBm`.
- If station connectivity drops, the firmware periodically attempts reconnect.
- If storage or runtime recovery fails irrecoverably, the device logs the reason and restarts.
- MQTT reconnects automatically (every 10 s) when the broker is unreachable.
