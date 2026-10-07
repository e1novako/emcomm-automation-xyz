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

### 1.8.19

- Fixed `Turn ON all`/`Turn OFF all`/`Leave Mesh All`/`Factory Reset All` (and the "All Outputs" page's bulk/per-output actions) appearing to silently do nothing when the server rejected the request (e.g. HTTP 409 "another action is already running", or 400/503). The page's JS submitted these via `fetch()` and only handled network-level failures (`.catch()`); a resolved response with a non-2xx status was never inspected, so a rejected action gave zero feedback. The JS now checks `response.ok` and shows an `alert()` with the HTTP status and server message on failure. Also added a `button:disabled` style (dimmed, `cursor:not-allowed`) to both the main and settings-page stylesheets, since disabled buttons previously looked identical to enabled ones and gave no visual indication of why a click had no effect.

### 1.8.18

- Fixed settings-page checkboxes (Network/MQTT, Diagnostics, etc.) rendering above their label text instead of to its left. Cause: the settings-page stylesheet applied `input,select{width:100%}` indiscriminately, which stretched every `<input type="checkbox">` to the full field width, pushing its label text onto the next line. Checkboxes are now excluded from that rule and sized/aligned explicitly (18x18px, inline, `vertical-align:middle`, small right margin) so the box sits inline to the left of its label text as intended.

### 1.8.17

- 1.8.16's zero-allocation fast path reduced but did not eliminate the heap-fragmentation reboot. The HTTP-exposed heap trace (`/settings/diagnostics/heaplog`) showed `freeAfterParse` still dropping by ~350-450 bytes on *every* message, with no recovery between messages -- even on the "no alloc" fast path. Root cause was in the MQTT message dispatch wrapper itself, not the parser: `mqttCallback()` built `String(STICKSERVER_ROOT_TOPIC) + '/'` (two temporary heap-allocated Strings) on **every single message** observed on the bus, unconditionally, to check the topic prefix; `handleStickserverMessage()`'s `addressedToUs` check called `stickserverInstanceTopic()`, which rebuilt the full instance topic (MAC token, hostname token, multiple concatenations) from scratch on every call, also on every message; and the debug-only payload log line called `payloadStr.substring(...)`, allocating again whenever `debugSerial` was enabled (as it was throughout testing). None of these leaked memory, but the repeated malloc/free churn across 14-16 back-to-back burst messages fragmented the heap into small blocks that never fully recovered. Fixed by: replacing the topic-prefix check with a raw `strncmp` (no temporary Strings); caching `stickserverInstanceTopic()` in a static String, recomputed only if `cfg.hostname`/`cfg.mac` actually change instead of on every call; and replacing the debug payload-log `substring()` with a fixed stack buffer + `memcpy`.

### 1.8.16

- Root-caused the remaining heap-fragmentation reboot using the 1.8.15 heap trace plus live captured MQTT traffic: every `kHelloIntervalMs`, all ~14-16 stickservers on the bus reply to a broadcast hello within the same second or two, each a ~1.7KB JSON payload -- a tight burst of back-to-back ArduinoJson allocations with no yield in between, even with the 1.8.12 field filter. Replaced that specific hot path with a hand-written, zero-heap-allocation scanner that parses only this firmware's own fixed hello/list response shape (status/id/ip/devices[].euid/name/state) directly out of the raw MQTT payload -- no String, no JsonDocument. Anything that doesn't match that exact shape (commands, action responses like reserve/release/status/join, or foreign/malformed traffic) still goes through the full ArduinoJson parser unchanged. Validated offline against real captured fleet traffic (16 real hello responses, AddressSanitizer/UBSan clean) before deploying. Also dropped the unused `instanceId` field from `DiscoveredServer` (parsed but never read).

### 1.8.15

- The 1.8.14 heap trace was serial-only, but this device is normally accessed remotely with no serial monitor attached. Added a fixed-size (no heap allocation, ~1KB static RAM), 16-entry ring buffer capturing the same per-message heap snapshots, exposed as JSON over `GET /settings/diagnostics/heaplog` -- so the trace can be read over HTTP/curl instead.

### 1.8.14

- Diagnostics instrumentation to pin down the remaining heap-fragmentation reboot: `/settings/diagnostics` and `/settings/diagnostics/data` now also report `maxFreeBlockSize` (largest contiguous free heap block, a better fragmentation signal than the percentage alone); `handleStickserverMessage()` now logs free heap and largest free block before/after each JSON parse and again after passive discovery-state updates, gated behind the existing `debugSerial` GUI toggle (Diagnostics & OTA page) so it can be enabled/disabled without a reflash. Goal: correlate specific incoming MQTT messages (hello/list vs. other commands) with heap drops/fragmentation jumps observed live, to find whatever is still fragmenting the heap after the 1.8.12 request-parse filter.

### 1.8.13

- Added a `/settings/diagnostics/data` REST endpoint (JSON: version, hostname, uptime, free heap, heap fragmentation, OTA/debug flags). The Diagnostics & OTA page now polls this every 5 seconds and refreshes only the uptime/heap text in place, instead of relying solely on a client-side uptime ticker -- heap and fragmentation numbers now stay live without a manual page reload.

### 1.8.12

- 1.8.11's 7168-byte static arena still crash-looped 192.168.1.194 within under a minute (free heap fluctuating 900-4700 bytes, truncated page responses) -- confirming (a second time, at a much smaller size than the original 12.8KB attempt) that this chip has essentially no safe margin for additional static RAM beyond its existing ~64% footprint. Removed the static arena and heap-fallback mechanism entirely. Kept the two changes that cost no additional static RAM: (1) parsing the incoming `request` document (still using the default heap allocator, freed every call as before 1.8.7) with a `DeserializationOption::Filter` restricted to only the fields this firmware actually reads, shrinking each parse's transient footprint by roughly a third; (2) no longer publishing an "invalid_json" error response for parse failures on messages that were never addressed to this device (root broadcast or its own instance topic) -- previously any device reacted to any malformed traffic anywhere on the bus, which is very likely the source of the stray `"error":"invalid_json"` messages observed fleet-wide from devices that were never the intended recipient. Static RAM is back to 64% (the 1.8.6 baseline). This is a smaller, safer mitigation than full pre-allocation; it reduces the size of the dominant variable-size allocation but does not eliminate heap churn the way a dedicated arena would have -- needs an extended live stability test to see whether it meaningfully improves on the original ~60-90s background-reboot cadence.

### 1.8.11

- 1.8.10's 9216-byte request arena (sized for the MAX_DEVICES=16 theoretical ceiling) left only ~2KB of boot-time free heap on 192.168.1.194 and crashed/truncated pages within under a minute -- too large for this chip's real headroom. Shrunk to 7168 bytes, sized instead for this device's realistic output count (DEFAULT_NUM_OUTPUTS=8; an ESP8266 only has ~9 usable GPIOs, so the 16-device ceiling is not physically reachable on real hardware), measured via an 20000-cycle randomized AddressSanitizer stress test (worst case 6632 bytes + margin). Added a one-time fallback to the default heap allocator if a message still doesn't fit the arena (rare, e.g. an unusually large peer), instead of either silently dropping it or risking a static arena large enough to starve the device. Also fixed a related bug while in this code path: a JSON parse failure (now more reachable with a smaller arena, but pre-existing for any genuinely malformed/truncated peer payload) no longer causes this device to publish an "invalid_json" error response for messages that were never addressed to it (root broadcast or its own instance topic) -- previously any device would react to any malformed traffic anywhere on the bus, which is very likely the source of the stray `"error":"invalid_json"` messages observed fleet-wide from devices that were never the intended recipient.

### 1.8.10

- Real fix attempt for the background MQTT heap-fragmentation reboot (1.8.9 was a clean revert back to the original unfixed behavior). Root cause: `request` -- the JsonDocument that parses every incoming message on the stickserver MQTT bus, including every peer's hello/list broadcast with 1-16 devices and arbitrary string lengths -- is parsed with a wildly variable size/shape every single time passive discovery or active querying is enabled. That variability (not frequency alone) is what fragments the ESP8266 heap allocator over time. `request` now uses a dedicated 9216-byte static arena (sized with a 5000-cycle AddressSanitizer stress test against randomized worst-case 16-device payloads, which measured 8232 bytes needed) instead of the default heap allocator, and is parsed with `DeserializationOption::Filter` restricted to only the fields this firmware actually reads (dropping a peer's manufacturer/model/pin/reserved metadata we never use), cutting the worst-case need by roughly a third versus the unfiltered document. `response` (built by this device to answer hello/list/reserve/etc.) deliberately keeps the default heap allocator: it is built with this device's own fixed, constant shape and size every time, which is a uniform allocation pattern that does not fragment the allocator the way `request`'s variable one does -- giving it its own arena as well was tried in 1.8.8 and required too much static RAM for this chip's budget. Static RAM is now 76% (up from 64% pre-1.8.7); needs an extended live stability test to confirm the background reboot is actually resolved.

### 1.8.9

- Reverted the 1.8.7/1.8.8 experimental static-allocator attempts at fixing the background MQTT heap-fragmentation reboot: both were confirmed unsafe. 1.8.7's `static JsonDocument` + `.clear()` reuse didn't help at all (`deserializeJson()`/`clear()` unconditionally free and reallocate a document's pools regardless, so it churned the heap exactly like a fresh document every time). 1.8.8's custom arena allocator sharing one static buffer between the incoming-request and outgoing-response `JsonDocument`s caused genuine memory corruption, confirmed with AddressSanitizer: each `JsonDocument` keeps its own internal string-pool list pointers as private members outside the arena, so once one document's `.clear()` ran after a different document had overwritten the same arena memory, it walked a stale pointer into whatever data was now there and crashed -- this is a structural ArduinoJson v7 limitation, not a sizing bug, and reproduced at every arena size tried, including generously large ones. Giving each document its own non-shared, correctly-sized arena instead would need roughly 12-14KB of static RAM for worst-case message sizes, which on this chip's ~80KB total RAM (already ~64% statically committed before any of this) leaves so little heap for WiFi/MQTT/lwIP's own needs that the device destabilizes just as fast, only via plain exhaustion instead of corruption. This version is back to the known, pre-1.8.7 behavior (plain per-call `JsonDocument`s, no custom allocator): the original background heap-fragmentation reboot (roughly every 60-90s with passive discovery enabled) is unfixed and still present; it needs a different strategy (e.g. filtering incoming JSON parses down to only the fields actually needed instead of holding the full document, or tolerating periodic self-recovering reboots) rather than pre-allocation as attempted here.

### 1.8.8

- Replaced the heap-allocated `JsonDocument`s on the hot MQTT background path (the incoming-message parser in `handleStickserverMessage()`, its response builders, and the periodic hello/list discovery-broadcast requests in `maintainStickserverDiscovery()`) with static arena allocators: fixed buffers reserved once at startup (two 6KB buffers plus one 512-byte buffer, as static RAM) that are reused via a bump pointer reset before each parse/build. The 1.8.7 attempt (a `static JsonDocument` cleared and reused) turned out not to help: `deserializeJson()` calls `JsonDocument::clear()` internally, which unconditionally frees every memory pool the document holds and reallocates fresh ones on next use, so a reused-but-default-allocated document still churned the general heap exactly like a fresh one per message. With passive discovery enabled, every fleet peer's hello/list broadcast hit this path every few seconds, fragmenting the heap down to under 3KB free (60%+ fragmentation) within about a minute and crashing the device -- confirmed still happening on 1.8.7 even with zero HTTP traffic. The new arena allocators never call the general heap at all on this path, eliminating that churn entirely.

### 1.8.7

- Attempted to fix repeated device reboots that persisted even with no web traffic at all by reusing a `static JsonDocument` (cleared, not reconstructed) in `handleStickserverMessage()`. Superseded by 1.8.8: this did not actually fix the heap fragmentation (see 1.8.8 notes for why) and the crash was still reproduced on this version.

### 1.8.7

- Fixed the actual root cause of repeated device reboots that persisted even with no web traffic at all: `handleStickserverMessage()` (the handler invoked for every single incoming MQTT message on the shared stickserver topic/wildcard subscription, including every peer's hello/list broadcast) was constructing a brand-new `JsonDocument` per call to parse the request and, in several command branches, a second new `JsonDocument` to build the response. ArduinoJson v7's `JsonDocument` always backs itself with heap allocations that grow via realloc, so with passive discovery enabled and many fleet devices broadcasting every few seconds, this repeated construct/parse/destroy cycle fragmented the heap over roughly a minute of uptime (free heap falling under 3KB, fragmentation over 60%) until the device crashed. Both documents are now `static`, reused across calls and `.clear()`ed before each use, so their internal pools grow once to a stable size instead of being torn down and rebuilt on every message.

### 1.8.6

- Replaced the "All Outputs" page's server-rendered HTML table with a REST API (`GET /fleet/data`, JSON) plus client-side rendering. The device previously had to rebuild and stream the entire output table (with a `<form>` per button) on every page load and every periodic refresh, which was the underlying cause of several reboots under heap/WiFi pressure; the `/fleet` page is now a small fixed-size shell that fetches `/fleet/data` on load and every 5s and builds/rebuilds the table entirely in the browser, so the device does constant, minimal work per refresh regardless of fleet size and the page renders correctly and consistently every time. The per-output toggle buttons and the four bulk-action buttons are now plain buttons wired to small JS `fetch()` calls instead of individual `<form>` elements, simplifying the markup. `POST /fleet/toggle` and `POST /fleet/bulk` now return a plain ack instead of a redirect, since they're called via `fetch()` rather than native form submission.

### 1.8.5

- Fixed another cause of the device rebooting while a browser had the main or All Outputs page open: the page's auto-refresh timer (`/partial` every 3s, `/fleet/partial` every 5s) fired unconditionally, so on a slow/weak Wi-Fi link a second connection to fetch the next refresh could start while the previous one was still being streamed. Having two large responses in flight at once has been observed to crash the device. Both refresh timers now skip starting a new fetch while the previous one is still outstanding.

### 1.8.4

- Fixed MQTT `hello`/`list` stickserver responses occasionally being published truncated (seen fleet-wide as `invalid_json`/`IncompleteInput` errors on receiving devices). The response JSON was previously serialized into an intermediate Arduino `String`, which can silently truncate on heap exhaustion; it is now streamed directly into PubSubClient's publish buffer via `beginPublish()`/`write()`/`endPublish()`, which also fails cleanly (publishing nothing) if the message does not fit the configured MQTT buffer, instead of risking a partial publish.

### 1.8.3

- Fixed the device rebooting while rendering the "All Outputs" page (`/fleet`, `/fleet/partial`) when many stickservers/outputs were discovered: the per-cell table render loop never yielded between the (potentially hundreds of) blocking network writes, which could starve the WiFi/TCP stack long enough to trip the watchdog. A `yield()` is now called after each cell.

### 1.8.2

- Reduced heap fragmentation and fixed intermittent truncated/incomplete page loads on `/settings`, `/settings/network`, and `/settings/diagnostics`: these pages previously built their entire HTML as one large `String` in RAM before sending it, which could silently truncate under heap pressure (Arduino `String` concatenation fails silently on out-of-memory). They now stream in small independent pieces, the same pattern already used by the home, fleet, and device-settings pages. This also reduces the chance of incoming MQTT stickserver requests failing to parse with `"error":"invalid_json","message":"NoMemory"` while a settings page is being served.

### 1.8.1

- Fixed "Turn ON all"/"Turn OFF all" (and other bulk/per-output actions) appearing to silently do nothing on some browsers: the `/partial` and `/fleet/partial` auto-refresh endpoints (and all other chunked HTML pages) now send `Cache-Control: no-store`, so a browser can no longer serve a stale cached copy of the output-status page after an action changed server-side state.

### 1.8.0

Consolidated release note covering all work since the original 1.8.0 (formerly versioned 1.8.0-1.13.0; those intermediate version numbers have been folded into this single entry).

- Split the single "stickserver fleet discovery" toggle into three independent flags on the Network settings page: `stickserverRespondEnabled` (respond to peers' `hello`/`list` requests, **enabled by default**), `stickserverQueryEnabled` (actively broadcast this device's own `hello`/`list` requests to discover other stickservers, **disabled by default**), and `stickserverPassiveDiscoveryEnabled` (passively parse `hello`/`list` responses observed on the bus to populate the "All Outputs" page, **disabled by default**). Configs saved under the old single `stickserverDiscoveryEnabled` flag are migrated automatically.
- Discovered stickserver records (and their per-output state) are timestamped when a `hello`/`list` response is parsed, and forgotten after 60 seconds without a refresh. The "All Outputs" page updates in real time as `power_on`/`power_off`/`reserve`/`release`/`status`/`join`/`reboot`/`factory_reset` responses are observed on the bus, not just on the next periodic `list` poll.
- The "All Outputs" page and the main output-control page both auto-refresh via lightweight partial-reload endpoints (`GET /fleet/partial`, `GET /partial`) that swap in just the status/output-table section via `fetch()`, instead of a full page reload. Clicking any button (toggle, reservation release, per-output Leave Mesh/Factory Reset, bulk actions) submits in place and schedules a refresh 1 second later.
- Removed the `|` separators between navigation menu buttons on every page.
- All pages share one unified header: device name "C4-VIBRANT-<MAC> Output Control" centered above the page content, with "FW: <firmware version>, Uptime: hh:mm:ss" centered underneath (uptime ticks live in the browser).
- All page content (header, navigation, tables, and form fields) is encapsulated in a single 1200px-max-width column, centered on the screen, consistently on every page including the "All Outputs" page.
- The navigation menu is identical and horizontally centered on every page: "Main output control", "Network, Wi-Fi & MQTT", "Devices & outputs", "Diagnostics & OTA", and "All Outputs", with the active page's button highlighted.
- Renamed the output table columns: "Manufacturer" -> "DUT Manufacturer", "Model" -> "DUT Model", "Name" -> "DUT Name", and (main page) "Output" -> "Status". The Reservation button shows "n/a" instead of "Not reserved" when unreserved and is always clickable: clicking it while "n/a" reserves that output for the GUI (owner "GUI"), blocking MQTT reservation of it. The Actions column is widened so its buttons no longer wrap onto two lines.
- Added bulk-action buttons ("Turn On", "Turn Off", "Leave Mesh", "Factory Reset") to the "All Outputs" page, sending one MQTT command per discovered output across every discovered stickserver via `POST /fleet/bulk`.
- On the "All Outputs" page, stickserver column headers strip the common `C4-VIBRANT-` hostname prefix, showing just the distinguishing MAC suffix.
- Output ON/OFF state and reservations can be persisted to flash and restored after boot/reboot, gated by two new, independent flags on the Network settings page (both **disabled by default**): **Restore output ON/OFF state after boot/reboot** and **Restore output reservations after boot/reboot**. Transient pulse/sequence steps (Leave Mesh, Factory Reset profiles) never trigger a flash save; only discrete, user-intentional state/reservation changes do.
- Configuration backups (`/config/export`) always include each output's current reservation (`reserved`/`reservedOwner`), captured live at export time regardless of the `restoreReservationsOnBoot` flag. Restoring a backup (`/config/import`) always restores each output's reservation from the file, independent of `restoreReservationsOnBoot` (a deliberate admin-initiated restore is distinct from normal boot behavior). After every import, the firmware verifies the restored data matches the uploaded backup file and logs a warning for any mismatch.

### 1.7.0

- Added a "stickserver fleet discovery" toggle on the Network settings page (`stickserverDiscoveryEnabled`), **disabled by default**. When disabled, a device neither broadcasts `hello`/`list` discovery requests nor responds to peers' `hello`/`list` requests, and does not build its "All Outputs" peer table from observed responses -- it still handles directly-addressed action commands (power_on/power_off/toggle/reserve/release/etc.) normally. This is an explicit opt-in for the MQTT discovery/All-Outputs feature, since most deployments only need direct device control and discovery was a source of continuous background MQTT broadcast traffic.

### 1.6.2

- Fixed MQTT "invalid_json"/"IncompleteInput" parse errors observed live on the bus: `PubSubClient::setBufferSize()` was called on every MQTT (re)connect attempt without checking its return value; under heap pressure the 2048-byte realloc can silently fail, leaving the client at its previous (possibly library-default 256-byte) buffer, which truncates large hello/list JSON responses in transit. The buffer size is now only (re)applied once, its result is checked, and a warning is logged if it doesn't take effect.
- Fixed each stickserver needlessly sending itself a periodic "list" request every ~10 seconds: a device's own "hello" response is echoed back to itself via the shared MQTT subscription and already includes a full `devices` array, so self-polling via "list" was pure redundant bus traffic. The periodic list-request loop now skips the entry matching the device's own instance topic.

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
