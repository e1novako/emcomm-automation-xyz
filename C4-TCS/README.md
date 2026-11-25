# C4-TCS (TCS_Intercom)

ESP8266 sniffer/transmitter for the TCS two-wire intercom bus, bridged to MQTT and Control4.

- Input GPIO14 (decoded in an ISR), output GPIO12 (bit-banged).
- Publishes decoded frames on `mqtt_topic`; accepts commands on `tcs_command_topic`; opens the door on `control4_command_topic`.
- Config is stored in NVRAM (EEPROM emulation, its own flash sector with CRC; survives firmware/FS updates). A copy is kept in LittleFS `/config.json` (also the legacy location); NVRAM is rebuilt from it if empty or corrupt. Keys: `mqtt_server`, `mqtt_port`, `mqtt_topic`, `mqtt_client`, `mqtt_user`, `mqtt_pass`, `control4_command_topic`, `control4_notify_topic`, `c4_node_unid`, `tcs_command_topic`, `tz`, `ntp_server`, `debug`, `serial_number` (hex), `wifi_ssid`, `password`, `allow_programming`. Users stay in LittleFS `/tcs_users.json`.

## Build

Sketch folder name must match the `.ino` name:

    arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --export-binaries TCS_Intercom/

Libraries: ArduinoJson 7.x, PubSubClient, WiFiManager 2.0.17.

OTA build + upload in one step: `./ota_upload.sh`. It reads `OTA_IP`, `OTA_PORT` and `OTA_PASS` from `ota.local.env` (git-ignored, create it locally). Under WSL it uploads through Windows Python, since the device has to connect back to the uploading host.

## Password, WiFi and OTA

The WiFi password is the one password: it protects the web GUI (HTTP basic auth, any user name, only the password is checked), OTA, and is used to join WiFi. By default (`password` empty) the device uses the WiFi password already stored in it. Set `wifi_ssid` plus a password (8-63 chars) in Settings to join another network, or tick "Use the WiFi password already stored" to clear the override. If joining the configured SSID fails (or it is blank) the device falls back to WiFiManager and its config AP `TCS-<chipid>` (password = the same, or `intercom` if none is known). Web form posts must come from the device's own pages (Origin/Referer is checked against Host) so another website cannot submit them with the browser's cached password. Holding the FLASH button (GPIO0) at boot clears `wifi_ssid`, `password` and the stored WiFi credentials and opens the config AP. The config AP closes after 3 minutes and the device reboots to retry the stored network (e.g. when the router comes up after a power cut). OTA upload: `espota.py -i <ip> -p 8266 -a <wifi password> -f <bin>`. Basic auth is cleartext over HTTP; an open (password-less) network leaves GUI and OTA unprotected.

## Backup / restore

Web GUI pages, reached from a rounded-button menu at the top: Home (`/`: no menu, only a small Config button to Access; big "Unlock door" button and door-unlocked counter; viewable without a password, pressing the button still asks for it), Access (`/status`: last rings, headed by the ring/unlock totals), WiFi (`/wifi`), MQTT (`/mqtt`), Time (`/time`), Statistics (`/rings`, auto-refreshes every 10 s; door-ring count, unlocks after a ring, last and average ring-to-unlock time and last ring per user; counters live in `tcs_users.json` as `rings`/`unlocks`/`last_ring` so they survive reboot/power loss. An unlock (bus, MQTT or web OPEN_DOOR) within 30 s of a door ring is credited once to the user that was rung), Users (`/users`), Backup/Restore (`/backup`: config backup/restore, users export/import as JSON via `/users/download` and `/users/restore`, last-rings export `/rings/download`, "Reset Unlocks" / "Reset Rings" buttons that zero that counter for every flat (`POST /stats/reset`, `what=unlocks|rings`), "Reset total counters" that zeroes every flat's unlock counter and the Home door-unlocked total (`what=all_unlocks`), and a "Reboot only" button). Debug (`/debug`) shows uptime/reset reason, WiFi, MQTT, door address, dropped frames, the debug-logging toggle and memory (heap, fragmentation, sketch/flash, LittleFS, users buffer). Access shows the last 20 rings (kept across reboots in LittleFS `/rings.json`) (time, door/internal, serial, user, time from ring to unlock). Users page shows each user's ring and unlock counters with a reset tick box (applied on Save) and holds up to 40 users (serials are 1-8 hex digits, unique, stored as 6 lower-case digits; names up to 48 bytes); serials seen in a door or internal ring that are not in the list are added automatically as `n/a` (rename them on the Users page). Saving a page writes NVRAM and reboots. Backup/Restore: download the JSON (`/backup/download`, includes passwords) or upload one to `/restore`; it is applied, saved and the device reboots. All pages except Home need the password.

## Door-open address

The door station address used for the Control4 open-door toggle is tracked dynamically from door-related bus frames (door call, talking, door opened/closed, end of ringtone, door readiness, init). If the controller changes address under bus congestion, the next such frame updates it. `tcs_command` messages without an `address` also use it. `address`, `payload` and `serial_number` in command messages are hex strings (or numbers).

## Web GUI / debug

`http://<device>/debug` shows status (MQTT, tracked door address, dropped frames) and a button to turn serial debug logging on/off; the setting persists in the config (`debug`). `tz` is a POSIX TZ string (default CET/CEST).

## MQTT topics

- `<mqtt_topic>`: decoded/sent frames and events (`sniffer_online`, `control4_notify`).
- `<tcs_command_topic>`: JSON commands to send on the bus. Programming/memory/reset commands (`RESET`, `SELECT_DEVICE_GROUP[_RESET]`, `PROGRAMMING_MODE`, `READ_MEMORY_BLOCK`, `SELECT_MEMORY_PAGE`, `WRITE_MEMORY`) are refused unless "Allow programming" is ticked on the MQTT page (`allow_programming`). Addresses above `ff` are refused.
- `<control4_command_topic>`: any message (payload ignored, may be empty) opens the door.
- `<mqtt_topic>/availability`: retained `online` / `offline` (last will).
- `<mqtt_topic>/status`: every 60 s: version, uptime, RSSI, free heap, RX overruns, TX dropped/queued, door address.
- Control4 fake Z-Wave node (`c4_node_unid`, default `zw-D80822E0-00F0`, blank = off): on every MQTT connect the device publishes retained UCL topics under `ucl/by-unid/<c4_node_unid>/` (State "Online functional", ep0 Basic as a SensorNotification named "TCS Intercom", IASZone, and OTA UIID `ZWave-00c4-00c4-00c4-00-01`, i.e. manufacturer/product type/product id c4:c4:c4), modelled on a real IAS zone sensor. The Control4 Z-Wave agent lists it as an online node, so a driver (`tcs_intercom.c4z`) can be bound to it; republishing keeps it alive across broker resets. The default command/notify topics point at this node.
- Control4 IAS zone (`control4_notify_topic`, handled by the driver bound to that node: `tcs_intercom.c4z` v2 or `zwave-common-sensor`): Alarm1 on an internal call to the selected flat (`serial_number` setting), Alarm2 on a door call to it. Each ring is pulsed once (set, then clear; repeated bus frames within 5 s are ignored), so the driver fires "Alarm 1/2 set (endpoint 0)" and opens/closes its "Alarm 1/2 contact" binding; use the Alarm 2 contact or event for a doorbell notification in Composer. Zone status messages are retained and an idle status is published on every MQTT connect, so the driver creates its IASZone topic before the first ring.

## Transmit

Frames are queued (4 deep, 20 ms gap) and bit-banged by hardware timer1. RX is ignored while transmitting. Received frames go into a 16-deep queue in the ISR, so frames arriving while `loop()` writes flash or waits on an MQTT connect (socket timeouts 2 s) are kept; a full queue counts as a dropped RX frame.

## Tests

Host-side protocol round-trip tests (no hardware needed):

    g++ -std=c++17 -Itest/stubs -I. test/test_protocol.cpp tcs_protocol.cpp -o /tmp/tcs_test && /tmp/tcs_test

## Changelog

- 1.2.8: fake Control4 Z-Wave node: the UCL topics of node `c4_node_unid` (default `zw-D80822E0-00F0`, manufacturer/product type/product id c4:c4:c4) are published retained on every MQTT connect; default Control4 command/notify topics moved from node 0084 to it.
- 1.2.7: Control4 ring notification fixed: the IAS zone status is retained and published idle on MQTT connect (the sensor driver created the topic from the first ring and lost it), and a ring for the selected flat is notified once instead of once per repeated bus frame.
- 1.2.6: Access: the Serial column is removed; a ring from a user without a name (empty or `-`) shows the serial number in the name column.
- 1.2.5: time to unlock: the time from a door ring to the unlock that follows it (within 30 s) is measured. Access shows it per ring ("Time to unlock" column, `resp` in seconds in `/rings.json` and the rings export); Statistics shows the last and average time per flat (kept in `tcs_users.json` as `resp_last`, `resp_total`, `resp_n`, in 0.1 s, and reset together with the unlock counter). Statistics now auto-refreshes every 10 s like Access.
- 1.2.4: each door ring in the Access log remembers whether the door was unlocked within 30 s (`unlocked` in `/rings.json` and the rings export); door rings without an unlock are shown in pastel red. Rings logged before 1.2.4 are not coloured.
- 1.2.3: Access and Statistics headings show "Total Rings: <sum of per-flat rings>, Unlock After Ring: <sum of per-flat unlocks>, Total Unlock: <door-unlocked total>" instead of "Last rings" / "Door rings per device".
- 1.2.2: uptime line on every page counts up each second in the browser and re-syncs from the new public `GET /uptime` (seconds, plain text) every 30 s when off by more than 2 s; uptime no longer wraps after 49.7 days.
- 1.2.1: Access page keeps the last 20 rings (was 10); the page is streamed like Users/Statistics.
- 1.2.0: "Reset all unlocks (incl. total)" moved from Users to Backup/Restore and renamed "Reset total counters".
- 1.1.7: Users and Statistics pages are streamed (chunked) instead of built in one String; the Users page with ~30 users ran out of heap and crashed the device.
- 1.1.6: Users page counter columns centered; "Reset all unlocks (incl. total)" button.
- 1.1.5: per-flat ring/unlock counter reset on the Users page; Reset Unlocks / Reset Rings buttons on Backup/Restore.
- 1.1.4: per-user "unlocked after ring" counter (`unlocks`): an unlock within 30 s of a door ring counts for the rung user; shown on Statistics.
- 1.1.3: config portal times out after 3 min (no more hanging after a power cut); FLASH-button reset also clears `wifi_ssid`/`password`; CSRF guard on all form posts; 16-frame RX queue and 2 s MQTT socket timeouts; Control4 toggle works with an empty/non-JSON payload; MQTT programming commands need `allow_programming`; users: hex-only unique serials, names cut to 48 bytes without splitting UTF-8 characters, compact users export; ArduinoJson 7 `JsonDocument` for users (Debug shows user count / file size).
- 1.1.2: Statistics page sorted by Ring No. descending (ties by flat number).
- 1.1.1: Rings page renamed Statistics (URL unchanged).
- 1.0.28: 200 px empty space above the unlock button.
- 1.0.27: unlock button at least 300 px wide.
- 1.0.26: Home has no menu, just a small Config button (to Access); unlock button at least 200 px wide (300 px since 1.0.27).
- 1.0.25: Home page needs no password (unlock button and all other pages still do); device-name line removed; narrower button.
- 1.0.24: Status renamed Access (`/status`); new Home page at `/` with device name, big Unlock door button (`POST /unlock`, sends OPEN_DOOR to the tracked door address) and the Door unlocked counter (moved from Access).
- 1.0.23: Rings page shows the serial when a user has no name; door-unlock counter (bus or MQTT/web OPEN_DOOR, once per 2 s) saved in LittleFS `/stats.json` and shown at the bottom of Status.
- 1.0.22: menu order Status, Rings, Users, WiFi, MQTT, Time, Debug, Backup/Restore; Rings page sorted by flat number (no flat last).
- 1.0.21: `Door`/`Internal` capitalised; Users page numbered; per-user door ring counter + last ring (persisted) and new Rings page.
- 1.0.20: rings table shows `<flat>. <name>` (name column 3x wider); Users page columns Serial No., Flat No., Surname / Name.
- 1.0.19: last-rings table with borders and equal-width columns, each centered.
- 1.0.18: the Status (main) page auto-refreshes every 10 s.
- 1.0.17: uptime shown under the page title on every page.
- 1.0.16: page content centered; rings table full width with a Date/Time column.
- 1.0.15: menu and rings table centered; users export/import and rings export on Backup/Restore; last rings persisted across reboots.
- 1.0.14: page title centered and larger.
- 1.0.13: page title with version moved above the menu; pages are 1000 px wide and centered.
- 1.0.12: new Debug page (status details, debug toggle, memory usage moved/added); Status page now shows only the ring list.
- 1.0.11: users limit 40; unknown serials auto-added on ring; last-10-rings table on Status; Reboot-only button.
- 1.0.10: TX carrier sense (waits for a quiet bus, max 300 ms) and TX watchdog; config saved atomically to flash `/config.json` plus NVRAM, and NVRAM is rebuilt from it whenever it is empty/corrupt; Users page (`/users`) to add/edit/delete `tcs_users.json` entries (saved atomically); MQTT bus messages carry `name` when the serial number is a known user; Home Assistant discovery file removed.
- 1.0.9: web GUI split into WiFi, MQTT, Time and Backup/Restore pages with a rounded-button top menu; `ntp_server` setting; `/config` removed, download moved to `/backup/download`.
- 1.0.8: the password is the WiFi password (default: the stored one), no user name; the `intercom` default is only the fallback AP password.
- 1.0.7: single password for GUI/OTA/WiFi; configurable SSID; all settings in NVRAM; settings backup/restore; GUI behind basic auth; `serial_number` setting; AP name `TCS-<chipid>`.
- 1.0.6: restored `stop_talking(address)` helper.
- 1.0.5: timer-driven non-blocking TX with queue; bounded JSON building and 512-byte MQTT buffer; availability (last will) and periodic status topics; stable MQTT client ID; `DEVICE_TEST` plus buildable `FOUND_DEVICE`/`FOUND_DEVICE_SUBSYSTEM`; `FLOOR_CALL` carries an address; Control4 notify carries alarm/address/serial and Alarm2 on door calls; `/config` web page for all settings (saves and reboots; no auth); dead code removed; host protocol tests.

- 1.0.4: dynamic door address; non-blocking WiFi/MQTT reconnect and NTP; ISR frame snapshot with overrun counter; frame length taken from the bus (long/short) and used when sending; `OPEN_DOOR_LONG` parsed distinctly; hex-only payload parsing, no malloc in callback; `address`/`payload` always published; configurable TZ; debug toggle web GUI; users buffer 8 KB; header/implementation overloads fixed.

- 1.0.3: re-subscribe to MQTT topics after a WiFi reconnect; bounds-checked config/MQTT buffers (`strlcpy`, `snprintf`); NUL-terminated file reads, missing JSON keys no longer crash; control4/command topics are now saved and loaded from `config.json`.
