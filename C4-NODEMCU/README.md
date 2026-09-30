# C4-NODEMCU

ESP8266 firmware for configurable relay and servo-actuator outputs. The web
Settings page controls the persisted verbose serial-debug setting.

## Shared helpers

This project uses `../libraries/EmcommCommon` for strict MAC-address parsing
and the debug-output gate. Invalid saved MAC values are ignored in favor of
the hardware MAC; invalid web values are rejected without changing settings.
The GUI toggle still controls verbose logs, while direct status and error
messages remain available.

## Version history

- 3.2.2 — Adopt shared MAC parsing and the GUI-controlled debug gate.
- 3.2.3 — Validate submitted MAC addresses before applying any configuration.
- 3.2.1 — Security and stability fixes; see the version notes in
  `C4-NODEMCU.ino`.
