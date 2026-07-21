# gpio-server

Python MQTT GPIO server that mirrors the **StickServer** command/response/topic
protocol but controls 8 logical GPIO outputs instead of USB Z-Wave sticks.

---

## Quick start

```bash
pip install paho-mqtt   # only external dependency (RPi.GPIO pre-installed on Pi)
python3 gpio_server.py --ip 192.168.1.6
```

Optional flags:

| Flag | Default | Description |
|------|---------|-------------|
| `--ip` | `127.0.0.1` | MQTT broker IP |
| `--port` | `1883` | MQTT broker port |
| `--id` | *(empty)* | Optional ID suffix appended to the instance topic |

---

## MQTT topics

| Role | Topic |
|------|-------|
| Root (broadcast) | `s1/c4/stickserver/v1` |
| Instance | `s1/c4/stickserver/v1/ssvr-<mac>` |
| Instance with `--id foo` | `s1/c4/stickserver/v1/ssvr-<mac>-foo` |

The server subscribes to **both** topics on startup (identical to existing
StickServer behaviour).  All responses are published to the **instance topic**.

---

## GPIO mapping

| Logical output | BCM GPIO pin | Physical label |
|:--------------:|:------------:|----------------|
| 1 | 17 | GPIO17 |
| 2 | 18 | GPIO18 |
| 3 | 27 | GPIO27 |
| 4 | 22 | GPIO22 |
| 5 | 23 | GPIO23 |
| 6 | 24 | GPIO24 |
| 7 | 25 | GPIO25 |
| 8 | 4  | GPIO4  |

### Active-low behaviour

All outputs use **active-low** logic (matching the VIBRANT.ino relay-board
convention):

| Logical state | GPIO level | Relay |
|---------------|-----------|-------|
| OFF / idle (default on boot) | **HIGH** (1) | Open  |
| ON  / reserved-active        | **LOW**  (0) | Closed |

Outputs are driven HIGH (safe / off) during initialisation — no relay closure
on power-up.

---

## Device identifiers

Each logical output is assigned a stable `euid`:

```
gpio-<12-hex-mac>-<index>
```

Example for MAC `aabbccddeeff`, output 3: `gpio-aabbccddeeff-3`

---

## Command reference

All commands must include `"ver": 1` and a `"mid"` (message-ID) field.
Responses mirror the same `"ver"` and `"mid"` values.

### `hello`

Announce server identity.

```json
// request
{"cmd": "hello", "ver": 1, "mid": 1}

// response
{"rsp": "hello", "ver": 1, "mid": 1,
 "id": "", "mac": "aabbccddeeff",
 "topic": "s1/c4/stickserver/v1/ssvr-aabbccddeeff",
 "count": 8, "available": 8}
```

### `list`

Return all 8 devices with reservation state.

```json
// request
{"cmd": "list", "ver": 1, "mid": 2}

// response
{"rsp": "list", "ver": 1, "mid": 2, "status": 0,
 "value": [
   {"euid": "gpio-aabbccddeeff-1", "reserved": false, "owner": "", "gpio_pin": 17},
   ...
 ]}
```

### `reserve`

Reserve `count` devices for `owner`.  If the owner already holds some devices
those are counted first; additional devices are allocated as needed.

`ntype` is accepted for protocol compatibility but ignored (GPIO outputs are
type-agnostic).

```json
// request
{"cmd": "reserve", "ver": 1, "mid": 3,
 "owner": "test-agent", "count": 2, "ntype": "gpio"}

// response
{"rsp": "reserve", "ver": 1, "mid": 3,
 "owner": "test-agent", "status": 0,
 "new": "all", "euids": ["gpio-...-1", "gpio-...-2"]}
```

`"new"` is `"all"` when all returned devices were freshly allocated, `"some"`
when a mix of pre-held and new, `"none"` when the owner already held enough.

Reserving a device drives its GPIO **LOW** (relay closed).

### `release`

Release devices.  Provide a non-empty `owner` to release all devices held by
that owner, **or** provide a non-empty `euids` list.

```json
// request (by owner)
{"cmd": "release", "ver": 1, "mid": 4,
 "owner": "test-agent", "euids": []}

// request (by euid)
{"cmd": "release", "ver": 1, "mid": 5,
 "owner": "", "euids": ["gpio-...-1"]}

// response
{"rsp": "release", "ver": 1, "mid": 4,
 "status": 0, "euid": ["gpio-...-1", "gpio-...-2"]}
```

Released devices are driven **HIGH** (relay open).

### `status`

Query current state for a list of euids.

```json
// request
{"cmd": "status", "ver": 1, "mid": 6,
 "euids": ["gpio-...-1"]}

// response
{"rsp": "status", "ver": 1, "mid": 6, "status": 0,
 "value": [{"euid": "gpio-...-1", "reserved": true,
             "owner": "test-agent", "gpio_pin": 17}]}
```

### `join`

Directly activate (drive LOW) a single GPIO output without changing ownership.
Maps to the network-join concept: the output is physically activated.

```json
// request
{"cmd": "join", "ver": 1, "mid": 7, "euid": "gpio-...-3"}

// response
{"rsp": "join", "ver": 1, "mid": 7, "status": 0, "euid": "gpio-...-3"}
```

### `leave`

Directly deactivate (drive HIGH) one or more GPIO outputs without changing
ownership.  Maps to leaving a network: the output is physically deactivated.

```json
// request
{"cmd": "leave", "ver": 1, "mid": 8, "euids": ["gpio-...-3"]}

// response (one per euid)
{"rsp": "leave", "ver": 1, "mid": 8, "status": 0, "euid": "gpio-...-3"}
```

### `reboot`

Pulse a single GPIO output: drives LOW for 500 ms then back to HIGH.
Models a momentary relay closure (e.g. hardware reset button).

```json
// request
{"cmd": "reboot", "ver": 1, "mid": 9, "euid": "gpio-...-5"}

// response
{"rsp": "reboot", "ver": 1, "mid": 9, "status": 0, "euid": "gpio-...-5"}
```

---

## Running without a Raspberry Pi

When `RPi.GPIO` is not installed the server falls back to a **MockGPIO** that
logs all pin operations at DEBUG level.  This allows full protocol testing on
any machine with an MQTT broker.

---

## Verification steps

1. Start a local MQTT broker (e.g. `mosquitto -v`).
2. Run `python3 gpio_server.py --ip 127.0.0.1`.
3. Subscribe to the root topic: `mosquitto_sub -t 's1/c4/stickserver/v1/#' -v`
4. Publish commands, e.g.:
   ```
   mosquitto_pub -t s1/c4/stickserver/v1 \
     -m '{"cmd":"hello","ver":1,"mid":1}'
   ```
5. Run the unit tests (no broker or GPIO required):
   ```
   python3 test_gpio_server.py
   ```
