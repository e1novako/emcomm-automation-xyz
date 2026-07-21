#!/usr/bin/env python3
"""
GPIO Server — MQTT-controlled 8-output GPIO driver.

Mirrors the StickServer command/response/topic protocol but drives
8 logical GPIO outputs instead of USB Z-Wave sticks.

Root topic    : s1/c4/stickserver/v1
Instance topic: s1/c4/stickserver/v1/ssvr-<mac>[-<id>]

GPIO mapping (BCM numbering, active-low):
  Output 1 → GPIO 17      Output 5 → GPIO 23
  Output 2 → GPIO 18      Output 6 → GPIO 24
  Output 3 → GPIO 27      Output 7 → GPIO 25
  Output 4 → GPIO 22      Output 8 → GPIO 4

Active-low convention (aligned with VIBRANT.ino relay-board semantics):
  Logical ON  / reserved-active  → GPIO driven LOW  (relay closed)
  Logical OFF / idle             → GPIO driven HIGH (relay open, safe default)

Supported commands (see class docstrings for per-command JSON contract):
  hello, list, reserve, release, status, join, leave, reboot
"""

from __future__ import annotations

import argparse
import json
import logging
import queue
import threading
import time
import uuid
from typing import Any

import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ROOT_TOPIC = "s1/c4/stickserver/v1"
NUM_OUTPUTS = 8

SUCCESS = 0
FAILURE = 1

# BCM GPIO pin numbers for logical outputs 1-8 (active-low).
# Modify to match your actual board wiring.
GPIO_PIN_MAP: dict[int, int] = {
    1: 17,
    2: 18,
    3: 27,
    4: 22,
    5: 23,
    6: 24,
    7: 25,
    8: 4,
}

# ---------------------------------------------------------------------------
# GPIO abstraction — real RPi.GPIO with graceful mock fallback
# ---------------------------------------------------------------------------


class _MockGPIO:
    """Fallback used when RPi.GPIO is not installed (CI / dev machines)."""

    BCM = "BCM"
    OUT = "OUT"

    def setmode(self, mode: str) -> None:
        logging.debug("[MockGPIO] setmode(%s)", mode)

    def setup(self, pin: int, mode: str, initial: int = 1) -> None:
        logging.debug("[MockGPIO] setup(pin=%d, mode=%s, initial=%d)", pin, mode, initial)

    def output(self, pin: int, value: int) -> None:
        logging.debug("[MockGPIO] output(pin=%d, value=%d)", pin, value)

    def cleanup(self) -> None:
        logging.debug("[MockGPIO] cleanup()")


try:
    import RPi.GPIO as _RealGPIO  # type: ignore

    GPIO = _RealGPIO
    logging.debug("Using real RPi.GPIO")
except ImportError:
    GPIO = _MockGPIO()  # type: ignore[assignment]
    logging.debug("RPi.GPIO not found — using MockGPIO (no physical output)")

# ---------------------------------------------------------------------------
# Device model
# ---------------------------------------------------------------------------


class GpioDevice:
    """One logical GPIO output device.

    ``euid`` format: ``gpio-<12-hex-mac>-<1-based-index>``
    Example: ``gpio-aabbccddeeff-3``
    """

    def __init__(self, index: int, mac: str) -> None:
        self.index = index  # 1-based logical index
        self.gpio_pin: int = GPIO_PIN_MAP[index]
        self.euid: str = f"gpio-{mac}-{index}"
        self.reserved: bool = False
        self.owner: str = ""

    # ------------------------------------------------------------------
    # GPIO helpers
    # ------------------------------------------------------------------

    def activate(self) -> None:
        """Drive GPIO LOW (active-low = relay closed = logically ON)."""
        GPIO.output(self.gpio_pin, 0)
        logging.debug("GPIO pin %d → LOW (output %d activated)", self.gpio_pin, self.index)

    def deactivate(self) -> None:
        """Drive GPIO HIGH (active-low = relay open = logically OFF)."""
        GPIO.output(self.gpio_pin, 1)
        logging.debug("GPIO pin %d → HIGH (output %d deactivated)", self.gpio_pin, self.index)

    def pulse(self, duration: float = 0.5) -> None:
        """Briefly activate (LOW) then deactivate (HIGH) — used for reboot."""
        self.activate()
        time.sleep(duration)
        self.deactivate()

    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"<GpioDevice euid={self.euid!r} pin={self.gpio_pin} "
            f"reserved={self.reserved} owner={self.owner!r}>"
        )


# ---------------------------------------------------------------------------
# GPIO Server
# ---------------------------------------------------------------------------


class GpioServer:
    """MQTT GPIO server that mirrors the StickServer protocol."""

    def __init__(self) -> None:
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s %(funcName)s: %(message)s",
            datefmt="%H:%M:%S",
        )

        parser = argparse.ArgumentParser(
            description="MQTT GPIO server — StickServer-compatible protocol"
        )
        parser.add_argument(
            "--ip", default="127.0.0.1", help="MQTT broker IP address"
        )
        parser.add_argument(
            "--port", type=int, default=1883, help="MQTT broker port"
        )
        parser.add_argument(
            "--id", default="", help="Optional ID suffix for the instance topic"
        )
        args = parser.parse_args()

        self.broker_ip: str = args.ip
        self.broker_port: int = args.port
        self.stick_server_id: str = args.id
        self.mac: str = f"{uuid.getnode():012x}"
        self.running: bool = True

        # Instance topic
        if self.stick_server_id:
            self.topic = f"{ROOT_TOPIC}/ssvr-{self.mac}-{self.stick_server_id}"
        else:
            self.topic = f"{ROOT_TOPIC}/ssvr-{self.mac}"

        # Message queue (thread-safe): items are (topic, payload) tuples
        self._msg_queue: queue.Queue[tuple[str, bytes]] = queue.Queue()

        # Initialise devices
        self.devices: dict[str, GpioDevice] = {}
        self._init_gpio()

        # Command dispatch table
        self.handlers = {
            "hello": self._handle_hello,
            "list": self._handle_list,
            "reserve": self._handle_reserve,
            "release": self._handle_release,
            "status": self._handle_status,
            "join": self._handle_join,
            "leave": self._handle_leave,
            "reboot": self._handle_reboot,
        }

        # MQTT client
        self._mqtt = mqtt.Client()
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt.on_disconnect = self._on_disconnect
        self._connected = threading.Event()

    # ------------------------------------------------------------------
    # GPIO initialisation
    # ------------------------------------------------------------------

    def _init_gpio(self) -> None:
        GPIO.setmode(GPIO.BCM)
        for idx in range(1, NUM_OUTPUTS + 1):
            pin = GPIO_PIN_MAP[idx]
            GPIO.setup(pin, GPIO.OUT, initial=1)  # initial HIGH = safe / OFF
            dev = GpioDevice(idx, self.mac)
            self.devices[dev.euid] = dev
        logging.info(
            "Initialised %d GPIO outputs (active-low). Devices: %s",
            NUM_OUTPUTS,
            list(self.devices.keys()),
        )

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_connect(
        self, client: mqtt.Client, userdata: Any, flags: Any, rc: int
    ) -> None:
        if rc == 0:
            logging.info("MQTT connected to %s:%d", self.broker_ip, self.broker_port)
            client.subscribe(ROOT_TOPIC)
            logging.debug("Subscribed to root topic: %s", ROOT_TOPIC)
            client.subscribe(self.topic)
            logging.debug("Subscribed to instance topic: %s", self.topic)
            self._connected.set()
        else:
            logging.error("MQTT connection refused (rc=%d)", rc)

    def _on_disconnect(
        self, client: mqtt.Client, userdata: Any, rc: int
    ) -> None:
        self._connected.clear()
        logging.warning("MQTT disconnected (rc=%d)", rc)

    def _on_message(
        self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage
    ) -> None:
        self._msg_queue.put((msg.topic, msg.payload))

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        logging.info("Connecting to MQTT broker %s:%d …", self.broker_ip, self.broker_port)
        self._mqtt.connect_async(self.broker_ip, self.broker_port, keepalive=60)
        self._mqtt.loop_start()

        try:
            while self.running:
                try:
                    topic, payload = self._msg_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                self._process_command(topic, payload)
        finally:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
            GPIO.cleanup()
            logging.info("GPIO server stopped.")

    def _process_command(self, topic: str, payload: bytes) -> None:
        if not topic or payload == b"":
            logging.warning("Ignore empty topic or payload")
            return

        try:
            msg: dict[str, Any] = json.loads(payload)
        except Exception:
            logging.warning("Failed to parse JSON: %s", payload)
            return

        if msg is None:
            logging.debug("Ignore None message")
            return
        if "rsp" in msg:
            logging.debug("Ignore response message: %s", msg.get("rsp"))
            return
        if "cmd" not in msg:
            logging.warning("Invalid message — no 'cmd' field: %s", msg)
            return
        if "ver" not in msg:
            logging.debug("Invalid message — missing 'ver' field")
            return
        if "mid" not in msg:
            logging.debug("Invalid message — missing 'mid' field")
            return

        cmd: str = msg["cmd"]
        handler = self.handlers.get(cmd)
        if handler is None:
            logging.error("Unrecognised command: %s", cmd)
            return
        handler(msg)

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------

    def _handle_hello(self, msg: dict[str, Any]) -> None:
        """Announce server identity/mac/count/available.

        Request : {"cmd": "hello", "ver": 1, "mid": <mid>}
        Response: {"rsp": "hello", "ver": 1, "mid": <mid>, "id": <id>,
                   "mac": <mac>, "topic": <topic>,
                   "count": 8, "available": <n>}
        """
        logging.debug("Handle hello")
        payload = {
            "rsp": "hello",
            "ver": 1,
            "mid": msg["mid"],
            "id": self.stick_server_id,
            "mac": self.mac,
            "topic": self.topic,
            "count": NUM_OUTPUTS,
            "available": self._available_count(),
        }
        self._publish(self.topic, json.dumps(payload))

    def _handle_list(self, msg: dict[str, Any]) -> None:
        """Return all 8 logical devices with reservation state.

        Request : {"cmd": "list", "ver": 1, "mid": <mid>}
        Response: {"rsp": "list", "ver": 1, "mid": <mid>,
                   "status": 0,
                   "value": [{"euid": <euid>, "reserved": bool,
                               "owner": <owner>, "gpio_pin": <pin>}, …]}
        """
        logging.debug("Handle list")
        value = [
            {
                "euid": d.euid,
                "reserved": d.reserved,
                "owner": d.owner,
                "gpio_pin": d.gpio_pin,
            }
            for d in self.devices.values()
        ]
        payload = {
            "rsp": "list",
            "ver": 1,
            "mid": msg["mid"],
            "status": SUCCESS,
            "value": value,
        }
        self._publish(self.topic, json.dumps(payload))

    def _handle_reserve(self, msg: dict[str, Any]) -> None:
        """Reserve *count* GPIO outputs for *owner*.

        Request : {"cmd": "reserve", "ver": 1, "mid": <mid>,
                   "owner": <str>, "count": <int>,
                   "ntype": <str>}   ← ntype accepted but ignored
        Response: {"rsp": "reserve", "ver": 1, "mid": <mid>,
                   "owner": <owner>, "status": 0|1,
                   "new": "all"|"some"|"none", "euids": [<euid>, …]}
        """
        logging.debug("Handle reserve")
        mid = msg["mid"]

        if self._check_field(msg, "owner", "reserve", mid):
            return
        owner: str = msg["owner"]

        if self._check_field(msg, "count", "reserve", mid):
            return
        count: int = msg["count"]

        # ntype accepted for protocol compat; GPIO outputs are type-agnostic
        msg.get("ntype", "")

        # Devices already held by this owner
        already: list[str] = [d.euid for d in self.devices.values() if d.owner == owner]

        new_flag = "none"
        if len(already) < count:
            need = count - len(already)
            new_flag = "all" if need == count else "some"
            newly = self._reserve_n(need, owner)
            already = already + newly

        payload = {
            "rsp": "reserve",
            "ver": 1,
            "mid": mid,
            "owner": owner,
            "status": SUCCESS,
            "new": new_flag,
            "euids": already,
        }
        self._publish(self.topic, json.dumps(payload))

    def _handle_release(self, msg: dict[str, Any]) -> None:
        """Release devices by owner name or explicit euid list.

        Request : {"cmd": "release", "ver": 1, "mid": <mid>,
                   "owner": <str>, "euids": [<euid>, …]}
        Response: {"rsp": "release", "ver": 1, "mid": <mid>,
                   "status": 0, "euid": [<released-euids>]}
        """
        logging.debug("Handle release: %s", msg)
        mid = msg["mid"]

        if self._check_field(msg, "owner", "release", mid):
            return
        owner: str = msg["owner"]

        if self._check_field(msg, "euids", "release", mid):
            return
        euids: list[str] = msg["euids"]

        released: list[str] = []

        if owner:
            for d in self.devices.values():
                if d.owner == owner:
                    self._release_device(d)
                    released.append(d.euid)
        elif euids:
            for d in self.devices.values():
                if d.euid in euids:
                    self._release_device(d)
                    released.append(d.euid)

        self._publish(self.topic, self._build_status_response("release", mid, SUCCESS, released))
        logging.debug("Released %d device(s)", len(released))

    def _handle_status(self, msg: dict[str, Any]) -> None:
        """Return state for requested euids.

        Request : {"cmd": "status", "ver": 1, "mid": <mid>,
                   "euids": [<euid>, …]}
        Response: {"rsp": "status", "ver": 1, "mid": <mid>,
                   "status": 0,
                   "value": [{"euid": <euid>, "reserved": bool,
                               "owner": <owner>, "gpio_pin": <pin>}, …]}
        """
        logging.debug("Handle status")
        mid = msg["mid"]

        if self._check_field(msg, "euids", "status", mid):
            return
        euids: list[str] = msg["euids"]

        value = []
        for euid in euids:
            d = self.devices.get(euid)
            if d is not None:
                value.append(
                    {
                        "euid": d.euid,
                        "reserved": d.reserved,
                        "owner": d.owner,
                        "gpio_pin": d.gpio_pin,
                    }
                )
            else:
                logging.warning("status: unknown euid %s", euid)

        payload = {
            "rsp": "status",
            "ver": 1,
            "mid": mid,
            "status": SUCCESS,
            "value": value,
        }
        self._publish(self.topic, json.dumps(payload))

    def _handle_join(self, msg: dict[str, Any]) -> None:
        """Activate a single GPIO output (equivalent to joining a network).

        Semantics: drives the GPIO LOW (active-low = ON) without changing
        reservation ownership — useful for direct single-output activation.

        Request : {"cmd": "join", "ver": 1, "mid": <mid>, "euid": <euid>}
        Response: {"rsp": "join", "ver": 1, "mid": <mid>,
                   "status": 0|1, "euid": <euid>}
        """
        logging.debug("Handle join")
        mid = msg["mid"]

        if self._check_field(msg, "euid", "join", mid):
            return
        euid: str = msg["euid"]

        d = self.devices.get(euid)
        if d is None:
            logging.warning("join: unknown euid %s", euid)
            self._publish(self.topic, self._build_status_response("join", mid, FAILURE, euid))
            return

        d.activate()
        self._publish(self.topic, self._build_status_response("join", mid, SUCCESS, euid))

    def _handle_leave(self, msg: dict[str, Any]) -> None:
        """Deactivate GPIO outputs (equivalent to leaving a network).

        Semantics: drives the GPIO HIGH (active-low = OFF) without changing
        reservation ownership.

        Request : {"cmd": "leave", "ver": 1, "mid": <mid>,
                   "euids": [<euid>, …]}
        Response: {"rsp": "leave", "ver": 1, "mid": <mid>,
                   "status": 0, "euid": <euid>}  ← one response per euid
        """
        logging.debug("Handle leave: %s", msg.get("euids"))
        mid = msg["mid"]

        if self._check_field(msg, "euids", "leave", mid):
            return
        euids: list[str] = msg["euids"]

        for euid in euids:
            d = self.devices.get(euid)
            if d is None:
                logging.warning("leave: unknown euid %s", euid)
                continue
            d.deactivate()
            self._publish(self.topic, self._build_status_response("leave", mid, SUCCESS, euid))

    def _handle_reboot(self, msg: dict[str, Any]) -> None:
        """Pulse a GPIO output (brief LOW then back to HIGH — simulates reboot).

        Semantics: activates the GPIO for 500 ms then deactivates, producing a
        momentary relay closure without leaving the output permanently ON.

        Request : {"cmd": "reboot", "ver": 1, "mid": <mid>, "euid": <euid>}
        Response: {"rsp": "reboot", "ver": 1, "mid": <mid>,
                   "status": 0, "euid": <euid>}
        """
        logging.debug("Handle reboot")
        mid = msg["mid"]

        if self._check_field(msg, "euid", "reboot", mid):
            return
        euid: str = msg["euid"]

        d = self.devices.get(euid)
        if d is None:
            logging.warning("reboot: unknown euid %s", euid)
            self._publish(self.topic, self._build_status_response("reboot", mid, FAILURE, euid))
            return

        # Run pulse in a background thread so we don't block the message loop
        threading.Thread(target=d.pulse, args=(0.5,), daemon=True).start()
        self._publish(self.topic, self._build_status_response("reboot", mid, SUCCESS, euid))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _available_count(self) -> int:
        return sum(1 for d in self.devices.values() if not d.reserved)

    def _reserve_n(self, count: int, owner: str) -> list[str]:
        """Reserve up to *count* free devices for *owner*; returns list of euids."""
        euids: list[str] = []
        for d in self.devices.values():
            if len(euids) >= count:
                break
            if not d.reserved:
                d.reserved = True
                d.owner = owner
                d.activate()
                euids.append(d.euid)
                logging.debug("Reserved %s for owner %r", d.euid, owner)
        return euids

    def _release_device(self, d: GpioDevice) -> None:
        d.reserved = False
        d.owner = ""
        d.deactivate()

    def _check_field(
        self,
        msg: dict[str, Any],
        field: str,
        cmd: str,
        mid: Any,
    ) -> bool:
        """Return True (and publish error response) if *field* is missing from *msg*."""
        if field not in msg:
            logging.debug("Invalid %s message — missing field %r", cmd, field)
            self._publish(
                self.topic,
                self._build_status_response(cmd, mid, FAILURE, []),
            )
            return True
        return False

    def _build_status_response(
        self, cmd: str, mid: Any, status: int, euid: Any
    ) -> str:
        return json.dumps({"rsp": cmd, "ver": 1, "mid": mid, "status": status, "euid": euid})

    def _publish(self, topic: str, message: str) -> None:
        result = self._mqtt.publish(topic, message)
        logging.debug("Published to %s: %s (rc=%s)", topic, message, result.rc)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def runner() -> None:
    server = GpioServer()
    server.run()


if __name__ == "__main__":
    runner()
