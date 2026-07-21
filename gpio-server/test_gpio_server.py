#!/usr/bin/env python3
"""
Unit tests for the GPIO Server protocol logic.

No MQTT broker, no physical GPIO, and no RPi hardware are required.
All GPIO calls are intercepted by MockGPIO (used automatically when
RPi.GPIO is absent), and MQTT publish calls are monkey-patched.

Run with:  python3 test_gpio_server.py
"""

import json
import queue
import sys
import unittest
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Patch RPi.GPIO before importing gpio_server so MockGPIO is used everywhere
# ---------------------------------------------------------------------------
import importlib
import types

# Ensure RPi.GPIO is absent so MockGPIO kicks in
sys.modules.pop("RPi", None)
sys.modules.pop("RPi.GPIO", None)

# Now import the module under test
import importlib.util
import os

_MODULE_PATH = os.path.join(os.path.dirname(__file__), "gpio_server.py")
_spec = importlib.util.spec_from_file_location("gpio_server", _MODULE_PATH)
_mod = importlib.util.module_from_spec(_spec)  # type: ignore[arg-type]
_spec.loader.exec_module(_mod)  # type: ignore[union-attr]

GpioServer = _mod.GpioServer
GpioDevice = _mod.GpioDevice
SUCCESS = _mod.SUCCESS
FAILURE = _mod.FAILURE
NUM_OUTPUTS = _mod.NUM_OUTPUTS
ROOT_TOPIC = _mod.ROOT_TOPIC


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_server() -> GpioServer:
    """Create a GpioServer bypassing argparse and MQTT connection."""
    with patch("sys.argv", ["gpio_server"]):
        server = GpioServer.__new__(GpioServer)
        # Minimal manual init (skip argparse / network)
        server.broker_ip = "127.0.0.1"
        server.broker_port = 1883
        server.stick_server_id = ""
        server.mac = "aabbccddeeff"
        server.running = False
        server.topic = f"{ROOT_TOPIC}/ssvr-{server.mac}"
        server._msg_queue = queue.Queue()
        server.devices = {}
        server._connected = MagicMock()
        # Initialise GPIO devices
        server._init_gpio()
        # Wire up handlers
        server.handlers = {
            "hello": server._handle_hello,
            "list": server._handle_list,
            "reserve": server._handle_reserve,
            "release": server._handle_release,
            "status": server._handle_status,
            "join": server._handle_join,
            "leave": server._handle_leave,
            "reboot": server._handle_reboot,
        }
        # Replace publish with a recorder
        server._published: list[tuple[str, dict]] = []  # type: ignore[attr-defined]
        def _fake_publish(topic: str, message: str) -> None:
            server._published.append((topic, json.loads(message)))
        server._publish = _fake_publish  # type: ignore[method-assign]
        return server


def _last(server: GpioServer) -> dict:
    return server._published[-1][1]  # type: ignore[attr-defined]


class TestGpioServerProtocol(unittest.TestCase):

    def setUp(self) -> None:
        self.server = _make_server()

    # ------------------------------------------------------------------ hello

    def test_hello_response_fields(self) -> None:
        msg = {"cmd": "hello", "ver": 1, "mid": 42}
        self.server._handle_hello(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "hello")
        self.assertEqual(rsp["mid"], 42)
        self.assertEqual(rsp["count"], NUM_OUTPUTS)
        self.assertEqual(rsp["available"], NUM_OUTPUTS)
        self.assertIn("mac", rsp)
        self.assertIn("topic", rsp)

    # ------------------------------------------------------------------- list

    def test_list_returns_all_devices(self) -> None:
        msg = {"cmd": "list", "ver": 1, "mid": 1}
        self.server._handle_list(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "list")
        self.assertEqual(rsp["status"], SUCCESS)
        self.assertEqual(len(rsp["value"]), NUM_OUTPUTS)
        # All unreserved at start
        for entry in rsp["value"]:
            self.assertFalse(entry["reserved"])
            self.assertEqual(entry["owner"], "")

    # ----------------------------------------------------------------- reserve

    def test_reserve_allocates_devices(self) -> None:
        msg = {"cmd": "reserve", "ver": 1, "mid": 2,
               "owner": "agent-1", "count": 3, "ntype": "gpio"}
        self.server._handle_reserve(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "reserve")
        self.assertEqual(rsp["status"], SUCCESS)
        self.assertEqual(len(rsp["euids"]), 3)
        self.assertEqual(rsp["new"], "all")

    def test_reserve_idempotent_for_same_owner(self) -> None:
        # First reserve 2
        msg1 = {"cmd": "reserve", "ver": 1, "mid": 10,
                "owner": "agent-x", "count": 2, "ntype": "gpio"}
        self.server._handle_reserve(msg1)
        # Ask for same count again — should return the same 2 (new="none")
        msg2 = {"cmd": "reserve", "ver": 1, "mid": 11,
                "owner": "agent-x", "count": 2, "ntype": "gpio"}
        self.server._handle_reserve(msg2)
        rsp = _last(self.server)
        self.assertEqual(len(rsp["euids"]), 2)
        self.assertEqual(rsp["new"], "none")

    def test_reserve_missing_owner_returns_failure(self) -> None:
        msg = {"cmd": "reserve", "ver": 1, "mid": 3,
               "count": 1, "ntype": "gpio"}
        self.server._handle_reserve(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["status"], FAILURE)

    def test_reserve_partial_allocation(self) -> None:
        # Reserve 6 first, then ask for 8 — only 2 new available
        msg1 = {"cmd": "reserve", "ver": 1, "mid": 20,
                "owner": "a", "count": 6, "ntype": "gpio"}
        self.server._handle_reserve(msg1)
        msg2 = {"cmd": "reserve", "ver": 1, "mid": 21,
                "owner": "a", "count": 8, "ntype": "gpio"}
        self.server._handle_reserve(msg2)
        rsp = _last(self.server)
        self.assertEqual(len(rsp["euids"]), 8)
        self.assertEqual(rsp["new"], "some")

    # ----------------------------------------------------------------- release

    def test_release_by_owner(self) -> None:
        # Reserve 2 devices
        msg_res = {"cmd": "reserve", "ver": 1, "mid": 30,
                   "owner": "agt", "count": 2, "ntype": "gpio"}
        self.server._handle_reserve(msg_res)

        # Release by owner
        msg_rel = {"cmd": "release", "ver": 1, "mid": 31,
                   "owner": "agt", "euids": []}
        self.server._handle_release(msg_rel)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "release")
        self.assertEqual(rsp["status"], SUCCESS)
        self.assertEqual(len(rsp["euid"]), 2)

        # Devices should now be free
        self.assertEqual(self.server._available_count(), NUM_OUTPUTS)

    def test_release_by_euid(self) -> None:
        msg_res = {"cmd": "reserve", "ver": 1, "mid": 40,
                   "owner": "agt2", "count": 2, "ntype": "gpio"}
        self.server._handle_reserve(msg_res)
        reserved_rsp = _last(self.server)
        euid_to_release = reserved_rsp["euids"][:1]

        msg_rel = {"cmd": "release", "ver": 1, "mid": 41,
                   "owner": "", "euids": euid_to_release}
        self.server._handle_release(msg_rel)
        rsp = _last(self.server)
        self.assertEqual(rsp["euid"], euid_to_release)
        self.assertEqual(self.server._available_count(), NUM_OUTPUTS - 1)

    # ----------------------------------------------------------------- status

    def test_status_returns_requested_euids(self) -> None:
        msg_res = {"cmd": "reserve", "ver": 1, "mid": 50,
                   "owner": "agt3", "count": 1, "ntype": "gpio"}
        self.server._handle_reserve(msg_res)
        euid = _last(self.server)["euids"][0]

        msg_stat = {"cmd": "status", "ver": 1, "mid": 51,
                    "euids": [euid]}
        self.server._handle_status(msg_stat)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "status")
        self.assertEqual(len(rsp["value"]), 1)
        self.assertTrue(rsp["value"][0]["reserved"])
        self.assertEqual(rsp["value"][0]["owner"], "agt3")

    # -------------------------------------------------------------------- join

    def test_join_activates_device(self) -> None:
        euid = list(self.server.devices.keys())[0]
        msg = {"cmd": "join", "ver": 1, "mid": 60, "euid": euid}
        self.server._handle_join(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "join")
        self.assertEqual(rsp["status"], SUCCESS)
        self.assertEqual(rsp["euid"], euid)

    def test_join_unknown_euid_returns_failure(self) -> None:
        msg = {"cmd": "join", "ver": 1, "mid": 61, "euid": "gpio-invalid-99"}
        self.server._handle_join(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["status"], FAILURE)

    # ------------------------------------------------------------------- leave

    def test_leave_deactivates_device(self) -> None:
        euid = list(self.server.devices.keys())[0]
        # First join (activate)
        self.server._handle_join({"cmd": "join", "ver": 1, "mid": 70, "euid": euid})
        # Then leave (deactivate)
        msg = {"cmd": "leave", "ver": 1, "mid": 71, "euids": [euid]}
        self.server._handle_leave(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "leave")
        self.assertEqual(rsp["status"], SUCCESS)

    # ------------------------------------------------------------------ reboot

    def test_reboot_returns_success_immediately(self) -> None:
        euid = list(self.server.devices.keys())[0]
        msg = {"cmd": "reboot", "ver": 1, "mid": 80, "euid": euid}
        self.server._handle_reboot(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["rsp"], "reboot")
        self.assertEqual(rsp["status"], SUCCESS)
        self.assertEqual(rsp["euid"], euid)

    def test_reboot_unknown_euid_returns_failure(self) -> None:
        msg = {"cmd": "reboot", "ver": 1, "mid": 81, "euid": "gpio-bad-99"}
        self.server._handle_reboot(msg)
        rsp = _last(self.server)
        self.assertEqual(rsp["status"], FAILURE)

    # --------------------------------------------------------- process_command

    def test_process_ignores_response_messages(self) -> None:
        initial_count = len(self.server._published)  # type: ignore[attr-defined]
        self.server._process_command(
            self.server.topic,
            json.dumps({"rsp": "hello", "ver": 1, "mid": 99}).encode(),
        )
        self.assertEqual(len(self.server._published), initial_count)  # type: ignore[attr-defined]

    def test_process_ignores_malformed_json(self) -> None:
        initial_count = len(self.server._published)  # type: ignore[attr-defined]
        self.server._process_command(self.server.topic, b"not-json{{{")
        self.assertEqual(len(self.server._published), initial_count)  # type: ignore[attr-defined]

    def test_process_ignores_missing_ver(self) -> None:
        initial_count = len(self.server._published)  # type: ignore[attr-defined]
        self.server._process_command(
            self.server.topic,
            json.dumps({"cmd": "hello", "mid": 1}).encode(),
        )
        self.assertEqual(len(self.server._published), initial_count)  # type: ignore[attr-defined]

    def test_process_unknown_command_no_crash(self) -> None:
        initial_count = len(self.server._published)  # type: ignore[attr-defined]
        self.server._process_command(
            self.server.topic,
            json.dumps({"cmd": "frobnicate", "ver": 1, "mid": 1}).encode(),
        )
        self.assertEqual(len(self.server._published), initial_count)  # type: ignore[attr-defined]

    # --------------------------------------------------------- topic / mid propagation

    def test_mid_is_echoed_in_response(self) -> None:
        for mid in (0, 1, 9999, "abc"):
            self.server._handle_hello({"cmd": "hello", "ver": 1, "mid": mid})
            self.assertEqual(_last(self.server)["mid"], mid)

    def test_response_published_to_instance_topic(self) -> None:
        self.server._handle_hello({"cmd": "hello", "ver": 1, "mid": 1})
        published_topic = self.server._published[-1][0]  # type: ignore[attr-defined]
        self.assertEqual(published_topic, self.server.topic)

    # --------------------------------------------------------- available count

    def test_available_decrements_on_reserve(self) -> None:
        self.assertEqual(self.server._available_count(), NUM_OUTPUTS)
        msg = {"cmd": "reserve", "ver": 1, "mid": 90,
               "owner": "x", "count": 3, "ntype": "gpio"}
        self.server._handle_reserve(msg)
        self.assertEqual(self.server._available_count(), NUM_OUTPUTS - 3)

    def test_available_restores_on_release(self) -> None:
        msg_res = {"cmd": "reserve", "ver": 1, "mid": 100,
                   "owner": "y", "count": 4, "ntype": "gpio"}
        self.server._handle_reserve(msg_res)
        msg_rel = {"cmd": "release", "ver": 1, "mid": 101,
                   "owner": "y", "euids": []}
        self.server._handle_release(msg_rel)
        self.assertEqual(self.server._available_count(), NUM_OUTPUTS)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("GPIO Server protocol unit tests")
    print("=" * 45)
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestGpioServerProtocol)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
