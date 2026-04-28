#!/usr/bin/env python3
"""
Simulation / validation test for the MQTT JSON event payloads
emitted by the C4-ROBOT firmware (nodemcu_webgui.ino ≥ 0.92).

Run with:  python3 test_event_payload.py

This script does NOT require a physical device or a running MQTT broker.
It verifies that the expected payload schema is stable and that sample
payloads produced here conform to the documented format.

Required fields for every event message
---------------------------------------
  event     : str  — "motion_started" | "endstop_reached"
  command   : str  — non-empty command identifier
  steps     : int  — current step position counter
  timestamp : str  — ISO-8601 UTC (or "T+<millis>ms" uptime fallback)
"""

import json
import re
import sys
from datetime import datetime, timezone

# ── helpers ───────────────────────────────────────────────────────────────────

ISO8601_RE = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$"
)
UPTIME_RE = re.compile(r"^T\+\d+ms$")

REQUIRED_KEYS = {"event", "command", "steps", "timestamp"}
VALID_EVENTS  = {"motion_started", "endstop_reached"}


def validate_payload(raw: str) -> dict:
    """Parse *raw* JSON string and assert it matches the event schema."""
    payload = json.loads(raw)

    missing = REQUIRED_KEYS - payload.keys()
    assert not missing, f"Missing keys: {missing}"

    assert payload["event"] in VALID_EVENTS, (
        f"Unknown event '{payload['event']}', expected one of {VALID_EVENTS}"
    )

    assert isinstance(payload["command"], str) and payload["command"], \
        "command must be a non-empty string (firmware falls back to 'unknown')"

    assert isinstance(payload["steps"], int), \
        f"steps must be int, got {type(payload['steps']).__name__}"

    ts = payload["timestamp"]
    assert ISO8601_RE.match(ts) or UPTIME_RE.match(ts), (
        f"timestamp '{ts}' is neither ISO-8601 UTC nor uptime fallback"
    )

    return payload


def make_payload(event: str, command: str, steps: int,
                 timestamp: str | None = None) -> str:
    """Build a JSON event payload string (mirrors firmware logic)."""
    if timestamp is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    return json.dumps({
        "event":     event,
        "command":   command,
        "steps":     steps,
        "timestamp": timestamp,
    })


# ── test cases ────────────────────────────────────────────────────────────────

def test_motion_started_http_move() -> None:
    raw = make_payload("motion_started", "move:500", 1240)
    p   = validate_payload(raw)
    assert p["event"]   == "motion_started"
    assert p["command"] == "move:500"
    assert p["steps"]   == 1240
    print(f"  PASS  motion_started (HTTP move)  payload={raw}")


def test_endstop_reached_home_begin() -> None:
    raw = make_payload("endstop_reached", "home:begin", 0,
                       timestamp="2025-01-15T14:32:09Z")
    p   = validate_payload(raw)
    assert p["event"]     == "endstop_reached"
    assert p["command"]   == "home:begin"
    assert p["steps"]     == 0
    assert p["timestamp"] == "2025-01-15T14:32:09Z"
    print(f"  PASS  endstop_reached (home:begin) payload={raw}")


def test_motion_started_mqtt_move() -> None:
    raw = make_payload("motion_started", "mqtt-move:-300", -300)
    p   = validate_payload(raw)
    assert p["event"]   == "motion_started"
    assert p["command"] == "mqtt-move:-300"
    assert p["steps"]   == -300
    print(f"  PASS  motion_started (MQTT move)   payload={raw}")


def test_endstop_reached_test_mode() -> None:
    raw = make_payload("endstop_reached", "test:end", 4987)
    p   = validate_payload(raw)
    assert p["event"]   == "endstop_reached"
    assert p["command"] == "test:end"
    assert p["steps"]   == 4987
    print(f"  PASS  endstop_reached (test mode)  payload={raw}")


def test_uptime_timestamp_fallback() -> None:
    """Before NTP sync the firmware emits 'T+<millis>ms'."""
    raw = make_payload("motion_started", "goto:3", 750,
                       timestamp="T+4523ms")
    p   = validate_payload(raw)
    assert UPTIME_RE.match(p["timestamp"]), \
        f"Expected uptime fallback, got: {p['timestamp']}"
    print(f"  PASS  uptime fallback timestamp    payload={raw}")


def test_invalid_event_rejected() -> None:
    raw = make_payload("unknown_event", "move:1", 0)
    try:
        validate_payload(raw)
        raise AssertionError("Should have raised on unknown event")
    except AssertionError as exc:
        if "Unknown event" in str(exc):
            print("  PASS  unknown event correctly rejected")
        else:
            raise


def test_missing_key_rejected() -> None:
    payload = {"event": "motion_started", "steps": 0, "timestamp": "T+0ms"}
    raw = json.dumps(payload)
    try:
        validate_payload(raw)
        raise AssertionError("Should have raised on missing 'command'")
    except AssertionError as exc:
        if "Missing keys" in str(exc):
            print("  PASS  missing key correctly rejected")
        else:
            raise


def test_unknown_command_fallback() -> None:
    """Firmware emits 'unknown' when activeCommand is empty."""
    raw = make_payload("motion_started", "unknown", 0)
    p   = validate_payload(raw)
    assert p["command"] == "unknown"
    print(f"  PASS  unknown command fallback      payload={raw}")


# ── runner ────────────────────────────────────────────────────────────────────

TESTS = [
    test_motion_started_http_move,
    test_endstop_reached_home_begin,
    test_motion_started_mqtt_move,
    test_endstop_reached_test_mode,
    test_uptime_timestamp_fallback,
    test_unknown_command_fallback,
    test_invalid_event_rejected,
    test_missing_key_rejected,
]

if __name__ == "__main__":
    print("C4-ROBOT MQTT event payload validation")
    print("=" * 45)
    failures = 0
    for test in TESTS:
        try:
            test()
        except Exception as exc:  # pylint: disable=broad-except
            print(f"  FAIL  {test.__name__}: {exc}")
            failures += 1
    print("=" * 45)
    if failures:
        print(f"FAILED: {failures}/{len(TESTS)} tests")
        sys.exit(1)
    else:
        print(f"All {len(TESTS)} tests passed.")
