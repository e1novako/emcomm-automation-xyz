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

Required fields for every state message
----------------------------------------
  fw         : str
  axis       : str
  moving     : bool
  pos        : int
  direction  : str  — "positive" | "negative"
  endstops   : dict — {begin, end, both_error}
  test_mode  : dict — {active, cycle, target}
  steps_done : int
  timestamp  : str
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

REQUIRED_STATE_KEYS = {"fw", "axis", "moving", "pos", "direction",
                       "endstops", "test_mode", "steps_done", "timestamp"}


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


def validate_state_payload(raw: str) -> dict:
    """Parse *raw* JSON string and assert it matches the state schema."""
    payload = json.loads(raw)

    missing = REQUIRED_STATE_KEYS - payload.keys()
    assert not missing, f"Missing state keys: {missing}"

    assert isinstance(payload["fw"], str) and payload["fw"], \
        "fw must be a non-empty string"

    assert payload["axis"] in ("vertical", "horizontal"), \
        f"axis must be 'vertical' or 'horizontal', got '{payload['axis']}'"

    assert isinstance(payload["moving"], bool), \
        f"moving must be bool, got {type(payload['moving']).__name__}"

    assert isinstance(payload["pos"], int), \
        f"pos must be int, got {type(payload['pos']).__name__}"

    assert payload["direction"] in ("positive", "negative"), \
        f"direction must be 'positive' or 'negative', got '{payload['direction']}'"

    endstops = payload["endstops"]
    assert isinstance(endstops, dict), "endstops must be a dict"
    for k in ("begin", "end", "both_error"):
        assert k in endstops, f"endstops missing key '{k}'"
        assert isinstance(endstops[k], bool), f"endstops.{k} must be bool"

    tm = payload["test_mode"]
    assert isinstance(tm, dict), "test_mode must be a dict"
    for k in ("active", "cycle", "target"):
        assert k in tm, f"test_mode missing key '{k}'"
    assert isinstance(tm["active"], bool), "test_mode.active must be bool"
    assert isinstance(tm["cycle"], int), "test_mode.cycle must be int"
    assert tm["target"] in ("end", "begin"), \
        f"test_mode.target must be 'end' or 'begin', got '{tm['target']}'"

    assert isinstance(payload["steps_done"], int), \
        f"steps_done must be int, got {type(payload['steps_done']).__name__}"

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


def make_state_payload(fw: str = "0.93", axis: str = "vertical",
                       moving: bool = False, pos: int = 0,
                       direction: str = "positive",
                       endstops: dict | None = None,
                       test_mode: dict | None = None,
                       steps_done: int = 0,
                       timestamp: str | None = None) -> str:
    """Build a JSON state payload string (mirrors firmware publishState())."""
    if timestamp is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    if endstops is None:
        endstops = {"begin": False, "end": False, "both_error": False}
    if test_mode is None:
        test_mode = {"active": False, "cycle": 0, "target": "end"}
    return json.dumps({
        "fw":        fw,
        "axis":      axis,
        "moving":    moving,
        "pos":       pos,
        "direction": direction,
        "endstops":  endstops,
        "test_mode": test_mode,
        "steps_done": steps_done,
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


# ── state payload tests ───────────────────────────────────────────────────────

def test_state_idle() -> None:
    """State payload when motor is idle at position 1240, positive direction."""
    raw = make_state_payload(
        fw="0.93", axis="vertical", moving=False, pos=1240,
        direction="positive",
        endstops={"begin": False, "end": False, "both_error": False},
        test_mode={"active": False, "cycle": 0, "target": "end"},
        steps_done=1240,
        timestamp="2025-01-15T14:32:07Z",
    )
    p = validate_state_payload(raw)
    assert p["moving"] is False
    assert p["pos"] == 1240
    assert p["direction"] == "positive"
    assert p["steps_done"] == 1240
    assert p["test_mode"]["active"] is False
    print(f"  PASS  state idle (moving=false)     payload={raw}")


def test_state_moving() -> None:
    """State payload while motor is actively moving (steps_done is live counter)."""
    raw = make_state_payload(
        fw="0.93", axis="horizontal", moving=True, pos=300,
        direction="negative",
        endstops={"begin": False, "end": False, "both_error": False},
        test_mode={"active": False, "cycle": 0, "target": "end"},
        steps_done=300,
        timestamp="T+8200ms",
    )
    p = validate_state_payload(raw)
    assert p["moving"] is True
    assert p["direction"] == "negative"
    assert p["axis"] == "horizontal"
    assert UPTIME_RE.match(p["timestamp"])
    print(f"  PASS  state moving (moving=true)    payload={raw}")


def test_state_endstop_error() -> None:
    """State payload when both endstops are pressed simultaneously (error)."""
    raw = make_state_payload(
        fw="0.93", axis="vertical", moving=False, pos=0,
        direction="negative",
        endstops={"begin": True, "end": True, "both_error": True},
        test_mode={"active": False, "cycle": 0, "target": "end"},
        steps_done=0,
        timestamp="2025-01-15T14:35:00Z",
    )
    p = validate_state_payload(raw)
    assert p["endstops"]["both_error"] is True
    assert p["endstops"]["begin"] is True
    assert p["endstops"]["end"] is True
    print(f"  PASS  state endstop error           payload={raw}")


def test_state_missing_key_rejected() -> None:
    """State payload missing a required key should be rejected."""
    payload = {
        "fw": "0.93", "axis": "vertical", "moving": False, "pos": 0,
        "direction": "positive",
        "endstops": {"begin": False, "end": False, "both_error": False},
        # test_mode missing
        "steps_done": 0, "timestamp": "T+0ms",
    }
    raw = json.dumps(payload)
    try:
        validate_state_payload(raw)
        raise AssertionError("Should have raised on missing 'test_mode'")
    except AssertionError as exc:
        if "Missing state keys" in str(exc):
            print("  PASS  state missing key correctly rejected")
        else:
            raise


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
    test_state_idle,
    test_state_moving,
    test_state_endstop_error,
    test_state_missing_key_rejected,
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
