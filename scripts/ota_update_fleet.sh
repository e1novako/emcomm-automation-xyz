#!/usr/bin/env bash
# Build and OTA-upload C4-VIBRANT firmware to a fleet of devices.
#
# Usage:
#   scripts/ota_update_fleet.sh [-p PASSWORD] [-s PROJECT_DIR] [IP ...]
#
# If no IPs are given, defaults to the known device fleet below.
# Edit DEFAULT_IPS to add/remove devices as the fleet changes.
set -uo pipefail

PASSWORD="${ARDUINO_OTA_PASSWORD:-Fiber714Cvet}"
PROJECT_DIR="C4-VIBRANT"
FQBN="esp8266:esp8266:nodemcuv2"
BUILD_DIR_NAME="esp8266.esp8266.nodemcuv2"
RETRIES=3
RETRY_DELAY=5

DEFAULT_IPS=(
  192.168.1.194
  192.168.1.81 192.168.1.82 192.168.1.83 192.168.1.84 192.168.1.85
  192.168.1.86 192.168.1.87 192.168.1.88 192.168.1.89 192.168.1.90
  192.168.1.91 192.168.1.92 192.168.1.93 192.168.1.94 192.168.1.95
)

while getopts "p:s:h" opt; do
  case "$opt" in
    p) PASSWORD="$OPTARG" ;;
    s) PROJECT_DIR="$OPTARG" ;;
    h)
      echo "Usage: $0 [-p PASSWORD] [-s PROJECT_DIR] [IP ...]"
      exit 0
      ;;
    *)
      echo "Usage: $0 [-p PASSWORD] [-s PROJECT_DIR] [IP ...]" >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))

IPS=("$@")
if [ ${#IPS[@]} -eq 0 ]; then
  IPS=("${DEFAULT_IPS[@]}")
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_ROOT/$PROJECT_DIR/build/$BUILD_DIR_NAME"

export PATH="$HOME/bin:$PATH"
cd "$REPO_ROOT"

echo "==> Cleaning stale build artifacts in $BUILD_DIR"
rm -f "$BUILD_DIR"/*.bin "$BUILD_DIR"/*.elf "$BUILD_DIR"/*.map 2>/dev/null

echo "==> Compiling $PROJECT_DIR for $FQBN"
arduino-cli compile --fqbn "$FQBN" --export-binaries "$PROJECT_DIR"
if [ $? -ne 0 ]; then
  echo "Build FAILED, aborting fleet upload." >&2
  exit 1
fi

declare -A RESULTS
for ip in "${IPS[@]}"; do
  echo "=== $ip ==="
  ok=0
  for attempt in $(seq 1 "$RETRIES"); do
    out=$(arduino-cli upload -p "$ip" -l network --fqbn "$FQBN" \
      -F password="$PASSWORD" --input-dir "$BUILD_DIR" "$PROJECT_DIR" 2>&1)
    echo "$out" | tail -3
    if echo "$out" | grep -q "New upload port"; then
      ok=1
      break
    fi
    echo "retrying $ip (attempt $attempt/$RETRIES)..."
    sleep "$RETRY_DELAY"
  done
  RESULTS["$ip"]=$([ $ok -eq 1 ] && echo "OK" || echo "FAILED")
done

echo
echo "===== SUMMARY ====="
for ip in "${IPS[@]}"; do
  echo "$ip: ${RESULTS[$ip]}"
done
