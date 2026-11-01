#!/usr/bin/env bash
# Builds the sketch and uploads it over ArduinoOTA. Settings come from ota.local.env (git-ignored):
#   OTA_IP=...  OTA_PORT=8266  OTA_PASS=...
# Under WSL the upload runs through Windows Python, because the device must connect back to the
# uploading host and cannot reach a WSL (NAT) address.
set -euo pipefail
cd "$(dirname "$0")"

[ -f ota.local.env ] || { echo "ota.local.env missing (OTA_IP, OTA_PORT, OTA_PASS)" >&2; exit 1; }
source ota.local.env
: "${OTA_IP:?}" "${OTA_PASS:?}"
OTA_PORT="${OTA_PORT:-8266}"
FQBN="esp8266:esp8266:d1_mini:eesz=4M2M"
CORE_VER=3.1.2

# arduino-cli needs the sketch folder to match the .ino name
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/TCS_Intercom"
cp TCS_Intercom.ino tcs_protocol.cpp tcs_protocol.h "$work/TCS_Intercom/"
arduino-cli compile --fqbn "$FQBN" --output-dir "$work/build" "$work/TCS_Intercom" | tail -1
bin="$work/build/TCS_Intercom.ino.bin"

if grep -qi microsoft /proc/version 2>/dev/null; then
  localappdata=$(cd /mnt/c && cmd.exe /c 'echo %LOCALAPPDATA%' 2>/dev/null | tr -d '\r')
  winbin="$localappdata\\Temp\\TCS_Intercom.ino.bin"
  cp "$bin" "$(wslpath "$winbin")"
  espota="$localappdata\\Arduino15\\packages\\esp8266\\hardware\\esp8266\\$CORE_VER\\tools\\espota.py"
  # no quotes: cmd.exe passes them through literally (these paths have no spaces)
  (cd /mnt/c && cmd.exe /c "python $espota -i $OTA_IP -p $OTA_PORT -a $OTA_PASS -f $winbin -r")
else
  python3 ~/.arduino15/packages/esp8266/hardware/esp8266/$CORE_VER/tools/espota.py \
    -i "$OTA_IP" -p "$OTA_PORT" -a "$OTA_PASS" -f "$bin" -r
fi
