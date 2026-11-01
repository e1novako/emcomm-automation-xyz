#!/usr/bin/env bash
# Stop hook: if the firmware sources changed since the last successful OTA, build and upload them.
# The hash of the last uploaded sources is kept in .claude/.ota_last_hash (git-ignored).
cd "$(dirname "$0")/.." || exit 0
hash_file=.claude/.ota_last_hash
cur=$(cat TCS_Intercom.ino tcs_protocol.cpp tcs_protocol.h | sha256sum | cut -d' ' -f1)
[ "$cur" = "$(cat "$hash_file" 2>/dev/null)" ] && exit 0

ver=$(grep -m1 '#define VERSION' TCS_Intercom.ino | cut -d'"' -f2)
log=$(mktemp)
if ./ota_upload.sh >"$log" 2>&1; then
  echo "$cur" > "$hash_file"
  msg="OTA: firmware $ver uploaded to the intercom"
else
  msg="OTA: build/upload of $ver FAILED, see $log"
fi
printf '{"systemMessage": "%s"}\n' "$msg"
