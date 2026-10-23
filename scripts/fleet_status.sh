#!/usr/bin/env bash
# Query uptime and heap usage from a fleet of C4-VIBRANT devices and print
# an aligned table (or Markdown with -m).
#
# Usage:
#   scripts/fleet_status.sh [-p PASSWORD] [-t TIMEOUT] [-m] [IP ...]
#
# If no IPs are given, defaults to the known device fleet below.
# Edit DEFAULT_IPS to add/remove devices as the fleet changes.
set -uo pipefail

PASSWORD="${ARDUINO_OTA_PASSWORD:-Fiber714Cvet}"
TIMEOUT=6
MARKDOWN=0
ENDPOINT="/settings/diagnostics/data"

DEFAULT_IPS=(
  192.168.1.38 192.168.1.39 192.168.1.45
  192.168.1.50 192.168.1.51 192.168.1.52 192.168.1.53
  192.168.1.81 192.168.1.82 192.168.1.83 192.168.1.84 192.168.1.85
  192.168.1.86 192.168.1.87 192.168.1.88 192.168.1.89 192.168.1.90
  192.168.1.91 192.168.1.92 192.168.1.93 192.168.1.94 192.168.1.95
)

usage() { echo "Usage: $0 [-p PASSWORD] [-t TIMEOUT] [-m] [IP ...]"; }

while getopts "p:t:mh" opt; do
  case "$opt" in
    p) PASSWORD="$OPTARG" ;;
    t) TIMEOUT="$OPTARG" ;;
    m) MARKDOWN=1 ;;
    h)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))

IPS=("$@")
if [ ${#IPS[@]} -eq 0 ]; then
  IPS=("${DEFAULT_IPS[@]}")
fi

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

# Query all devices in parallel; each result is "<http_code>\t<body>".
for ip in "${IPS[@]}"; do
  (
    body=$(curl -s -m "$TIMEOUT" -u "admin:$PASSWORD" \
      -w '\n%{http_code}' "http://$ip$ENDPOINT")
    code="${body##*$'\n'}"
    printf '%s\t%s\n' "$code" "${body%$'\n'*}" >"$TMP_DIR/$ip"
  ) &
done
wait

# Print rows sorted by uptime (shortest first, so recent reboots are on top);
# devices that did not respond go last.
for ip in "${IPS[@]}"; do
  printf '%s\t' "$ip"
  cat "$TMP_DIR/$ip"
done | python3 -I -c '
import json, sys

markdown = sys.argv[1] == "1"
header = ["IP", "Hostname", "Version", "Uptime", "Free heap (B)", "Frag",
          "Max block (B)", "OTA"]
right = {4, 5, 6}  # numeric columns
rows = []
failed = []
for line in sys.stdin:
    ip, code, body = (line.rstrip("\n").split("\t", 2) + ["", ""])[:3]
    try:
        d = json.loads(body) if code == "200" else None
    except ValueError:
        d = None
    if d is None:
        reason = {"000": "unreachable", "401": "auth failed"}.get(code, "HTTP " + code)
        failed.append([ip, "-", "-", reason, "-", "-", "-", "-"])
        continue
    rows.append((d.get("uptimeSeconds", 0), [
        ip, str(d.get("hostname", "-")), str(d.get("version", "-")),
        str(d.get("uptime", "-")), "{:,}".format(d.get("freeHeap", 0)),
        "{}%".format(d.get("heapFragmentation", "-")),
        "{:,}".format(d.get("maxFreeBlockSize", 0)),
        "on" if d.get("arduinoOtaEnabled") else "off",
    ]))

ok, fail = len(rows), len(failed)
rows = [r for _, r in sorted(rows, key=lambda t: t[0])] + failed

if markdown:
    print("| " + " | ".join(header) + " |")
    print("|" + "|".join("---:" if i in right else "---" for i in range(len(header))) + "|")
    for r in rows:
        print("| " + " | ".join(r) + " |")
else:
    widths = [max(len(r[i]) for r in [header] + rows) for i in range(len(header))]
    def fmt(r):
        return "  ".join(c.rjust(w) if i in right else c.ljust(w)
                         for i, (c, w) in enumerate(zip(r, widths))).rstrip()
    print(fmt(header))
    print("  ".join("-" * w for w in widths))
    for r in rows:
        print(fmt(r))
print("\n{} responded, {} failed.".format(ok, fail))
' "$MARKDOWN"
