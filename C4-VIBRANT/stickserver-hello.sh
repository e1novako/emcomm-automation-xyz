#!/usr/bin/env bash
# Broadcast a stickserver "hello" and list every stickserver that answers it.
# Usage: stickserver-hello.sh [-h broker] [-p port] [-w seconds] [-d]
#   -h  MQTT broker host (default 192.168.1.6)
#   -p  MQTT broker port (default 1883)
#   -w  seconds to wait for responses (default 8)
#   -r  hello rounds; replies from all rounds are merged (default 3).
#       Replies are ~1.7 KB QoS-0 bursts and some get lost, so one round
#       can miss stickservers.
#   -d  debug: print raw matching responses to stderr
set -u

HOST=192.168.1.6
PORT=1883
WAIT=8
ROUNDS=3
DEBUG=0
ROOT=s1/c4/stickserver/v1

while getopts "h:p:w:r:d" opt; do
  case $opt in
    h) HOST=$OPTARG ;;
    p) PORT=$OPTARG ;;
    w) WAIT=$OPTARG ;;
    r) ROUNDS=$OPTARG ;;
    d) DEBUG=1 ;;
    *) sed -n '2,10p' "$0"; exit 2 ;;
  esac
done

for tool in mosquitto_sub mosquitto_pub jq; do
  command -v "$tool" >/dev/null || { echo "Missing tool: $tool" >&2; exit 1; }
done

MID="hello-$$-$(date +%s)"
OUT=$(mktemp)
trap 'rm -f "$OUT"' EXIT

# Subscribe first (QoS 1 to limit loss), then send several hello rounds.
# The wait window starts after the first hello, not at subscribe time.
mosquitto_sub -h "$HOST" -p "$PORT" -q 1 -t "$ROOT/#" -W "$((WAIT + ROUNDS + 2))" >"$OUT" 2>/dev/null &
SUB=$!
sleep 1
for n in $(seq 1 "$ROUNDS"); do
  mosquitto_pub -h "$HOST" -p "$PORT" -q 1 -t "$ROOT" \
    -m "{\"cmd\":\"hello\",\"ver\":1,\"mid\":\"$MID-$n\"}" || {
    echo "Failed to publish to $HOST:$PORT" >&2
    kill "$SUB" 2>/dev/null
    exit 1
  }
  sleep 1
done
wait "$SUB" 2>/dev/null

# Keep every hello response seen on the bus, whatever its mid (stickservers
# also answer other peers' hellos, which is how a manual watch sees them);
# one row per stickserver.
ROWS=$(jq -r '
  select(type == "object" and .rsp == "hello")
  | [(.mac // .id // "?"), (.id // "?"),
     ((.ip // "") | if . == "" then null else . end) // .mac // "?",
     ([.devices[]? | (.ntype // empty) | split("[ ,]+"; null) | .[] | select(length > 0)]
       | unique | if length == 0 then "?" else join(",") end),
     (.count // 0), (.available // 0),
     ([.devices[]? | (.ntype // empty) | split("[ ,]+"; null) | .[] | select(length > 0)]
       | group_by(.) | map("\(.[0])=\(length)") | join(";"))] | @tsv
' "$OUT" 2>/dev/null | sort -u -t$'\t' -k1,1 | cut -f2- | sort -V -t$'\t' -k2,2)

[ "$DEBUG" = 1 ] && jq -c \
  'select(type == "object" and .rsp == "hello")' "$OUT" >&2

if [ -z "$ROWS" ]; then
  echo "No stickserver answered hello (mid=$MID) within ${WAIT}s on $HOST:$PORT"
  exit 1
fi

{
  printf 'Stickserver ID\tStickserver IP/MAC\tType\tSticks\tAvailable\tTypeCounts\n'
  printf '%s\n' "$ROWS"
} | awk -F'\t' '
  { for (i=1;i<=5;i++) { c[NR,i]=$i; if (length($i) > w[i]) w[i]=length($i) }
    if (NR > 1) { m = split($6, kv, ";")
      for (j=1;j<=m;j++) { split(kv[j], p, "="); tc[p[1]] += p[2] } } }
  END {
    fmt = "| %-" w[1] "s | %-" w[2] "s | %-" w[3] "s | %" w[4] "s | %" w[5] "s |\n"
    sep = "+"
    for (i=1;i<=5;i++) sep = sep "-" sprintf("%*s", w[i], "") "-+"
    gsub(/ /, "-", sep)
    print sep; printf fmt, c[1,1], c[1,2], c[1,3], c[1,4], c[1,5]; print sep
    for (r=2; r<=NR; r++) { printf fmt, c[r,1], c[r,2], c[r,3], c[r,4], c[r,5]
      tn += c[r,4]; ta += c[r,5] }
    print sep
    print ""
    print "Devices per type:"
    for (t in tc) printf "  %-20s %d\n", t, tc[t] | "sort"
    close("sort")
    print ""
    printf "Stickservers: %d   Total sticks: %d   Total available: %d\n", NR-1, tn, ta
  }'
