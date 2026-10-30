#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/nx3/cage_uav_ws
STATE=/home/nx3/.cage_site_db
PID_FILE=$STATE/pid
LOG=$STATE/site-db.log
mkdir -p "$STATE"
if [[ -s "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "site-db already running pid=$(cat "$PID_FILE")"
  exit 0
fi
export PYTHONPATH="$ROOT/src/cage_site_db${PYTHONPATH:+:$PYTHONPATH}"
nohup /usr/bin/python3 "$ROOT/src/cage_site_db/scripts/cage_site_db_server" --bind 127.0.0.1 --port 8090 >>"$LOG" 2>&1 &
echo $! > "$PID_FILE"
for _ in $(seq 1 30); do
  if curl -fsS http://127.0.0.1:8090/api/site-db/health >/dev/null; then
    echo "site-db started pid=$(cat "$PID_FILE")"
    exit 0
  fi
  sleep 0.2
done
echo "site-db failed; see $LOG" >&2
exit 1
