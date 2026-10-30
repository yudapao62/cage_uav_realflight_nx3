#!/usr/bin/env bash
set -euo pipefail
PID_FILE=/home/nx3/.cage_site_db/pid
if [[ -s "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  kill "$(cat "$PID_FILE")"
  for _ in $(seq 1 20); do
    kill -0 "$(cat "$PID_FILE")" 2>/dev/null || break
    sleep 0.1
  done
fi
rm -f "$PID_FILE"
echo "site-db stopped"
