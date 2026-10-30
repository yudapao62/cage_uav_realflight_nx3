#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/nx3/cage_uav_ws
export PYTHONPATH="$ROOT/src/cage_site_db${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m cage_site_db.cli backup "$@"
