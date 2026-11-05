#!/usr/bin/env bash
set -Eeuo pipefail

geometry=circular
input_profile=
feeding=false
start_rviz=true
start_web=true
web_port=8080
check_only=false
gps_port=/dev/ttyUSB1
fcu_port=/dev/ttyUSB0
fcu_url=$fcu_port:115200
session_name=cage_realflight
state_dir=/home/nx3/.cage_realflight_stack
pid_file=$state_dir/pids
profile_file=$state_dir/input_profile
log_dir=$state_dir/logs

usage() {
  echo "Usage: $0 --input-profile ground_gps|photo_coords [--geometry circle|circular|rectangular] [--feeding true|false] [--rviz true|false] [--web true|false] [--web-port 1024..65535] [--check-only]"
}
bool_arg() { [[ "$1" == true || "$1" == false ]] || { echo "Expected true or false, got: $1" >&2; exit 2; }; }
while (($#)); do
  case "$1" in
    --geometry) geometry=${2:?missing geometry}; shift 2 ;;
    --input-profile) input_profile=${2:?missing input profile}; shift 2 ;;
    --feeding) bool_arg "${2:?missing feeding value}"; feeding=$2; shift 2 ;;
    --rviz) bool_arg "${2:?missing rviz value}"; start_rviz=$2; shift 2 ;;
    --web) bool_arg "${2:?missing web value}"; start_web=$2; shift 2 ;;
    --web-port) web_port=${2:?missing web port}; shift 2 ;;
    --check-only) check_only=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done
[[ "$geometry" == circle ]] && geometry=circular
[[ "$geometry" == circular || "$geometry" == rectangular ]] || { echo "Invalid geometry: $geometry" >&2; exit 2; }
input_profile=${input_profile,,}
[[ "$input_profile" == ground_gps || "$input_profile" == photo_coords ]] || {
  echo "--input-profile ground_gps|photo_coords is required" >&2
  exit 2
}
input_profile_ros=${input_profile^^}
[[ "$web_port" =~ ^[0-9]+$ ]] && ((web_port >= 1024 && web_port <= 65535)) || { echo "Invalid web port: $web_port" >&2; exit 2; }
feeder_mode=dry_run
[[ "$feeding" == true ]] && feeder_mode=real

# Required, explicit workspace source order from the preparation specification.
# Catkin setup hooks legitimately inspect variables that may not exist in a
# fresh non-interactive shell, so suspend nounset only while sourcing them.
set +u
source /opt/ros/noetic/setup.bash
if [[ "$input_profile" == ground_gps ]]; then
  source /home/nx3/gps_ws/devel/setup.bash
fi
source /home/nx3/acfly_ws/devel/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
set -u
export ROS_PACKAGE_PATH="/home/nx3/cage_uav_ws/src:/home/nx3/acfly_ws/src:${ROS_PACKAGE_PATH:-}"
if [[ "$input_profile" == ground_gps ]]; then
  export ROS_PACKAGE_PATH="/home/nx3/gps_ws/src:$ROS_PACKAGE_PATH"
  # gps_ws and acfly_ws are independent overlays. Sourcing the later
  # workspaces replaces generated Python/library paths, so restore them.
  export PYTHONPATH="/home/nx3/gps_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="/home/nx3/gps_ws/devel/lib:${LD_LIBRARY_PATH:-}"
fi

errors=0
check() { if "$@" >/dev/null 2>&1; then echo "OK    $*"; else echo "FAIL  $*"; errors=$((errors+1)); fi; }
echo "=== REALFLIGHT STACK PRECHECK ==="
required_paths=(/home/nx3/acfly_ws/devel/setup.bash /home/nx3/cage_uav_ws/devel/setup.bash
                /home/nx3/cage_uav_ws/src/cage_realflight/config/realflight_circle_test.yaml
                /home/nx3/cage_uav_ws/src/cage_site_db/scripts/start_site_db.sh)
[[ "$input_profile" == photo_coords ]] || required_paths+=(/home/nx3/gps_ws/devel/setup.bash)
for path in "${required_paths[@]}"; do
  check test -r "$path"
done
check test -c "$fcu_port"
[[ "$input_profile" == photo_coords ]] || check test -c "$gps_port"
check command -v lsof
[[ "$input_profile" == ground_gps ]] || check command -v exiftool

running_nodes=$(rosnode list 2>/dev/null || true)
mavros_running=false
gps_running=false
grep -qx /mavros <<<"$running_nodes" && mavros_running=true
grep -qx /gps_receiver <<<"$running_nodes" && gps_running=true

if [[ "$input_profile" == ground_gps && -e "$fcu_port" && -e "$gps_port" ]]; then
  fcu_resolved=$(readlink -f "$fcu_port")
  gps_resolved=$(readlink -f "$gps_port")
  if [[ "$fcu_resolved" == "$gps_resolved" ]]; then
    echo "FAIL  FCU and ground GPS resolve to the same device: $fcu_resolved"
    errors=$((errors+1))
  else
    echo "OK    distinct serial devices: FCU=$fcu_resolved GPS=$gps_resolved"
  fi
fi

devices=("$fcu_port")
[[ "$input_profile" == photo_coords ]] || devices+=("$gps_port")
for device in "${devices[@]}"; do
  if [[ -e "$device" ]] && lsof "$device" >/dev/null 2>&1; then
    if [[ "$device" == "$fcu_port" && "$mavros_running" == true ]]; then
      echo "OK    FCU serial is owned by the existing /mavros node: $device"
    elif [[ "$device" == "$gps_port" && "$gps_running" == true ]]; then
      echo "OK    GPS serial is owned by the existing /gps_receiver node: $device"
    else
      echo "FAIL  serial device is already in use by an unexpected process: $device"
      lsof "$device" || true
      errors=$((errors+1))
    fi
  elif [[ -e "$device" ]]; then
    echo "OK    serial device is free: $device"
  fi
done

gps_pkg=$(rospack find gps_receiver 2>/dev/null || true)
mavros_pkg=$(rospack find mavros 2>/dev/null || true)
cage_pkg=$(rospack find cage_realflight 2>/dev/null || true)
printf 'gps_receiver: %s\nmavros:       %s\ncage_realflight: %s\n' "$gps_pkg" "$mavros_pkg" "$cage_pkg"
if [[ "$input_profile" == ground_gps ]]; then
  [[ "$gps_pkg" == /home/nx3/gps_ws/* ]] || { echo "ERROR: gps_receiver is not from gps_ws"; errors=$((errors+1)); }
else
  echo "SKIP  gps_receiver package/hardware (PHOTO_COORDS)"
fi
[[ "$mavros_pkg" == /home/nx3/acfly_ws/* ]] || { echo "ERROR: mavros is not from acfly_ws"; errors=$((errors+1)); }
[[ "$cage_pkg" == /home/nx3/cage_uav_ws/* ]] || { echo "ERROR: cage_realflight is not from cage_uav_ws"; errors=$((errors+1)); }

if rosnode list >/dev/null 2>&1; then
  echo "ROS master: OK"
else
  echo "ROS master: NOT RUNNING (will be created by the first roslaunch)"
fi

conflict_nodes=$(rosnode list 2>/dev/null | grep -Ei 'uav_control_bridge|offboard' || true)
setpoint_info=$(rostopic info /mavros/setpoint_raw/local 2>/dev/null || true)
if [[ -n "$conflict_nodes" ]] || grep -A20 '^Publishers:' <<<"$setpoint_info" | grep -qE '^ \* /(uav_control_bridge|.*offboard)'; then
  echo "ERROR: ACFLY_MODE_CONFLICT detected. Cage mission start will remain blocked."
  printf '%s\n' "$conflict_nodes"
fi

if [[ "$check_only" == true ]]; then
  echo "=== CHECK-ONLY LIVE STATUS ==="
  live_errors=0
  topics=(/mavros/state /mission/state /feeding/task_status)
  if [[ "$input_profile" == ground_gps ]]; then
    topics+=(/farm/scene_1/anchor_A/fix /farm/scene_1/anchor_B/fix /cage/geometry)
  fi
  [[ "$feeding" == false ]] || topics+=(/feeder/status)
  for topic in "${topics[@]}"; do
    if timeout 2 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then echo "OK    $topic"; else echo "MISS  $topic"; live_errors=$((live_errors+1)); fi
  done
  echo "input_profile=$input_profile_ros geometry=$geometry feeding=$feeding web=$start_web rviz=$start_rviz"
  ((errors == 0 && live_errors == 0)) || exit 1
  exit 0
fi
((errors == 0)) || { echo "Static precheck failed; stack was not started." >&2; exit 1; }

mkdir -p "$state_dir" "$log_dir"
printf '%s\n' "$input_profile_ros" > "$profile_file"
stack_running=false
if [[ -s "$pid_file" ]]; then
  while read -r pid _; do
    if kill -0 "$pid" 2>/dev/null; then stack_running=true; break; fi
  done < "$pid_file"
fi
if [[ "$stack_running" == true ]]; then
  echo "A stack started by this script is already running. Use stop_realflight_stack.sh first." >&2
  exit 1
fi
: > "$pid_file"

cleanup_on_error() {
  local code=$?
  ((code != 0)) || return 0
  trap - EXIT
  echo "Startup failed; stopping only process groups created by this run." >&2
  while read -r pid _; do
    [[ "$pid" =~ ^[0-9]+$ ]] && kill -TERM -- "-$pid" 2>/dev/null || true
  done < "$pid_file"
  exit "$code"
}
trap cleanup_on_error EXIT

start_bg() {
  local label=$1 command=$2
  : >"$log_dir/$label.log"
  setsid bash -lc "$command" >>"$log_dir/$label.log" 2>&1 &
  local pid=$!
  printf '%s %s\n' "$pid" "$label" >> "$pid_file"
  echo "Started $label (pid $pid, log $log_dir/$label.log)"
}
wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  until timeout 3 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
    ((SECONDS < deadline)) || { echo "Timed out waiting for $topic" >&2; return 1; }
    sleep 1
  done
  echo "Ready: $topic"
}
wait_node() {
  local node="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  until rosnode list 2>/dev/null | grep -qx "$node"; do
    ((SECONDS < deadline)) || { echo "Timed out waiting for node $node" >&2; return 1; }
    sleep 1
  done
  echo "Ready: $node"
}
wait_mavros_connected() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  local connected
  while ((SECONDS < deadline)); do
    connected=$(timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null |
      awk '/^connected:/ {print tolower($2); exit}' || true)
    if [[ "$connected" == true ]]; then
      echo "Ready: MAVROS connected to FCU"
      return 0
    fi
    sleep 1
  done
  echo "MAVROS is running but FCU did not connect within ${timeout_sec}s" >&2
  return 1
}

if [[ "$input_profile" == ground_gps ]]; then
  if ! rosnode list 2>/dev/null | grep -qx /gps_receiver; then
    start_bg gps "source /opt/ros/noetic/setup.bash; source /home/nx3/gps_ws/devel/setup.bash; exec roslaunch gps_receiver gps_receiver.launch serial_port:=$gps_port baudrate:=115200"
  fi
  wait_topic /farm/scene_1/anchor_A/fix 90
  wait_topic /farm/scene_1/anchor_B/fix 90
else
  echo "Skipped ground GPS receiver and A/B topics (PHOTO_COORDS)"
  /home/nx3/cage_uav_ws/src/cage_site_db/scripts/start_site_db.sh
  curl -fsS http://127.0.0.1:8090/api/site-db/health >/dev/null
  echo "Ready: cage site coordinate database"
fi

if ! rosnode list 2>/dev/null | grep -qx /mavros; then
  start_bg mavros "source /opt/ros/noetic/setup.bash; source /home/nx3/acfly_ws/devel/setup.bash; exec roslaunch mavros acfly.launch fcu_url:=$fcu_url fcu_protocol:=v2.0"
fi
wait_topic /mavros/state 45
wait_mavros_connected 30
if [[ "$feeding" == true ]]; then
  aux9_func=$(rosservice call /mavros/param/get "{param_id: 'Aux_9Func'}" 2>/dev/null | awk '/integer:/ {print $2; exit}')
  [[ "$aux9_func" == 0 ]] || {
    echo "PWM9 is not free: Aux_9Func=${aux9_func:-UNKNOWN}. Set it to 0 before real feeding." >&2
    exit 1
  }
  echo "Ready: Aux_9Func=0 (PWM9 direct control)"
fi

conflict_nodes=$(rosnode list 2>/dev/null | grep -Ei 'uav_control_bridge|offboard' || true)
[[ -z "$conflict_nodes" ]] || echo "WARNING: ACFLY_MODE_CONFLICT; Cage will start for diagnosis but mission start is blocked."

cage_source="source /opt/ros/noetic/setup.bash; source /home/nx3/acfly_ws/devel/setup.bash; source /home/nx3/cage_uav_ws/devel/setup.bash"
if [[ "$input_profile" == ground_gps ]]; then
  cage_source="source /opt/ros/noetic/setup.bash; source /home/nx3/gps_ws/devel/setup.bash; source /home/nx3/acfly_ws/devel/setup.bash; source /home/nx3/cage_uav_ws/devel/setup.bash; export PYTHONPATH=/home/nx3/gps_ws/devel/lib/python3/dist-packages:\${PYTHONPATH:-}; export LD_LIBRARY_PATH=/home/nx3/gps_ws/devel/lib:\${LD_LIBRARY_PATH:-}"
fi
start_bg cage "$cage_source; exec roslaunch cage_realflight realflight.launch input_profile:=$input_profile_ros geometry_mode:=$geometry enable_feeding:=$feeding feeder_mode:=$feeder_mode start_mavros:=false start_web:=$start_web web_port:=$web_port start_rviz:=$start_rviz"
wait_topic /mission/state 45
wait_topic /cage/realflight/precheck 45
[[ "$feeding" == false ]] || wait_topic /feeder/status 30
[[ "$start_web" == false ]] || wait_node /cage_web_bridge 30
[[ "$start_rviz" == false ]] || wait_node /cage_uav_rviz 30

ip=$(hostname -I | awk '{print $1}')
fcmode=$(timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null |
  awk -F': ' '/^mode:/ {gsub(/"/, "", $2); print $2; exit}' || true)
ready=$(rosparam get /realflight/start_ready 2>/dev/null || echo false)
confirmed=$(rosparam get /realflight/site_alignment_confirmed 2>/dev/null || echo false)
echo "=============================="
echo "REALFLIGHT STACK STATUS"
echo "=============================="
echo "Input profile    $input_profile_ros"
[[ "$input_profile" == ground_gps ]] && echo "GPS A/B          OK" || echo "GPS A/B          SKIPPED"
echo "MAVROS/FCU       OK"
echo "Cage FSM         OK"
[[ "$start_web" == true ]] && echo "Web              OK"
[[ "$start_rviz" == true ]] && echo "RViz             OK"
echo "Geometry mode    $geometry"
echo "Feeding          $([[ $feeding == true ]] && echo ENABLED || echo DISABLED)"
echo "FC mode          ${fcmode:-UNKNOWN}"
echo "Start ready      $ready"
echo "Site confirmed   $confirmed"
[[ "$start_web" == true ]] && echo "Web URL          http://$ip:$web_port"
echo "Logs             $log_dir"
echo "=============================="
trap - ERR
