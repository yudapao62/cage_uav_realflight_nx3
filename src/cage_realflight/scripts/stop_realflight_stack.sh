#!/usr/bin/env bash
# ROS setup scripts read optional environment variables that may be unset.
# Load ROS before enabling nounset so web-supervisor initiated stops work in
# its minimal process environment.
source /opt/ros/noetic/setup.bash
set -Eeuo pipefail
state_dir=/home/nx3/.cage_realflight_stack
pid_file=$state_dir/pids
profile_file=$state_dir/input_profile

echo "WARNING: this stops ROS software only. If airborne, use H12/FCU to take over, RTL or land safely first."
if [[ -s "$pid_file" ]]; then
  while read -r pid label; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $label (process group $pid)"
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done < "$pid_file"
  sleep 3
  while read -r pid label; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      echo "Force stopping $label (process group $pid)"
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done < "$pid_file"
  : > "$pid_file"
else
  echo "No processes recorded by start_realflight_stack.sh; nothing was stopped."
fi
rosparam set /realflight/site_alignment_confirmed false 2>/dev/null || true
rosparam set /realflight/start_ready false 2>/dev/null || true
rosparam set /realflight/resume_ready false 2>/dev/null || true
rm -f "$profile_file"
/home/nx3/cage_uav_ws/src/cage_site_db/scripts/stop_site_db.sh 2>/dev/null || true
echo "Realflight site confirmation and readiness were cleared."
