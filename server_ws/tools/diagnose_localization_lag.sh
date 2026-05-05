#!/usr/bin/env bash
set -u

DURATION="${1:-8}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT="${2:-$WS_DIR/localization_lag_report_$(date +%Y%m%d_%H%M%S).txt}"

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # colcon's setup.bash references COLCON_TRACE without guarding against set -u
  set +u
  # shellcheck disable=SC1091
  source "$WS_DIR/install/setup.bash"
  set -u
else
  echo "WARN: $WS_DIR/install/setup.bash was not found. Run from a built workspace or source ROS manually." >&2
fi

mkdir -p "$(dirname "$OUT")"
: > "$OUT"

section() {
  local title="$1"
  shift
  {
    echo
    echo "================================================================================"
    echo "$title"
    echo "================================================================================"
    echo "COMMAND: $*"
  } | tee -a "$OUT"

  "$@" >> "$OUT" 2>&1
  local status=$?
  echo "EXIT_STATUS: $status" | tee -a "$OUT"
}

section_timeout() {
  local seconds="$1"
  local title="$2"
  shift 2
  {
    echo
    echo "================================================================================"
    echo "$title"
    echo "================================================================================"
    echo "COMMAND: timeout -s INT -k 2s ${seconds}s $*"
  } | tee -a "$OUT"

  timeout -s INT -k 2s "${seconds}s" "$@" >> "$OUT" 2>&1
  local status=$?
  if [[ "$status" -eq 124 ]]; then
    echo "EXIT_STATUS: 124 (timeout reached; this is normal for streaming ROS commands)" | tee -a "$OUT"
  else
    echo "EXIT_STATUS: $status" | tee -a "$OUT"
  fi
}

topic_hz() {
  local topic="$1"
  section_timeout "$DURATION" "topic hz $topic" ros2 topic hz "$topic" --qos-profile sensor_data
}

topic_delay() {
  local topic="$1"
  section_timeout "$DURATION" "topic delay $topic" ros2 topic delay "$topic" --qos-profile sensor_data
}

topic_echo_once() {
  local topic="$1"
  section_timeout 4 "topic echo --once $topic" ros2 topic echo "$topic" --once --qos-profile sensor_data
}

tf_monitor() {
  local source="$1"
  local target="$2"
  section_timeout "$DURATION" "tf2_monitor $source $target" ros2 run tf2_ros tf2_monitor "$source" "$target"
}

tf_echo() {
  local source="$1"
  local target="$2"
  section_timeout 4 "tf2_echo $source $target" ros2 run tf2_ros tf2_echo "$source" "$target"
}

{
  echo "Localization lag diagnostic report"
  echo "Generated: $(date --iso-8601=seconds)"
  echo "Workspace: $WS_DIR"
  echo "Duration per streaming check: ${DURATION}s"
  echo "ROS_DISTRO: ${ROS_DISTRO:-unset}"
  echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-unset}"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-unset}"
  echo
  echo "Before sending this report, also note RViz Fixed Frame manually: map / odom / base_link"
} | tee -a "$OUT"

section "ros2 node list" ros2 node list
section "ros2 topic list" ros2 topic list

for topic in \
  /deadwheel_ticks \
  /odom_deadwheels \
  /odometry/local \
  /camera/global_pose \
  /odometry/global \
  /tf \
  /tf_static
  do
  section_timeout 3 "topic info -v $topic" ros2 topic info -v "$topic"
done

for topic in \
  /deadwheel_ticks \
  /odom_deadwheels \
  /odometry/local \
  /camera/global_pose \
  /odometry/global
  do
  topic_hz "$topic"
done

for topic in \
  /deadwheel_ticks \
  /odom_deadwheels \
  /odometry/local \
  /camera/global_pose \
  /odometry/global
  do
  topic_delay "$topic"
done

for topic in \
  /deadwheel_ticks \
  /odom_deadwheels \
  /odometry/local \
  /camera/global_pose \
  /odometry/global
  do
  topic_echo_once "$topic"
done

tf_monitor odom base_link
tf_monitor map odom
tf_monitor map base_link

tf_echo odom base_link
tf_echo map odom
tf_echo map base_link

section_timeout 3 "EKF local parameters" ros2 param dump /ekf_local_node
section_timeout 3 "EKF global parameters" ros2 param dump /ekf_global_node
section_timeout 3 "camera global localization parameters" ros2 param dump /global_localization_node

echo | tee -a "$OUT"
echo "Report written to: $OUT" | tee -a "$OUT"
