#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_localization.yaml"
DEFAULT_RVIZ_CONFIG="$ROOT_DIR/super_odometry/ros2_minimal.rviz"
FAST_LIO2_LOC_ROOT=${FAST_LIO2_LOC_ROOT:-"$HOME/workspace/fast_lio2_loc"}

SKIP_BUILD=0
ENABLE_RVIZ=1
ENABLE_MONITOR=1
WAIT_FOR_INITIAL_POSE=1
RECORD_OUTPUT=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--skip-build] [--no-rviz] [--no-monitor] [--no-wait-initial-pose] [--record-output] <bag_dir> <prior_map_pcd> [run_name]

Examples:
  $(basename "$0") data_bag/K-rail1 /abs/path/map.pcd
  $(basename "$0") --record-output data_bag/K-rail1 /abs/path/map.pcd robocup_localization

Environment variables:
  CONFIG_FILE               Override localization config. Default: $DEFAULT_CONFIG
  RVIZ_CONFIG_FILE          Override RViz config. Default: $DEFAULT_RVIZ_CONFIG
  OUTPUT_BASE_DIR           Override output root. Default: $ROOT_DIR/loc_output
  EXTRA_LAUNCH_ARGS         Extra args appended to ros2 launch.
  EXTRA_PLAY_ARGS           Extra args appended to ros2 bag play.
  PLAY_RATE                 ros2 bag play rate. Default: 1.0
  STARTUP_DELAY_SEC         Delay before initial-pose waiting. Default: 5
  INITIAL_POSE_TIMEOUT_SEC  0 means wait forever. Default: 0
  KEEP_RVIZ_OPEN            Keep RViz open after playback until user closes it. Default: 1
  MONITOR_PERIOD            Minimum time between stats prints. Default: 0.5
EOF
}

SCRIPT_FLAGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --no-rviz)
      ENABLE_RVIZ=0
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --no-monitor)
      ENABLE_MONITOR=0
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --no-wait-initial-pose)
      WAIT_FOR_INITIAL_POSE=0
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --record-output)
      RECORD_OUTPUT=1
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

if [[ $# -lt 2 ]]; then
  usage
  exit 1
fi

BAG_PATH=$1
PRIOR_MAP_PCD=$2
RUN_NAME=${3:-"$(basename "$BAG_PATH")_localization_$(date +%Y%m%d_%H%M%S)"}
CONFIG_FILE=${CONFIG_FILE:-"$DEFAULT_CONFIG"}
RVIZ_CONFIG_FILE=${RVIZ_CONFIG_FILE:-"$DEFAULT_RVIZ_CONFIG"}
OUTPUT_BASE_DIR=${OUTPUT_BASE_DIR:-"$ROOT_DIR/loc_output"}
OUTPUT_DIR="$OUTPUT_BASE_DIR/$RUN_NAME"
ROS_HOME_DIR="$OUTPUT_DIR/ros_home"
ROS_LOG_DIR_VALUE="$OUTPUT_DIR/ros_log"
EXTRA_LAUNCH_ARGS=${EXTRA_LAUNCH_ARGS:-}
EXTRA_PLAY_ARGS=${EXTRA_PLAY_ARGS:-}
PLAY_RATE=${PLAY_RATE:-1.0}
STARTUP_DELAY_SEC=${STARTUP_DELAY_SEC:-5}
INITIAL_POSE_TIMEOUT_SEC=${INITIAL_POSE_TIMEOUT_SEC:-0}
KEEP_RVIZ_OPEN=${KEEP_RVIZ_OPEN:-1}
MONITOR_PERIOD=${MONITOR_PERIOD:-0.5}
PRIOR_MAP_DIR=$(dirname "$PRIOR_MAP_PCD")
PRIOR_MAP_STEM=$(basename "$PRIOR_MAP_PCD")
PRIOR_MAP_STEM=${PRIOR_MAP_STEM%.*}
INITIAL_POSE_FILE="$PRIOR_MAP_DIR/$PRIOR_MAP_STEM.start_pose.txt"

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Bag directory not found: $BAG_PATH" >&2
  exit 1
fi
if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "ROS2 bag metadata.yaml not found in: $BAG_PATH" >&2
  exit 1
fi
if [[ ! -f "$PRIOR_MAP_PCD" ]]; then
  echo "Prior map not found: $PRIOR_MAP_PCD" >&2
  exit 1
fi
if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Config file not found: $CONFIG_FILE" >&2
  exit 1
fi
if [[ $ENABLE_RVIZ -eq 1 && ! -f "$RVIZ_CONFIG_FILE" ]]; then
  echo "RViz config file not found: $RVIZ_CONFIG_FILE" >&2
  exit 1
fi
if [[ $ENABLE_RVIZ -eq 1 && -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set, disabling RViz for this run." >&2
  ENABLE_RVIZ=0
fi

read_config_bool() {
  local key=$1
  python3 - "$CONFIG_FILE" "$key" <<'PY'
import sys, yaml
path, key = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
params = data.get("/**", {}).get("ros__parameters", {})
node = params.get("laser_mapping_node", {})
value = node.get(key, False)
print("true" if bool(value) else "false")
PY
}

USE_RVIZ_INITIAL_POSE_CFG=$(read_config_bool use_rviz_initial_pose)
RVIZ_XY_YAW_ONLY_CFG=$(read_config_bool rviz_initial_pose_xy_yaw_only)

mkdir -p "$OUTPUT_DIR" "$ROS_HOME_DIR" "$ROS_LOG_DIR_VALUE"

quoted_flags=()
for flag in "${SCRIPT_FLAGS[@]}"; do
  printf -v quoted_flag '%q' "$flag"
  quoted_flags+=("$quoted_flag")
done
printf -v quoted_bag '%q' "$BAG_PATH"
printf -v quoted_map '%q' "$PRIOR_MAP_PCD"
printf -v quoted_run '%q' "$RUN_NAME"

cat >"$OUTPUT_DIR/run_metadata.txt" <<EOF
timestamp=$(date -Iseconds)
root_dir=$ROOT_DIR
bag_path=$BAG_PATH
prior_map_pcd=$PRIOR_MAP_PCD
initial_pose_file=$INITIAL_POSE_FILE
run_name=$RUN_NAME
config_file=$CONFIG_FILE
rviz_config_file=$RVIZ_CONFIG_FILE
skip_build=$SKIP_BUILD
enable_rviz=$ENABLE_RVIZ
enable_monitor=$ENABLE_MONITOR
wait_for_initial_pose=$WAIT_FOR_INITIAL_POSE
record_output=$RECORD_OUTPUT
play_rate=$PLAY_RATE
startup_delay_sec=$STARTUP_DELAY_SEC
initial_pose_timeout_sec=$INITIAL_POSE_TIMEOUT_SEC
keep_rviz_open=$KEEP_RVIZ_OPEN
monitor_period=$MONITOR_PERIOD
extra_launch_args=$EXTRA_LAUNCH_ARGS
extra_play_args=$EXTRA_PLAY_ARGS
EOF

cat >"$OUTPUT_DIR/rerun_command.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "$ROOT_DIR"
bash script/run_superodom_livox_localization.sh ${quoted_flags[*]} $quoted_bag $quoted_map $quoted_run
EOF
chmod +x "$OUTPUT_DIR/rerun_command.sh"

if [[ $SKIP_BUILD -eq 0 ]]; then
  "$ROOT_DIR/script/build_superodom_local.sh"
fi

set +u
source /opt/ros/humble/setup.bash
source "$FAST_LIO2_LOC_ROOT/install/setup.bash"
source "$ROOT_DIR/install/setup.bash"
set -u

export ROS_HOME="$ROS_HOME_DIR"
export ROS_LOG_DIR="$ROS_LOG_DIR_VALUE"
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-1}

ros2 bag info "$BAG_PATH" >"$OUTPUT_DIR/input_bag_info.txt"
cp "$CONFIG_FILE" "$OUTPUT_DIR/used_config.yaml"
if [[ -f "$RVIZ_CONFIG_FILE" ]]; then
  cp "$RVIZ_CONFIG_FILE" "$OUTPUT_DIR/used_rviz_config.rviz"
fi

launch_log="$OUTPUT_DIR/launch.log"
bag_log="$OUTPUT_DIR/bag_play.log"
rviz_log="$OUTPUT_DIR/rviz.log"
monitor_log="$OUTPUT_DIR/monitor.log"
wait_pose_log="$OUTPUT_DIR/wait_initial_pose.log"
record_log="$OUTPUT_DIR/output_record.log"
record_dir="$OUTPUT_DIR/output_topics"

launch_pid=""
bag_pid=""
rviz_pid=""
monitor_pid=""
wait_pose_pid=""
record_pid=""
STARTED_PID=""

start_group() {
  local log_file=$1
  shift
  setsid "$@" >"$log_file" 2>&1 &
  STARTED_PID=$!
}

stop_group() {
  local pid=$1
  local signal_name=${2:-INT}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal_name" -- "-$pid" 2>/dev/null || kill "-$signal_name" "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  stop_group "$bag_pid" INT
  stop_group "$record_pid" INT
  stop_group "$wait_pose_pid" TERM
  stop_group "$rviz_pid" TERM
  stop_group "$monitor_pid" TERM
  stop_group "$launch_pid" INT
}

trap cleanup EXIT INT TERM

launch_cmd=(
  ros2 launch super_odometry livox_mid360.launch.py
  "config_file:=$CONFIG_FILE"
  "map_dir:=$PRIOR_MAP_PCD"
)

if [[ -n "$EXTRA_LAUNCH_ARGS" ]]; then
  # shellcheck disable=SC2206
  extra_launch_array=($EXTRA_LAUNCH_ARGS)
  launch_cmd+=("${extra_launch_array[@]}")
fi

bag_cmd=(
  ros2 bag play
  --rate "$PLAY_RATE"
)

if [[ -n "$EXTRA_PLAY_ARGS" ]]; then
  # shellcheck disable=SC2206
  extra_play_array=($EXTRA_PLAY_ARGS)
  bag_cmd+=("${extra_play_array[@]}")
fi
bag_cmd+=("$BAG_PATH")

if [[ $RECORD_OUTPUT -eq 1 ]]; then
  record_cmd=(
    ros2 bag record
    -o "$record_dir"
    /laser_odometry
    /state_estimation
    /laser_odom_path
    /imuodom_path
    /super_odometry_stats
  )
fi

normalize_status() {
  local status=$1
  case "$status" in
    0|130|143)
      echo 0
      ;;
    *)
      echo "$status"
      ;;
  esac
}

set +e
start_group "$launch_log" "${launch_cmd[@]}"
launch_pid=$STARTED_PID
sleep "$STARTUP_DELAY_SEC"

if [[ $ENABLE_MONITOR -eq 1 ]]; then
  setsid python3 "$ROOT_DIR/script/monitor_superodom_stats_live.py" \
    --topic /super_odometry_stats \
    --min-period "$MONITOR_PERIOD" \
    --log-file "$monitor_log" &
  monitor_pid=$!
fi

if [[ $RECORD_OUTPUT -eq 1 ]]; then
  start_group "$record_log" "${record_cmd[@]}"
  record_pid=$STARTED_PID
fi

if [[ $ENABLE_RVIZ -eq 1 ]]; then
  start_group "$rviz_log" rviz2 -d "$RVIZ_CONFIG_FILE"
  rviz_pid=$STARTED_PID
fi

echo "Localization session started."
echo "  output_dir: $OUTPUT_DIR"
echo "  bag_path:   $BAG_PATH"
echo "  prior_map:  $PRIOR_MAP_PCD"
echo "  init_pose:  $INITIAL_POSE_FILE"
echo "  config:     $CONFIG_FILE"
if [[ $ENABLE_RVIZ -eq 1 ]]; then
  echo "  rviz:       $RVIZ_CONFIG_FILE"
fi
if [[ "$USE_RVIZ_INITIAL_POSE_CFG" == "true" ]]; then
  if [[ "$RVIZ_XY_YAW_ONLY_CFG" == "true" ]]; then
    echo "  rviz_init:  x/y/yaw only; z/roll/pitch are preserved from config"
  else
    echo "  rviz_init:  full pose from /initialpose"
  fi
else
  echo "  rviz_init:  disabled in config; fixed init pose / pose file will be used"
fi

if [[ $WAIT_FOR_INITIAL_POSE -eq 1 ]]; then
  echo "Use RViz '2D Pose Estimate' to publish /initialpose, then bag playback will start."
  setsid python3 "$ROOT_DIR/script/wait_for_initial_pose.py" \
    --topic /initialpose \
    --timeout-sec "$INITIAL_POSE_TIMEOUT_SEC" >"$wait_pose_log" 2>&1 &
  wait_pose_pid=$!
  wait "$wait_pose_pid"
  wait_pose_status=$?
  wait_pose_pid=""
  if [[ $wait_pose_status -ne 0 ]]; then
    echo "Failed while waiting for /initialpose. See $wait_pose_log" >&2
    exit "$wait_pose_status"
  fi
  if [[ -f "$wait_pose_log" ]]; then
    tail -n 1 "$wait_pose_log"
  fi
  echo "Initial pose accepted. Starting localization playback now."
  sleep 2
else
  wait_pose_status=0
fi

start_group "$bag_log" "${bag_cmd[@]}"
bag_pid=$STARTED_PID

wait "$bag_pid"
bag_status=$?
bag_pid=""

if [[ $KEEP_RVIZ_OPEN -eq 1 && -n "$rviz_pid" ]] && kill -0 "$rviz_pid" 2>/dev/null; then
  echo "Bag playback finished. Close RViz to end the session, or press Ctrl-C."
  wait "$rviz_pid"
  rviz_status=$?
  rviz_pid=""
else
  rviz_status=0
fi

cleanup

wait "$launch_pid"
launch_status=$?
launch_pid=""

if [[ -n "$monitor_pid" ]]; then
  wait "$monitor_pid"
  monitor_status=$?
  monitor_pid=""
else
  monitor_status=0
fi

if [[ -n "$record_pid" ]]; then
  wait "$record_pid"
  record_status=$?
  record_pid=""
else
  record_status=0
fi
set -e

launch_status=$(normalize_status "$launch_status")
monitor_status=$(normalize_status "$monitor_status")
record_status=$(normalize_status "$record_status")
rviz_status=$(normalize_status "$rviz_status")
wait_pose_status=$(normalize_status "${wait_pose_status:-0}")

printf 'bag_status=%s\nlaunch_status=%s\nrviz_status=%s\nmonitor_status=%s\nrecord_status=%s\nwait_initial_pose_status=%s\n' \
  "$bag_status" "$launch_status" "$rviz_status" "$monitor_status" "$record_status" "$wait_pose_status" \
  >"$OUTPUT_DIR/run_status.txt"

if [[ -d "$record_dir" && -f "$record_dir/metadata.yaml" ]]; then
  ros2 bag info "$record_dir" >"$OUTPUT_DIR/output_bag_info.txt"
fi

echo "Localization session complete. Outputs written to: $OUTPUT_DIR"
