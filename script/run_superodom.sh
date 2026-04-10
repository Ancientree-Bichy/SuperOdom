#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=script/common_env.sh
source "$ROOT_DIR/script/common_env.sh"

DEPENDENCY_WS_ROOT=$(resolve_dependency_ws_root)
PACKAGE_PLY_PATH="$ROOT_DIR/super_odometry/PLY/saved_scans.ply"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") mapping-bag [--skip-build] [--record-output|--no-record-output] <bag_dir> [run_name]
  $(basename "$0") mapping-live [--skip-build] [--no-rviz] [--no-monitor] [--record-output] [run_name]
  $(basename "$0") localization-bag [--skip-build] [--no-rviz] [--no-monitor] [--no-wait-initial-pose] [--record-output] <bag_dir> <prior_map_pcd> [run_name]
  $(basename "$0") localization-live [--skip-build] [--no-rviz] [--no-monitor] [--no-wait-initial-pose] [--record-output] <prior_map_pcd> [run_name]
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

MODE=$1
shift

case "$MODE" in
  mapping-bag)
    DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_local_bag.yaml"
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/mapping_output"
    ENABLE_RVIZ=0
    ENABLE_MONITOR=0
    WAIT_FOR_INITIAL_POSE=0
    RECORD_OUTPUT=1
    MODE_REQUIRES_BAG=1
    MODE_REQUIRES_MAP=0
    MODE_IS_LIVE=0
    MODE_COPY_PLY=1
    ;;
  mapping-live)
    DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_local_bag.yaml"
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/mapping_output_live"
    ENABLE_RVIZ=1
    ENABLE_MONITOR=1
    WAIT_FOR_INITIAL_POSE=0
    RECORD_OUTPUT=0
    MODE_REQUIRES_BAG=0
    MODE_REQUIRES_MAP=0
    MODE_IS_LIVE=1
    MODE_COPY_PLY=0
    ;;
  localization-bag)
    DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_localization.yaml"
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/loc_output"
    ENABLE_RVIZ=1
    ENABLE_MONITOR=1
    WAIT_FOR_INITIAL_POSE=1
    RECORD_OUTPUT=0
    MODE_REQUIRES_BAG=1
    MODE_REQUIRES_MAP=1
    MODE_IS_LIVE=0
    MODE_COPY_PLY=0
    ;;
  localization-live)
    DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_localization.yaml"
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/loc_output"
    ENABLE_RVIZ=1
    ENABLE_MONITOR=1
    WAIT_FOR_INITIAL_POSE=1
    RECORD_OUTPUT=0
    MODE_REQUIRES_BAG=0
    MODE_REQUIRES_MAP=1
    MODE_IS_LIVE=1
    MODE_COPY_PLY=0
    ;;
  --help|-h|help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown mode: $MODE" >&2
    usage
    exit 1
    ;;
esac

SKIP_BUILD=0
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
    --no-record-output)
      RECORD_OUTPUT=0
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
    *)
      break
      ;;
  esac
done

BAG_PATH=""
PRIOR_MAP_PCD=""
RUN_NAME=""

if [[ $MODE_REQUIRES_BAG -eq 1 && $MODE_REQUIRES_MAP -eq 1 ]]; then
  if [[ $# -lt 2 ]]; then
    usage
    exit 1
  fi
  BAG_PATH=$1
  PRIOR_MAP_PCD=$2
  RUN_NAME=${3:-"$(basename "$BAG_PATH")_localization_$(date +%Y%m%d_%H%M%S)"}
elif [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  if [[ $# -lt 1 ]]; then
    usage
    exit 1
  fi
  BAG_PATH=$1
  if [[ "$MODE" == "mapping-bag" ]]; then
    RUN_NAME=${2:-"$(basename "$BAG_PATH")_$(date +%Y%m%d_%H%M%S)"}
  else
    RUN_NAME=${2:-"${MODE}_$(date +%Y%m%d_%H%M%S)"}
  fi
elif [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  if [[ $# -lt 1 ]]; then
    usage
    exit 1
  fi
  PRIOR_MAP_PCD=$1
  RUN_NAME=${2:-"$(basename "$PRIOR_MAP_PCD" .pcd)_${MODE}_$(date +%Y%m%d_%H%M%S)"}
else
  RUN_NAME=${1:-"${MODE}_$(date +%Y%m%d_%H%M%S)"}
fi

CONFIG_FILE=${CONFIG_FILE:-"$DEFAULT_CONFIG"}
RVIZ_CONFIG_FILE=${RVIZ_CONFIG_FILE:-"$ROOT_DIR/super_odometry/ros2_minimal.rviz"}
OUTPUT_BASE_DIR=${OUTPUT_BASE_DIR:-"$DEFAULT_OUTPUT_BASE_DIR"}
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

INITIAL_POSE_FILE=""
if [[ -n "$PRIOR_MAP_PCD" ]]; then
  PRIOR_MAP_DIR=$(dirname "$PRIOR_MAP_PCD")
  PRIOR_MAP_STEM=$(basename "$PRIOR_MAP_PCD")
  PRIOR_MAP_STEM=${PRIOR_MAP_STEM%.*}
  INITIAL_POSE_FILE="$PRIOR_MAP_DIR/$PRIOR_MAP_STEM.start_pose.txt"
fi

if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  if [[ ! -d "$BAG_PATH" ]]; then
    echo "Bag directory not found: $BAG_PATH" >&2
    exit 1
  fi
  if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
    echo "ROS2 bag metadata.yaml not found in: $BAG_PATH" >&2
    exit 1
  fi
fi

if [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  if [[ ! -f "$PRIOR_MAP_PCD" ]]; then
    echo "Prior map not found: $PRIOR_MAP_PCD" >&2
    exit 1
  fi
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

read_config_value() {
  local path_expr=$1
  python3 - "$CONFIG_FILE" "$path_expr" <<'PY'
import sys, yaml
path, expr = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
params = data.get("/**", {}).get("ros__parameters", {})
current = params
for part in expr.split("."):
    if isinstance(current, dict):
        current = current.get(part)
    else:
        current = None
        break
if isinstance(current, bool):
    print("true" if current else "false")
elif current is None:
    print("")
else:
    print(current)
PY
}

USE_RVIZ_INITIAL_POSE_CFG=$(read_config_value laser_mapping_node.use_rviz_initial_pose)
RVIZ_XY_YAW_ONLY_CFG=$(read_config_value laser_mapping_node.rviz_initial_pose_xy_yaw_only)
LASER_TOPIC_CFG=$(read_config_value laser_topic)
IMU_TOPIC_CFG=$(read_config_value imu_topic)

mkdir -p "$OUTPUT_DIR" "$ROS_HOME_DIR" "$ROS_LOG_DIR_VALUE"

quoted_cli_args=()
for item in "${SCRIPT_FLAGS[@]}"; do
  printf -v q '%q' "$item"
  quoted_cli_args+=("$q")
done
if [[ -n "$BAG_PATH" ]]; then
  printf -v q '%q' "$BAG_PATH"
  quoted_cli_args+=("$q")
fi
if [[ -n "$PRIOR_MAP_PCD" ]]; then
  printf -v q '%q' "$PRIOR_MAP_PCD"
  quoted_cli_args+=("$q")
fi
if [[ -n "$RUN_NAME" ]]; then
  printf -v q '%q' "$RUN_NAME"
  quoted_cli_args+=("$q")
fi

cat >"$OUTPUT_DIR/run_metadata.txt" <<EOF
timestamp=$(date -Iseconds)
mode=$MODE
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
laser_topic=$LASER_TOPIC_CFG
imu_topic=$IMU_TOPIC_CFG
EOF

printf -v quoted_mode '%q' "$MODE"
cat >"$OUTPUT_DIR/rerun_command.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "$ROOT_DIR"
bash script/run_superodom.sh $quoted_mode ${quoted_cli_args[*]}
EOF
chmod +x "$OUTPUT_DIR/rerun_command.sh"

if [[ $SKIP_BUILD -eq 0 ]]; then
  "$ROOT_DIR/script/build_superodom_local.sh"
fi

source_ros_runtime_env "$ROOT_DIR" "$DEPENDENCY_WS_ROOT"

export ROS_HOME="$ROS_HOME_DIR"
export ROS_LOG_DIR="$ROS_LOG_DIR_VALUE"
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-1}

if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  ros2 bag info "$BAG_PATH" >"$OUTPUT_DIR/input_bag_info.txt"
fi
cp "$CONFIG_FILE" "$OUTPUT_DIR/used_config.yaml"
if [[ -f "$RVIZ_CONFIG_FILE" ]]; then
  cp "$RVIZ_CONFIG_FILE" "$OUTPUT_DIR/used_rviz_config.rviz"
fi

run_marker="$OUTPUT_DIR/run_started.marker"
touch "$run_marker"

duration_ceiling=0
if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  duration_raw=$(sed -n 's/^Duration:[[:space:]]*\([0-9.]*\)s/\1/p' "$OUTPUT_DIR/input_bag_info.txt")
  if [[ -z "$duration_raw" ]]; then
    duration_raw=60
  fi
  duration_ceiling=$(awk -v value="$duration_raw" 'BEGIN { print int(value + 0.999999) }')
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
)
if [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  launch_cmd+=("map_dir:=$PRIOR_MAP_PCD")
fi
if [[ -n "$EXTRA_LAUNCH_ARGS" ]]; then
  # shellcheck disable=SC2206
  extra_launch_array=($EXTRA_LAUNCH_ARGS)
  launch_cmd+=("${extra_launch_array[@]}")
fi

if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
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
fi

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

set +e
if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  launch_timeout=$((duration_ceiling + 25))
  start_group "$launch_log" timeout "${launch_timeout}s" "${launch_cmd[@]}"
else
  start_group "$launch_log" "${launch_cmd[@]}"
fi
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
  if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
    record_timeout=$((duration_ceiling + 35))
    start_group "$record_log" timeout "${record_timeout}s" "${record_cmd[@]}"
  else
    start_group "$record_log" "${record_cmd[@]}"
  fi
  record_pid=$STARTED_PID
fi

if [[ $ENABLE_RVIZ -eq 1 ]]; then
  start_group "$rviz_log" rviz2 -d "$RVIZ_CONFIG_FILE"
  rviz_pid=$STARTED_PID
fi

echo "SuperOdom session started."
echo "  mode:       $MODE"
echo "  output_dir: $OUTPUT_DIR"
if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  echo "  bag_path:   $BAG_PATH"
fi
if [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  echo "  prior_map:  $PRIOR_MAP_PCD"
  echo "  init_pose:  $INITIAL_POSE_FILE"
fi
echo "  config:     $CONFIG_FILE"
if [[ $ENABLE_RVIZ -eq 1 ]]; then
  echo "  rviz:       $RVIZ_CONFIG_FILE"
fi
if [[ $MODE_IS_LIVE -eq 1 ]]; then
  echo "  required_input_topics:"
  echo "    lidar: $LASER_TOPIC_CFG"
  echo "    imu:   $IMU_TOPIC_CFG"
fi
if [[ $MODE_REQUIRES_MAP -eq 1 && "$USE_RVIZ_INITIAL_POSE_CFG" == "true" ]]; then
  if [[ "$RVIZ_XY_YAW_ONLY_CFG" == "true" ]]; then
    echo "  rviz_init:  x/y/yaw only; z/roll/pitch are preserved from config"
  else
    echo "  rviz_init:  full pose from /initialpose"
  fi
fi

if [[ $MODE_REQUIRES_MAP -eq 1 && $WAIT_FOR_INITIAL_POSE -eq 1 ]]; then
  if [[ $MODE_IS_LIVE -eq 1 ]]; then
    echo "Use RViz '2D Pose Estimate' to publish /initialpose. Localization will continue on live robot topics after that."
  else
    echo "Use RViz '2D Pose Estimate' to publish /initialpose, then bag playback will start."
  fi
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
  if [[ $MODE_IS_LIVE -eq 1 ]]; then
    echo "Initial pose accepted. Live localization is now running."
  else
    echo "Initial pose accepted. Starting localization playback now."
  fi
  sleep 2
else
  wait_pose_status=0
fi

if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
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
else
  bag_status=0
  echo "Press Ctrl-C to stop."
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
bag_status=$(normalize_status "$bag_status")
wait_pose_status=$(normalize_status "${wait_pose_status:-0}")
if [[ -n "${rviz_status:-}" ]]; then
  rviz_status=$(normalize_status "$rviz_status")
else
  rviz_status=0
fi

printf 'mode=%s\nbag_status=%s\nlaunch_status=%s\nrviz_status=%s\nmonitor_status=%s\nrecord_status=%s\nwait_initial_pose_status=%s\n' \
  "$MODE" "$bag_status" "$launch_status" "$rviz_status" "$monitor_status" "$record_status" "$wait_pose_status" \
  >"$OUTPUT_DIR/run_status.txt"

if [[ $MODE_COPY_PLY -eq 1 && -f "$PACKAGE_PLY_PATH" && "$PACKAGE_PLY_PATH" -nt "$run_marker" ]]; then
  cp "$PACKAGE_PLY_PATH" "$OUTPUT_DIR/saved_scans.ply"
fi

if [[ -d "$record_dir" && -f "$record_dir/metadata.yaml" ]]; then
  ros2 bag info "$record_dir" >"$OUTPUT_DIR/output_bag_info.txt"
fi

echo "Run complete. Outputs written to: $OUTPUT_DIR"
