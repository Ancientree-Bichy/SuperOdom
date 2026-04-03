#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_local_bag.yaml"
FAST_LIO2_LOC_ROOT=${FAST_LIO2_LOC_ROOT:-"$HOME/workspace/fast_lio2_loc"}
PACKAGE_PLY_PATH="$ROOT_DIR/super_odometry/PLY/saved_scans.ply"
SKIP_BUILD=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--skip-build] <bag_dir> [run_name]

Examples:
  $(basename "$0") data_bag/ramp1
  $(basename "$0") data_bag/K-rail1 robocup_full

Environment variables:
  FAST_LIO2_LOC_ROOT   Override dependent ROS workspace root.
  CONFIG_FILE          Override test config file.
  OUTPUT_BASE_DIR      Override output root. Default: $ROOT_DIR/mapping_output
  EXTRA_LAUNCH_ARGS    Extra args appended to ros2 launch.
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

if [[ "${1:-}" == "--skip-build" ]]; then
  SKIP_BUILD=1
  shift
fi

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

BAG_PATH=$1
RUN_NAME=${2:-"$(basename "$BAG_PATH")_$(date +%Y%m%d_%H%M%S)"}
CONFIG_FILE=${CONFIG_FILE:-"$DEFAULT_CONFIG"}
OUTPUT_BASE_DIR=${OUTPUT_BASE_DIR:-"$ROOT_DIR/mapping_output"}
OUTPUT_DIR="$OUTPUT_BASE_DIR/$RUN_NAME"
EXTRA_LAUNCH_ARGS=${EXTRA_LAUNCH_ARGS:-}
ROS_HOME_DIR="$OUTPUT_DIR/ros_home"
ROS_LOG_DIR_VALUE="$OUTPUT_DIR/ros_log"

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Bag directory not found: $BAG_PATH" >&2
  exit 1
fi

if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "ROS2 bag metadata.yaml not found in: $BAG_PATH" >&2
  exit 1
fi

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Config file not found: $CONFIG_FILE" >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR"
mkdir -p "$ROS_HOME_DIR" "$ROS_LOG_DIR_VALUE"

cat >"$OUTPUT_DIR/run_metadata.txt" <<EOF
timestamp=$(date -Iseconds)
root_dir=$ROOT_DIR
bag_path=$BAG_PATH
run_name=$RUN_NAME
config_file=$CONFIG_FILE
skip_build=$SKIP_BUILD
extra_launch_args=$EXTRA_LAUNCH_ARGS
EOF

cat >"$OUTPUT_DIR/rerun_command.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "$ROOT_DIR"
bash script/run_superodom_livox_bag.sh --skip-build "$BAG_PATH" "$RUN_NAME"
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

ros2 bag info "$BAG_PATH" >"$OUTPUT_DIR/input_bag_info.txt"
cp "$CONFIG_FILE" "$OUTPUT_DIR/used_config.yaml"

run_marker="$OUTPUT_DIR/run_started.marker"
touch "$run_marker"

duration_raw=$(sed -n 's/^Duration:[[:space:]]*\([0-9.]*\)s/\1/p' "$OUTPUT_DIR/input_bag_info.txt")
if [[ -z "$duration_raw" ]]; then
  duration_raw=60
fi
duration_ceiling=$(awk -v value="$duration_raw" 'BEGIN { print int(value + 0.999999) }')
launch_timeout=$((duration_ceiling + 25))
record_timeout=$((duration_ceiling + 35))

record_dir="$OUTPUT_DIR/output_topics"
launch_log="$OUTPUT_DIR/launch.log"
bag_log="$OUTPUT_DIR/bag_play.log"
record_log="$OUTPUT_DIR/output_record.log"

record_cmd=(
  timeout "${record_timeout}s"
  ros2 bag record
  -o "$record_dir"
  /laser_odometry
  /state_estimation
  /laser_odom_path
  /imuodom_path
  /super_odometry_stats
)

launch_cmd=(
  timeout "${launch_timeout}s"
  ros2 launch super_odometry livox_mid360.launch.py
  "config_file:=$CONFIG_FILE"
)

if [[ -n "$EXTRA_LAUNCH_ARGS" ]]; then
  # shellcheck disable=SC2206
  extra_launch_array=($EXTRA_LAUNCH_ARGS)
  launch_cmd+=("${extra_launch_array[@]}")
fi

cleanup() {
  local pids=("$@")
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
}

set +e
"${record_cmd[@]}" >"$record_log" 2>&1 &
record_pid=$!

"${launch_cmd[@]}" >"$launch_log" 2>&1 &
launch_pid=$!

sleep 5
ros2 bag play "$BAG_PATH" >"$bag_log" 2>&1
bag_status=$?

sleep 5
cleanup "$record_pid" "$launch_pid"
wait "$record_pid"
record_status=$?
wait "$launch_pid"
launch_status=$?
set -e

printf 'bag_status=%s\nrecord_status=%s\nlaunch_status=%s\n' \
  "$bag_status" "$record_status" "$launch_status" >"$OUTPUT_DIR/run_status.txt"

if [[ -f "$PACKAGE_PLY_PATH" && "$PACKAGE_PLY_PATH" -nt "$run_marker" ]]; then
  cp "$PACKAGE_PLY_PATH" "$OUTPUT_DIR/saved_scans.ply"
fi

if [[ -d "$record_dir" && -f "$record_dir/metadata.yaml" ]]; then
  ros2 bag info "$record_dir" >"$OUTPUT_DIR/output_bag_info.txt"
fi

echo "Run complete. Outputs written to: $OUTPUT_DIR"
