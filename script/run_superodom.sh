#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=script/common_env.sh
source "$ROOT_DIR/script/common_env.sh"

DEPENDENCY_WS_ROOT=$(resolve_dependency_ws_root)
PACKAGE_PLY_PATH="$ROOT_DIR/super_odometry/PLY/saved_scans.ply"
DEFAULT_JT128_BAG_ROOT="$(dirname "$ROOT_DIR")/data/JT128bag"
ROS_PYTHON_EXECUTABLE=${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}
if [[ "$ROS_PYTHON_EXECUTABLE" != */* ]]; then
  ROS_PYTHON_EXECUTABLE=$(command -v "$ROS_PYTHON_EXECUTABLE" || true)
fi
if [[ -z "$ROS_PYTHON_EXECUTABLE" || ! -x "$ROS_PYTHON_EXECUTABLE" ]]; then
  echo "ROS Python executable not found: ${ROS_PYTHON_EXECUTABLE:-<empty>}" >&2
  echo "Set ROS_PYTHON_EXECUTABLE=/usr/bin/python3 or another ROS Humble Python 3.10 executable." >&2
  exit 1
fi

usage() {
  cat <<EOF
Usage:
  $(basename "$0") mapping-bag [options] <bag_name_or_dir> [run_name]
  $(basename "$0") mapping-live [options] [run_name]
  $(basename "$0") localization-bag [options] <bag_name_or_dir> <prior_map_pcd> [run_name]
  $(basename "$0") localization-live [options] <prior_map_pcd> [run_name]

Core options:
  --lidar MODEL             mid360 or jt128. Default: mid360
  --skip-build              Reuse the current install tree
  --rviz / --no-rviz        RViz is enabled by default when DISPLAY is set
  --rate RATE               ros2 bag play rate. Default: 1.0
  --startup-delay SEC       Delay between SuperOdom launch and bag play. Default: 5
  --keep-rviz / --no-keep-rviz
                            Keep RViz open after bag playback. Default: keep

JT128 options:
  --front / --rear / --fused
                            Select Unitree A2 front, rear, or fused topics
  --point-topic TOPIC       Override LiDAR point cloud topic
  --imu-topic TOPIC         Override IMU topic

Localization / SuperLoc options:
  --no-wait-initial-pose    Do not wait for RViz /initialpose before playback
  --record-output / --no-record-output
  --monitor / --no-monitor

Advanced overrides:
  --config-file FILE
  --calibration-file FILE
  --rviz-config FILE
  --output-base-dir DIR

Examples:
  # MID360 offline mapping with RViz, bag replay, and SuperOdom in one command
  bash script/run_superodom.sh mapping-bag --lidar mid360 data_bag/K-rail1 mid360_krail

  # Unitree A2 / JT128 front LiDAR offline mapping
  bash script/run_superodom.sh mapping-bag --lidar jt128 krail jt128_krail

  # Prior-map localization / SuperLoc-style mode
  bash script/run_superodom.sh localization-bag --lidar mid360 data_bag/K-rail1 /abs/map.pcd loc_run

  # Live robot, no bag path
  bash script/run_superodom.sh mapping-live --lidar jt128 live_jt128

Environment overrides:
  JT128_BAG_ROOT            Default: $DEFAULT_JT128_BAG_ROOT
  OUTPUT_BASE_DIR           Overrides the mode/lidar output directory
  PLAY_RATE                 Default: 1.0
  STARTUP_DELAY_SEC         Default: 5
  EXTRA_LAUNCH_ARGS         Extra ros2 launch args
  EXTRA_PLAY_ARGS           Extra ros2 bag play args
  JT128_IMU_ACC_SCALE       Optional JT128 IMU acceleration scale override
  JT128_IMU_GYR_SCALE       Optional JT128 IMU gyro scale override
  ROS_PYTHON_EXECUTABLE     Python used for ROS rclpy helpers. Default:
                            $ROS_PYTHON_EXECUTABLE
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
    DEFAULT_OUTPUT_KIND="mapping"
    ENABLE_RVIZ=1
    ENABLE_MONITOR=0
    WAIT_FOR_INITIAL_POSE=0
    RECORD_OUTPUT=1
    MODE_REQUIRES_BAG=1
    MODE_REQUIRES_MAP=0
    MODE_IS_LIVE=0
    MODE_COPY_PLY=1
    ;;
  mapping-live)
    DEFAULT_OUTPUT_KIND="mapping_live"
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
    DEFAULT_OUTPUT_KIND="localization"
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
    DEFAULT_OUTPUT_KIND="localization"
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
LIDAR_MODEL=${LIDAR_MODEL:-mid360}
LIDAR_SOURCE="front"
POINT_TOPIC=""
IMU_TOPIC=""
POINT_TOPIC_EXPLICIT=0
IMU_TOPIC_EXPLICIT=0
CONFIG_FILE_OVERRIDE=""
CALIBRATION_FILE_OVERRIDE=""
RVIZ_CONFIG_FILE_OVERRIDE=""
OUTPUT_BASE_DIR_OVERRIDE=""
KEEP_RVIZ_OPEN=${KEEP_RVIZ_OPEN:-1}
PLAY_RATE=${PLAY_RATE:-1.0}
STARTUP_DELAY_SEC=${STARTUP_DELAY_SEC:-5}
INITIAL_POSE_TIMEOUT_SEC=${INITIAL_POSE_TIMEOUT_SEC:-0}
MONITOR_PERIOD=${MONITOR_PERIOD:-0.5}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --lidar)
      LIDAR_MODEL=${2:?--lidar requires mid360 or jt128}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --mid360)
      LIDAR_MODEL="mid360"
      SCRIPT_FLAGS+=("--lidar" "mid360")
      shift
      ;;
    --jt128)
      LIDAR_MODEL="jt128"
      SCRIPT_FLAGS+=("--lidar" "jt128")
      shift
      ;;
    --front)
      LIDAR_SOURCE="front"
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --rear)
      LIDAR_SOURCE="rear"
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --fused)
      LIDAR_SOURCE="fused"
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --point-topic)
      POINT_TOPIC=${2:?--point-topic requires a topic}
      POINT_TOPIC_EXPLICIT=1
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --imu-topic)
      IMU_TOPIC=${2:?--imu-topic requires a topic}
      IMU_TOPIC_EXPLICIT=1
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --rviz)
      ENABLE_RVIZ=1
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --no-rviz)
      ENABLE_RVIZ=0
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --monitor)
      ENABLE_MONITOR=1
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
    --rate)
      PLAY_RATE=${2:?--rate requires a numeric value}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --startup-delay)
      STARTUP_DELAY_SEC=${2:?--startup-delay requires a numeric value}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --keep-rviz)
      KEEP_RVIZ_OPEN=1
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --no-keep-rviz)
      KEEP_RVIZ_OPEN=0
      SCRIPT_FLAGS+=("$1")
      shift
      ;;
    --config-file)
      CONFIG_FILE_OVERRIDE=${2:?--config-file requires a path}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --calibration-file)
      CALIBRATION_FILE_OVERRIDE=${2:?--calibration-file requires a path}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --rviz-config)
      RVIZ_CONFIG_FILE_OVERRIDE=${2:?--rviz-config requires a path}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
      ;;
    --output-base-dir)
      OUTPUT_BASE_DIR_OVERRIDE=${2:?--output-base-dir requires a path}
      SCRIPT_FLAGS+=("$1" "$2")
      shift 2
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

case "$LIDAR_MODEL" in
  mid360|livox)
    LIDAR_MODEL="mid360"
    ;;
  jt128|hesai)
    LIDAR_MODEL="jt128"
    ;;
  *)
    echo "Unknown --lidar value: $LIDAR_MODEL" >&2
    usage
    exit 1
    ;;
esac

resolve_bag_path() {
  local selector=$1
  local bag_root=$2
  if [[ -d "$selector" && -f "$selector/metadata.yaml" ]]; then
    printf '%s\n' "$selector"
    return
  fi
  if [[ -n "$bag_root" && -d "$bag_root/$selector" && -f "$bag_root/$selector/metadata.yaml" ]]; then
    printf '%s\n' "$bag_root/$selector"
    return
  fi
  echo "Bag not found or missing metadata.yaml: $selector" >&2
  if [[ -n "$bag_root" ]]; then
    echo "Also tried: $bag_root/$selector" >&2
  fi
  exit 1
}

BAG_SELECTOR=""
BAG_PATH=""
PRIOR_MAP_PCD=""
RUN_NAME=""

if [[ $MODE_REQUIRES_BAG -eq 1 && $MODE_REQUIRES_MAP -eq 1 ]]; then
  if [[ $# -lt 2 ]]; then
    usage
    exit 1
  fi
  BAG_SELECTOR=$1
  PRIOR_MAP_PCD=$2
  RUN_NAME=${3:-"$(basename "$BAG_SELECTOR")_${LIDAR_MODEL}_localization_$(date +%Y%m%d_%H%M%S)"}
elif [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  if [[ $# -lt 1 ]]; then
    usage
    exit 1
  fi
  BAG_SELECTOR=$1
  RUN_NAME=${2:-"$(basename "$BAG_SELECTOR")_${LIDAR_MODEL}_$(date +%Y%m%d_%H%M%S)"}
elif [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  if [[ $# -lt 1 ]]; then
    usage
    exit 1
  fi
  PRIOR_MAP_PCD=$1
  RUN_NAME=${2:-"$(basename "$PRIOR_MAP_PCD" .pcd)_${LIDAR_MODEL}_${MODE}_$(date +%Y%m%d_%H%M%S)"}
else
  RUN_NAME=${1:-"${LIDAR_MODEL}_${MODE}_$(date +%Y%m%d_%H%M%S)"}
fi

if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  if [[ "$LIDAR_MODEL" == "mid360" ]]; then
    BAG_PATH=$(resolve_bag_path "$BAG_SELECTOR" "$ROOT_DIR/data_bag")
  else
    BAG_PATH=$(resolve_bag_path "$BAG_SELECTOR" "${JT128_BAG_ROOT:-"$DEFAULT_JT128_BAG_ROOT"}")
  fi
fi

if [[ $MODE_REQUIRES_MAP -eq 1 && ! -f "$PRIOR_MAP_PCD" ]]; then
  echo "Prior map not found: $PRIOR_MAP_PCD" >&2
  exit 1
fi

bag_has_topic() {
  local topic=$1
  [[ -n "$BAG_PATH" ]] && grep -q "name: $topic\$" "$BAG_PATH/metadata.yaml"
}

first_available_topic() {
  local topic
  for topic in "$@"; do
    if bag_has_topic "$topic"; then
      printf '%s\n' "$topic"
      return
    fi
  done
  printf '%s\n' ""
}

if [[ "$LIDAR_MODEL" == "mid360" ]]; then
  if [[ $POINT_TOPIC_EXPLICIT -eq 0 ]]; then
    POINT_TOPIC="/livox/lidar"
  fi
  if [[ $IMU_TOPIC_EXPLICIT -eq 0 ]]; then
    IMU_TOPIC="/livox/imu"
  fi
else
  if [[ $POINT_TOPIC_EXPLICIT -eq 0 ]]; then
    if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
      case "$LIDAR_SOURCE" in
        rear)
          POINT_TOPIC=$(first_available_topic "/rt/unitree/slam_lidar/points2" "/lidar_points_2")
          ;;
        fused)
          POINT_TOPIC=$(first_available_topic "/rt/unitree/slam_lidar/points" "/lidar_points")
          ;;
        *)
          POINT_TOPIC=$(first_available_topic "/rt/unitree/slam_lidar/points1" "/lidar_points1" "/lidar_points")
          ;;
      esac
    else
      case "$LIDAR_SOURCE" in
        rear) POINT_TOPIC="/rt/unitree/slam_lidar/points2" ;;
        fused) POINT_TOPIC="/rt/unitree/slam_lidar/points" ;;
        *) POINT_TOPIC="/rt/unitree/slam_lidar/points1" ;;
      esac
    fi
  fi
  if [[ $IMU_TOPIC_EXPLICIT -eq 0 ]]; then
    if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
      case "$LIDAR_SOURCE" in
        rear)
          IMU_TOPIC=$(first_available_topic "/rt/unitree/slam_lidar/imu2" "/lidar_imu_2")
          ;;
        fused)
          IMU_TOPIC=$(first_available_topic "/rt/unitree/slam_lidar/imu1" "/lidar_imu1" "/lidar_imu")
          ;;
        *)
          IMU_TOPIC=$(first_available_topic "/rt/unitree/slam_lidar/imu1" "/lidar_imu1" "/lidar_imu")
          ;;
      esac
    else
      case "$LIDAR_SOURCE" in
        rear) IMU_TOPIC="/rt/unitree/slam_lidar/imu2" ;;
        *) IMU_TOPIC="/rt/unitree/slam_lidar/imu1" ;;
      esac
    fi
  fi
  if [[ -z "$POINT_TOPIC" || -z "$IMU_TOPIC" ]]; then
    echo "Could not resolve $LIDAR_SOURCE JT128 topics from bag metadata: $BAG_PATH/metadata.yaml" >&2
    echo "Use --point-topic and --imu-topic to override explicitly." >&2
    exit 1
  fi
fi

if [[ "$LIDAR_MODEL" == "mid360" ]]; then
  if [[ "$MODE" == localization-* ]]; then
    DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_localization.yaml"
  else
    DEFAULT_CONFIG="$ROOT_DIR/script/config/livox_mid360_local_bag.yaml"
  fi
  DEFAULT_CALIBRATION="$ROOT_DIR/super_odometry/config/livox/livox_mid360_calibration.yaml"
  DEFAULT_RVIZ="$ROOT_DIR/super_odometry/ros2_minimal.rviz"
  DEFAULT_LAUNCH="livox_mid360.launch.py"
  DEFAULT_OUTPUT_PREFIX=""
else
  DEFAULT_CONFIG="$ROOT_DIR/super_odometry/config/hesai_jt128.yaml"
  DEFAULT_CALIBRATION="$ROOT_DIR/super_odometry/config/hesai/jt128_calibration.yaml"
  DEFAULT_RVIZ="$ROOT_DIR/super_odometry/rviz_jt128_debug.rviz"
  DEFAULT_LAUNCH="hesai_jt128_superodom.launch.py"
  DEFAULT_OUTPUT_PREFIX="_jt128"
fi

case "$DEFAULT_OUTPUT_KIND" in
  mapping)
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/mapping_output${DEFAULT_OUTPUT_PREFIX}"
    ;;
  mapping_live)
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/mapping_output_live${DEFAULT_OUTPUT_PREFIX}"
    ;;
  localization)
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/loc_output${DEFAULT_OUTPUT_PREFIX}"
    ;;
  *)
    DEFAULT_OUTPUT_BASE_DIR="$ROOT_DIR/run_output${DEFAULT_OUTPUT_PREFIX}"
    ;;
esac

CONFIG_TEMPLATE_FILE=${CONFIG_FILE_OVERRIDE:-${CONFIG_FILE:-"$DEFAULT_CONFIG"}}
CALIBRATION_FILE=${CALIBRATION_FILE_OVERRIDE:-${CALIBRATION_FILE:-"$DEFAULT_CALIBRATION"}}
RVIZ_CONFIG_FILE=${RVIZ_CONFIG_FILE_OVERRIDE:-${RVIZ_CONFIG_FILE:-"$DEFAULT_RVIZ"}}
OUTPUT_BASE_DIR=${OUTPUT_BASE_DIR_OVERRIDE:-${OUTPUT_BASE_DIR:-"$DEFAULT_OUTPUT_BASE_DIR"}}
OUTPUT_DIR="$OUTPUT_BASE_DIR/$RUN_NAME"
ROS_HOME_DIR="$OUTPUT_DIR/ros_home"
ROS_LOG_DIR_VALUE="$OUTPUT_DIR/ros_log"
EFFECTIVE_CONFIG_FILE="$OUTPUT_DIR/effective_config.yaml"
USED_CONFIG_FILE="$OUTPUT_DIR/used_config.yaml"
EXTRA_LAUNCH_ARGS=${EXTRA_LAUNCH_ARGS:-}
EXTRA_PLAY_ARGS=${EXTRA_PLAY_ARGS:-}
JT128_IMU_ACC_SCALE=${JT128_IMU_ACC_SCALE:-}
JT128_IMU_GYR_SCALE=${JT128_IMU_GYR_SCALE:-}

if [[ ! -f "$CONFIG_TEMPLATE_FILE" ]]; then
  echo "Config file not found: $CONFIG_TEMPLATE_FILE" >&2
  exit 1
fi
if [[ ! -f "$CALIBRATION_FILE" ]]; then
  echo "Calibration file not found: $CALIBRATION_FILE" >&2
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

mkdir -p "$OUTPUT_DIR" "$ROS_HOME_DIR" "$ROS_LOG_DIR_VALUE"

IS_LOCALIZATION=0
if [[ "$MODE" == localization-* ]]; then
  IS_LOCALIZATION=1
fi

ADAPTER_OUTPUT_TOPIC="/hesai/jt128/points_superodom"
LASER_TOPIC="$POINT_TOPIC"
if [[ "$LIDAR_MODEL" != "mid360" ]]; then
  LASER_TOPIC="$ADAPTER_OUTPUT_TOPIC"
fi

"$ROS_PYTHON_EXECUTABLE" - "$CONFIG_TEMPLATE_FILE" "$EFFECTIVE_CONFIG_FILE" "$LASER_TOPIC" "$IMU_TOPIC" \
  "$POINT_TOPIC" "$ADAPTER_OUTPUT_TOPIC" "$IS_LOCALIZATION" "$WAIT_FOR_INITIAL_POSE" \
  "$JT128_IMU_ACC_SCALE" "$JT128_IMU_GYR_SCALE" <<'PY'
import sys
import yaml

(
    source_path,
    output_path,
    laser_topic,
    imu_topic,
    point_topic,
    adapter_output_topic,
    is_localization,
    wait_for_initial_pose,
    imu_acc_scale,
    imu_gyr_scale,
) = sys.argv[1:11]

with open(source_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}

params = data.setdefault("/**", {}).setdefault("ros__parameters", {})
params["laser_topic"] = laser_topic
params["imu_topic"] = imu_topic

mapping = params.setdefault("laser_mapping_node", {})
mapping["localization_mode"] = is_localization == "1"
mapping["use_rviz_initial_pose"] = is_localization == "1" and wait_for_initial_pose == "1"
mapping.setdefault("rviz_initial_pose_xy_yaw_only", True)
mapping["read_pose_file"] = False

adapter = data.get("hesai_to_superodom_node", {}).get("ros__parameters")
if isinstance(adapter, dict):
    adapter["input_point_cloud_topic"] = point_topic
    adapter["output_point_cloud_topic"] = adapter_output_topic

if imu_acc_scale:
    value = float(imu_acc_scale)
    params.setdefault("feature_extraction_node", {})["imu_acc_scale"] = value
    params.setdefault("imu_preintegration_node", {})["imu_acc_scale"] = value
if imu_gyr_scale:
    value = float(imu_gyr_scale)
    params.setdefault("feature_extraction_node", {})["imu_gyr_scale"] = value
    params.setdefault("imu_preintegration_node", {})["imu_gyr_scale"] = value

with open(output_path, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, sort_keys=False)
PY

CONFIG_FILE="$EFFECTIVE_CONFIG_FILE"

read_config_value() {
  local path_expr=$1
  "$ROS_PYTHON_EXECUTABLE" - "$CONFIG_FILE" "$path_expr" <<'PY'
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
RVIZ_INITIAL_POSE_FRAME_CFG=$(read_config_value laser_mapping_node.rviz_initial_pose_frame)
LASER_TOPIC_CFG=$(read_config_value laser_topic)
IMU_TOPIC_CFG=$(read_config_value imu_topic)

INITIAL_POSE_FILE=""
if [[ -n "$PRIOR_MAP_PCD" ]]; then
  PRIOR_MAP_DIR=$(dirname "$PRIOR_MAP_PCD")
  PRIOR_MAP_STEM=$(basename "$PRIOR_MAP_PCD")
  PRIOR_MAP_STEM=${PRIOR_MAP_STEM%.*}
  INITIAL_POSE_FILE="$PRIOR_MAP_DIR/$PRIOR_MAP_STEM.start_pose.txt"
fi

quoted_cli_args=()
for item in "${SCRIPT_FLAGS[@]}"; do
  printf -v q '%q' "$item"
  quoted_cli_args+=("$q")
done
if [[ -n "$BAG_SELECTOR" ]]; then
  printf -v q '%q' "$BAG_SELECTOR"
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
lidar_model=$LIDAR_MODEL
lidar_source=$LIDAR_SOURCE
root_dir=$ROOT_DIR
bag_path=$BAG_PATH
prior_map_pcd=$PRIOR_MAP_PCD
initial_pose_file=$INITIAL_POSE_FILE
run_name=$RUN_NAME
config_template_file=$CONFIG_TEMPLATE_FILE
config_file=$CONFIG_FILE
calibration_file=$CALIBRATION_FILE
rviz_config_file=$RVIZ_CONFIG_FILE
skip_build=$SKIP_BUILD
enable_rviz=$ENABLE_RVIZ
enable_monitor=$ENABLE_MONITOR
wait_for_initial_pose=$WAIT_FOR_INITIAL_POSE
rviz_initial_pose_frame=$RVIZ_INITIAL_POSE_FRAME_CFG
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
input_point_topic=$POINT_TOPIC
EOF

printf -v quoted_mode '%q' "$MODE"
cat >"$OUTPUT_DIR/rerun_command.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "$ROOT_DIR"
bash script/run_superodom.sh $quoted_mode ${quoted_cli_args[*]}
EOF
chmod +x "$OUTPUT_DIR/rerun_command.sh"

if [[ "$LIDAR_MODEL" == "mid360" ]]; then
  BUILD_BASE_ENV=${SUPERODOM_BUILD_BASE:-}
  INSTALL_BASE_ENV=${SUPERODOM_INSTALL_BASE:-}
  LOG_BASE_ENV=${SUPERODOM_LOG_BASE:-}
else
  BUILD_BASE_ENV=${SUPERODOM_BUILD_BASE:-"$ROOT_DIR/.superodom_build/jt128"}
  INSTALL_BASE_ENV=${SUPERODOM_INSTALL_BASE:-"$ROOT_DIR/.superodom_install/jt128"}
  LOG_BASE_ENV=${SUPERODOM_LOG_BASE:-"$ROOT_DIR/.superodom_log/jt128"}
fi

if [[ $SKIP_BUILD -eq 0 ]]; then
  if [[ -n "$BUILD_BASE_ENV" || -n "$INSTALL_BASE_ENV" || -n "$LOG_BASE_ENV" ]]; then
    SUPERODOM_BUILD_BASE="$BUILD_BASE_ENV" \
      SUPERODOM_INSTALL_BASE="$INSTALL_BASE_ENV" \
      SUPERODOM_LOG_BASE="$LOG_BASE_ENV" \
      "$ROOT_DIR/script/build_superodom_local.sh"
  else
    "$ROOT_DIR/script/build_superodom_local.sh"
  fi
fi

if [[ -n "$INSTALL_BASE_ENV" ]]; then
  export SUPERODOM_INSTALL_BASE="$INSTALL_BASE_ENV"
fi
source_ros_runtime_env "$ROOT_DIR" "$DEPENDENCY_WS_ROOT"

export ROS_HOME="$ROS_HOME_DIR"
export ROS_LOG_DIR="$ROS_LOG_DIR_VALUE"
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-1}

if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  ros2 bag info "$BAG_PATH" >"$OUTPUT_DIR/input_bag_info.txt"
fi
cp "$CONFIG_TEMPLATE_FILE" "$OUTPUT_DIR/source_config.yaml"
cp "$CONFIG_FILE" "$USED_CONFIG_FILE"
cp "$CALIBRATION_FILE" "$OUTPUT_DIR/used_calibration.yaml"
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

wait_for_child_status() {
  local pid=$1
  local state
  while kill -0 "$pid" 2>/dev/null; do
    state=$(ps -o stat= -p "$pid" 2>/dev/null | awk '{print $1}')
    [[ "$state" == Z* ]] && break
    sleep 0.2
  done
  wait "$pid"
}

normalize_status() {
  local status=$1
  case "$status" in
    0|124|130|143)
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
  ros2 launch super_odometry "$DEFAULT_LAUNCH"
  "config_file:=$CONFIG_FILE"
  "calibration_file:=$CALIBRATION_FILE"
)
if [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  launch_cmd+=("map_dir:=$PRIOR_MAP_PCD")
fi
if [[ "$LIDAR_MODEL" != "mid360" ]]; then
  launch_cmd+=(
    "input_point_cloud_topic:=$POINT_TOPIC"
    "output_point_cloud_topic:=$ADAPTER_OUTPUT_TOPIC"
    "imu_topic:=$IMU_TOPIC"
  )
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
    /body_odometry
    /super_odometry_stats
  )
fi

set +e
if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  launch_timeout=$((duration_ceiling + STARTUP_DELAY_SEC + 35))
  start_group "$launch_log" timeout "${launch_timeout}s" "${launch_cmd[@]}"
else
  start_group "$launch_log" "${launch_cmd[@]}"
fi
launch_pid=$STARTED_PID
sleep "$STARTUP_DELAY_SEC"

if [[ $ENABLE_MONITOR -eq 1 ]]; then
  setsid "$ROS_PYTHON_EXECUTABLE" "$ROOT_DIR/script/monitor_superodom_stats_live.py" \
    --topic /super_odometry_stats \
    --min-period "$MONITOR_PERIOD" \
    --log-file "$monitor_log" &
  monitor_pid=$!
fi

if [[ $RECORD_OUTPUT -eq 1 ]]; then
  if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
    record_timeout=$((duration_ceiling + STARTUP_DELAY_SEC + 45))
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
echo "  lidar:      $LIDAR_MODEL"
echo "  output_dir: $OUTPUT_DIR"
if [[ $MODE_REQUIRES_BAG -eq 1 ]]; then
  echo "  bag_path:   $BAG_PATH"
fi
if [[ $MODE_REQUIRES_MAP -eq 1 ]]; then
  echo "  prior_map:  $PRIOR_MAP_PCD"
  echo "  init_pose:  $INITIAL_POSE_FILE"
fi
echo "  config:     $CONFIG_FILE"
echo "  lidar_topic:$POINT_TOPIC"
echo "  imu_topic:  $IMU_TOPIC"
if [[ $ENABLE_RVIZ -eq 1 ]]; then
  echo "  rviz:       $RVIZ_CONFIG_FILE"
fi
if [[ $MODE_REQUIRES_MAP -eq 1 && "$USE_RVIZ_INITIAL_POSE_CFG" == "true" ]]; then
  if [[ "$RVIZ_XY_YAW_ONLY_CFG" == "true" ]]; then
    echo "  rviz_init:  x/y/yaw only; z/roll/pitch are preserved from config"
  else
    echo "  rviz_init:  full pose from /initialpose"
  fi
fi
if [[ $MODE_IS_LIVE -eq 1 ]]; then
  echo "  live input topics:"
  echo "    lidar: $POINT_TOPIC"
  echo "    imu:   $IMU_TOPIC"
fi

if [[ $MODE_REQUIRES_MAP -eq 1 && $WAIT_FOR_INITIAL_POSE -eq 1 ]]; then
  if [[ $MODE_IS_LIVE -eq 1 ]]; then
    echo "Use RViz '2D Pose Estimate' to publish /initialpose. Localization will continue on live robot topics after that."
  else
    echo "Use RViz '2D Pose Estimate' to publish /initialpose, then bag playback will start."
  fi
  setsid "$ROS_PYTHON_EXECUTABLE" "$ROOT_DIR/script/wait_for_initial_pose.py" \
    --topic /initialpose \
    --timeout-sec "$INITIAL_POSE_TIMEOUT_SEC" >"$wait_pose_log" 2>&1 &
  wait_pose_pid=$!
  wait_for_child_status "$wait_pose_pid"
  wait_pose_status=$?
  wait_pose_pid=""
  if [[ $wait_pose_status -ne 0 ]]; then
    if [[ $wait_pose_status -eq 130 || $wait_pose_status -eq 143 ]]; then
      exit "$wait_pose_status"
    fi
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
  wait_for_child_status "$bag_pid"
  bag_status=$?
  bag_pid=""

  if [[ $KEEP_RVIZ_OPEN -eq 1 && -n "$rviz_pid" ]] && kill -0 "$rviz_pid" 2>/dev/null; then
    echo "Bag playback finished. Close RViz to end the session, or press Ctrl-C."
    wait_for_child_status "$rviz_pid"
    rviz_status=$?
    rviz_pid=""
  else
    rviz_status=0
  fi
else
  bag_status=0
  echo "Press Ctrl-C to stop."
  wait_for_child_status "$launch_pid"
  launch_status=$?
  launch_pid=""
fi

cleanup

if [[ -n "$launch_pid" ]]; then
  wait_for_child_status "$launch_pid"
  launch_status=$?
  launch_pid=""
else
  launch_status=${launch_status:-0}
fi

if [[ -n "$monitor_pid" ]]; then
  wait_for_child_status "$monitor_pid"
  monitor_status=$?
  monitor_pid=""
else
  monitor_status=0
fi

if [[ -n "$record_pid" ]]; then
  wait_for_child_status "$record_pid"
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

printf 'mode=%s\nlidar_model=%s\nbag_status=%s\nlaunch_status=%s\nrviz_status=%s\nmonitor_status=%s\nrecord_status=%s\nwait_initial_pose_status=%s\n' \
  "$MODE" "$LIDAR_MODEL" "$bag_status" "$launch_status" "$rviz_status" "$monitor_status" "$record_status" "$wait_pose_status" \
  >"$OUTPUT_DIR/run_status.txt"

if [[ $MODE_COPY_PLY -eq 1 && -f "$PACKAGE_PLY_PATH" && "$PACKAGE_PLY_PATH" -nt "$run_marker" ]]; then
  cp "$PACKAGE_PLY_PATH" "$OUTPUT_DIR/saved_scans.ply"
fi

if [[ -d "$record_dir" && -f "$record_dir/metadata.yaml" ]]; then
  ros2 bag info "$record_dir" >"$OUTPUT_DIR/output_bag_info.txt"
fi

echo "Run complete. Outputs written to: $OUTPUT_DIR"
exit "$(( bag_status || launch_status || rviz_status || monitor_status || record_status || wait_pose_status ))"
