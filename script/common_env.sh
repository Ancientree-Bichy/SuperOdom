#!/usr/bin/env bash

ROOT_DIR_FOR_ENV=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
LOCAL_DEPENDENCY_CONFIG="$ROOT_DIR_FOR_ENV/script/config/dependency_workspace.local.sh"

if [[ -f "$LOCAL_DEPENDENCY_CONFIG" ]]; then
  # shellcheck source=/dev/null
  source "$LOCAL_DEPENDENCY_CONFIG"
fi

resolve_dependency_ws_root() {
  if [[ -n "${DEPENDENCY_WS_ROOT:-}" ]]; then
    printf '%s\n' "$DEPENDENCY_WS_ROOT"
    return
  fi
  if [[ -n "${FAST_LIO2_LOC_ROOT:-}" ]]; then
    printf '%s\n' "$FAST_LIO2_LOC_ROOT"
    return
  fi
  printf '%s\n' ""
}

find_gtsam_dir() {
  if [[ -n "${GTSAM_DIR:-}" ]]; then
    printf '%s\n' "$GTSAM_DIR"
    return
  fi

  local dependency_ws_root=$1
  local candidates=()
  if [[ -n "$dependency_ws_root" ]]; then
    candidates+=("$dependency_ws_root/gtsam_install/lib/cmake/GTSAM")
  fi
  candidates+=(
    "/usr/local/lib/cmake/GTSAM"
    "/usr/lib/x86_64-linux-gnu/cmake/GTSAM"
    "/usr/lib/cmake/GTSAM"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "$candidate/GTSAMConfig.cmake" ]]; then
      printf '%s\n' "$candidate"
      return
    fi
  done

  printf '%s\n' ""
}

find_sophus_source_dir() {
  if [[ -n "${SOPHUS_SOURCE_DIR:-}" ]]; then
    printf '%s\n' "$SOPHUS_SOURCE_DIR"
    return
  fi

  local dependency_ws_root=$1
  local candidates=()
  if [[ -n "$dependency_ws_root" ]]; then
    candidates+=("$dependency_ws_root/Sophus")
  fi
  candidates+=(
    "/usr/local/include"
    "/usr/include"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -d "$candidate/sophus" ]]; then
      printf '%s\n' "$candidate"
      return
    fi
  done

  printf '%s\n' ""
}

source_ros_underlays() {
  local dependency_ws_root=$1
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -n "$dependency_ws_root" && -f "$dependency_ws_root/install/setup.bash" ]]; then
    source "$dependency_ws_root/install/setup.bash"
  fi
  set -u
}

source_ros_runtime_env() {
  local root_dir=$1
  local dependency_ws_root=$2
  local install_base=${SUPERODOM_INSTALL_BASE:-"$root_dir/install"}
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -n "$dependency_ws_root" && -f "$dependency_ws_root/install/setup.bash" ]]; then
    source "$dependency_ws_root/install/setup.bash"
  fi
  if [[ ! -f "$install_base/setup.bash" ]]; then
    echo "SuperOdom install setup not found: $install_base/setup.bash" >&2
    echo "Run script/build_superodom_local.sh or unset --skip-build." >&2
    set -u
    return 1
  fi
  source "$install_base/setup.bash"
  set -u
}
