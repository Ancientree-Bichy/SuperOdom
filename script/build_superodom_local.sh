#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=script/common_env.sh
source "$ROOT_DIR/script/common_env.sh"

DEPENDENCY_WS_ROOT=$(resolve_dependency_ws_root)
SOPHUS_SOURCE_DIR=${SOPHUS_SOURCE_DIR:-"$(find_sophus_source_dir "$DEPENDENCY_WS_ROOT")"}
GTSAM_DIR=${GTSAM_DIR:-"$(find_gtsam_dir "$DEPENDENCY_WS_ROOT")"}
SOPHUS_SHIM_DIR="$ROOT_DIR/third_party/sophus_shim"
COLCON_PREFIX_ARGS=()
COLCON_BASE_ARGS=()

clean_default_build_tree_if_moved() {
  if [[ -n "${SUPERODOM_BUILD_BASE:-}" || -n "${SUPERODOM_INSTALL_BASE:-}" || -n "${SUPERODOM_LOG_BASE:-}" ]]; then
    return
  fi

  local cache source_dir binary_dir
  local stale_cache=""
  while IFS= read -r -d '' cache; do
    source_dir=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache" | head -n 1)
    binary_dir=$(sed -n 's/^# For build in directory: //p' "$cache" | head -n 1)
    if [[ -n "$source_dir" && "$source_dir" != "$ROOT_DIR/"* ]]; then
      stale_cache=$cache
      break
    fi
    if [[ -n "$binary_dir" && "$binary_dir" != "$ROOT_DIR/build/"* ]]; then
      stale_cache=$cache
      break
    fi
  done < <(find "$ROOT_DIR/build" -name CMakeCache.txt -print0 2>/dev/null || true)

  if [[ -n "$stale_cache" ]]; then
    cat >&2 <<EOF
Detected stale CMake cache from another SuperOdom checkout:
  $stale_cache

Cleaning generated default colcon directories under:
  $ROOT_DIR/build
  $ROOT_DIR/install
  $ROOT_DIR/log
EOF
    rm -rf "$ROOT_DIR/build" "$ROOT_DIR/install" "$ROOT_DIR/log"
  fi
}

if [[ -n "${SUPERODOM_LOG_BASE:-}" ]]; then
  COLCON_PREFIX_ARGS+=(--log-base "$SUPERODOM_LOG_BASE")
fi
if [[ -n "${SUPERODOM_BUILD_BASE:-}" ]]; then
  COLCON_BASE_ARGS+=(--build-base "$SUPERODOM_BUILD_BASE")
fi
if [[ -n "${SUPERODOM_INSTALL_BASE:-}" ]]; then
  COLCON_BASE_ARGS+=(--install-base "$SUPERODOM_INSTALL_BASE")
fi

clean_default_build_tree_if_moved

if [[ ! -d "$SOPHUS_SOURCE_DIR/sophus" ]]; then
  cat >&2 <<EOF
Sophus source directory not found.

Checked:
  SOPHUS_SOURCE_DIR=${SOPHUS_SOURCE_DIR:-<empty>}
  DEPENDENCY_WS_ROOT=${DEPENDENCY_WS_ROOT:-<empty>}

Provide one of:
  1. SOPHUS_SOURCE_DIR=/path/to/Sophus_source
  2. DEPENDENCY_WS_ROOT=/path/to/external_ros_workspace

See:
  doc/LOC_TASK_GUIDE.md
EOF
  exit 1
fi

if [[ ! -f "$GTSAM_DIR/GTSAMConfig.cmake" ]]; then
  cat >&2 <<EOF
GTSAMConfig.cmake not found.

Checked:
  GTSAM_DIR=${GTSAM_DIR:-<empty>}
  DEPENDENCY_WS_ROOT=${DEPENDENCY_WS_ROOT:-<empty>}

Provide one of:
  1. GTSAM_DIR=/path/to/GTSAM/cmake/dir
  2. DEPENDENCY_WS_ROOT=/path/to/external_ros_workspace

Expected:
  <GTSAM_DIR>/GTSAMConfig.cmake

See:
  doc/LOC_TASK_GUIDE.md
EOF
  exit 1
fi

mkdir -p "$SOPHUS_SHIM_DIR"
export DEPENDENCY_WS_ROOT
export SOPHUS_SOURCE_DIR

cat >"$SOPHUS_SHIM_DIR/SophusConfig.cmake" <<EOF
include(CMakeFindDependencyMacro)
find_dependency(Eigen3 REQUIRED)

set(_sophus_source_dir "\$ENV{SOPHUS_SOURCE_DIR}")
if("\${_sophus_source_dir}" STREQUAL "" AND NOT "\$ENV{DEPENDENCY_WS_ROOT}" STREQUAL "")
  set(_sophus_source_dir "\$ENV{DEPENDENCY_WS_ROOT}/Sophus")
endif()
if("\${_sophus_source_dir}" STREQUAL "")
  message(FATAL_ERROR "Set SOPHUS_SOURCE_DIR or DEPENDENCY_WS_ROOT before finding Sophus")
endif()

if(NOT TARGET Sophus::Sophus)
  add_library(Sophus::Sophus INTERFACE IMPORTED)
  set_target_properties(Sophus::Sophus PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "\${_sophus_source_dir}"
  )
endif()

set(Sophus_INCLUDE_DIRS "\${_sophus_source_dir}")
set(Sophus_FOUND TRUE)
EOF

cat >"$SOPHUS_SHIM_DIR/SophusConfigVersion.cmake" <<'EOF'
set(PACKAGE_VERSION "1.0.0")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT TRUE)
EOF

source_ros_underlays "$DEPENDENCY_WS_ROOT"

colcon "${COLCON_PREFIX_ARGS[@]}" build \
  "${COLCON_BASE_ARGS[@]}" \
  --packages-up-to super_odometry super_odometry_msgs \
  --symlink-install \
  --cmake-args \
    -DGTSAM_DIR="$GTSAM_DIR" \
    -DSophus_DIR="$SOPHUS_SHIM_DIR"
