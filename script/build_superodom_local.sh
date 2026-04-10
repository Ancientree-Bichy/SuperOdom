#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=script/common_env.sh
source "$ROOT_DIR/script/common_env.sh"

DEPENDENCY_WS_ROOT=$(resolve_dependency_ws_root)
SOPHUS_SOURCE_DIR=${SOPHUS_SOURCE_DIR:-"$(find_sophus_source_dir "$DEPENDENCY_WS_ROOT")"}
GTSAM_DIR=${GTSAM_DIR:-"$(find_gtsam_dir "$DEPENDENCY_WS_ROOT")"}
SOPHUS_SHIM_DIR="$ROOT_DIR/third_party/sophus_shim"

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

cat >"$SOPHUS_SHIM_DIR/SophusConfig.cmake" <<EOF
include(CMakeFindDependencyMacro)
find_dependency(Eigen3 REQUIRED)

if(NOT TARGET Sophus::Sophus)
  add_library(Sophus::Sophus INTERFACE IMPORTED)
  set_target_properties(Sophus::Sophus PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${SOPHUS_SOURCE_DIR}"
  )
endif()

set(Sophus_INCLUDE_DIRS "${SOPHUS_SOURCE_DIR}")
set(Sophus_FOUND TRUE)
EOF

cat >"$SOPHUS_SHIM_DIR/SophusConfigVersion.cmake" <<'EOF'
set(PACKAGE_VERSION "1.0.0")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT TRUE)
EOF

source_ros_underlays "$DEPENDENCY_WS_ROOT"

colcon build \
  --packages-up-to super_odometry super_odometry_msgs \
  --symlink-install \
  --cmake-args \
    -DGTSAM_DIR="$GTSAM_DIR" \
    -DSophus_DIR="$SOPHUS_SHIM_DIR"
