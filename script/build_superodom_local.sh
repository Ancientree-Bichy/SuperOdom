#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
FAST_LIO2_LOC_ROOT=${FAST_LIO2_LOC_ROOT:-"$HOME/workspace/fast_lio2_loc"}
SOPHUS_SOURCE_DIR=${SOPHUS_SOURCE_DIR:-"$FAST_LIO2_LOC_ROOT/Sophus"}
GTSAM_DIR=${GTSAM_DIR:-"$FAST_LIO2_LOC_ROOT/gtsam_install/lib/cmake/GTSAM"}
SOPHUS_SHIM_DIR="$ROOT_DIR/third_party/sophus_shim"

if [[ ! -d "$SOPHUS_SOURCE_DIR/sophus" ]]; then
  echo "Sophus source directory not found: $SOPHUS_SOURCE_DIR" >&2
  exit 1
fi

if [[ ! -f "$GTSAM_DIR/GTSAMConfig.cmake" ]]; then
  echo "GTSAMConfig.cmake not found: $GTSAM_DIR/GTSAMConfig.cmake" >&2
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

set +u
source /opt/ros/humble/setup.bash
source "$FAST_LIO2_LOC_ROOT/install/setup.bash"
set -u

colcon build \
  --packages-up-to super_odometry super_odometry_msgs \
  --symlink-install \
  --cmake-args \
    -DGTSAM_DIR="$GTSAM_DIR" \
    -DSophus_DIR="$SOPHUS_SHIM_DIR"
