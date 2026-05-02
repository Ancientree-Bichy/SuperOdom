# AGENTS.md

This repository is a ROS 2 Humble workspace for `SuperOdom`.

## Environment

- Source ROS before running anything substantial:
  - `source /opt/ros/humble/setup.bash`
- If dependencies such as `livox_ros_driver2` or non-system `GTSAM`/`Sophus` are provided by
  an external ROS workspace, point scripts at it with:
  - `DEPENDENCY_WS_ROOT=/path/to/that/workspace`
- Prefer the provided build script over raw `colcon` because it wires the local `GTSAM` install and generates the temporary `Sophus` CMake shim:
  - `bash script/build_superodom_local.sh`

## Preferred run paths

- Offline mapping / odometry replay:
  - `bash script/run_superodom.sh mapping-bag --skip-build data_bag/ramp1 smoke_ramp1`
- Live replay with terminal stats and optional RViz:
  - `bash script/run_superodom.sh mapping-live --skip-build live_smoke_ramp1`
- Offline prior-map localization:
  - `bash script/run_superodom.sh localization-bag --skip-build data_bag/ramp1 /abs/path/map.pcd loc_smoke`
- Live prior-map localization:
  - `bash script/run_superodom.sh localization-live --skip-build /abs/path/map.pcd live_loc`
- Local Livox bag config lives at:
  - `script/config/livox_mid360_local_bag.yaml`
- Local Livox localization config lives at:
  - `script/config/livox_mid360_localization.yaml`
- That config assumes:
  - lidar topic: `/livox/lidar`
  - imu topic: `/livox/imu`

## Validation

- For code changes, prefer a short bag smoke test over blind edits.
- Smallest useful replay in this workspace is usually:
  - `data_bag/ramp1`
- If GUI is unavailable, use:
  - `bash script/run_superodom.sh mapping-live --skip-build --no-rviz live_smoke_ramp1`

## Commit hygiene

- Do not commit local datasets or generated run outputs.
- Keep these paths untracked:
  - `build/`
  - `install/`
  - `log/`
  - `data_bag/`
  - `mapping_output/`
  - `mapping_output_live/`
  - `loc_output/`
  - `super_odometry/PLY/`
  - `third_party/sophus_shim/`
- Commit helper scripts and configs under `script/` when they are intentional source changes.
- Keep local-only dependency workspace overrides untracked:
  - `script/config/dependency_workspace.local.sh`

## Code notes

- The local installed `GTSAM` layout in this workspace expects:
  - `<gtsam/nonlinear/IncrementalFixedLagSmoother.h>`
  instead of the old `gtsam_unstable` include path.
- Keep edits focused; do not revert unrelated local changes in this workspace.
