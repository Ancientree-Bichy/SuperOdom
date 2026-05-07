# SuperOdom Localization Task Guide

This document is a task-oriented guide for using this repository for:

- offline mapping from recorded ROS 2 bags
- prior-map localization
- live robot localization with RViz-assisted initial pose selection

It intentionally does not modify or replace the upstream `readme.md`.

## Scope

This repository currently supports a practical localization workflow built around:

- `laser_mapping_node` in `localization_mode`
- a prior map in `PCD` format
- optional RViz `2D Pose Estimate` initialization

The current public implementation is still a shared `SuperOdom` / `SuperLoc` codebase.
In practice:

- mapping mode builds a local-map-driven map from live or bagged LiDAR + IMU data
- localization mode loads a prior map and localizes against it

## Dependency Model

This repository does **not** require a project called FAST_LIO to run.

Some scripts may need an external ROS workspace that already provides dependencies such as:

- `livox_ros_driver2`
- non-system `GTSAM`
- non-system `Sophus`

Use the neutral environment variable:

```bash
DEPENDENCY_WS_ROOT=/path/to/that/workspace
```

The scripts still accept the legacy `FAST_LIO2_LOC_ROOT` name as a compatibility fallback,
but new usage should prefer `DEPENDENCY_WS_ROOT`.

## Build

Preferred build entrypoint:

```bash
bash script/build_superodom_local.sh
```

If dependencies come from another ROS workspace:

```bash
DEPENDENCY_WS_ROOT=/path/to/dependency_ws bash script/build_superodom_local.sh
```

## Offline Mapping

Single bag:

```bash
bash script/run_superodom.sh mapping-bag --lidar mid360 --skip-build data_bag/K-rail1 run_name
```

The same entrypoint also supports Unitree A2 / JT128 bags:

```bash
bash script/run_superodom.sh mapping-bag --lidar jt128 krail jt128_krail
```

Batch mapping across every bag under `data_bag/`:

```bash
bash script/run_superodom_all_data_bags.sh --skip-build --batch-name quality_compare_v1
```

Batch outputs are now grouped under:

```text
mapping_output/<batch_name>/<bag_name>/
```

This makes it easier to compare map quality across multiple runs.

## Prior Maps

Available prior maps in this workspace are typically stored under:

```text
map_by_scanner/
```

Current localization scripts expect the prior map in `PCD` format.

Examples:

- `map_by_scanner/k-rail.pcd`
- `map_by_scanner/ramp.pcd`

## Pose File Naming

Manual RViz initial poses are saved next to the prior map using the map stem:

- `map_by_scanner/k-rail.pcd` -> `map_by_scanner/k-rail.start_pose.txt`
- `map_by_scanner/ramp.pcd` -> `map_by_scanner/ramp.start_pose.txt`

This avoids a single shared `start_pose.txt` file for all maps.

## Offline Localization

Bag replay + prior map localization:

```bash
bash script/run_superodom.sh localization-bag --lidar mid360 --skip-build \
  data_bag/K-rail1 \
  /abs/path/to/prior_map.pcd \
  robocup_loc
```

This script:

- launches localization mode
- loads the prior map
- opens RViz by default
- waits for `/initialpose`
- then starts `ros2 bag play`

## Live Robot Localization

For real hardware, use the same unified runner without a bag path:

```bash
bash script/run_superodom.sh localization-live --lidar mid360 --skip-build \
  /abs/path/to/prior_map.pcd \
  robocup_live_loc
```

This script does **not** play a bag.
It only launches the nodes and waits for live robot topics.

## Required Live Topics

With `--lidar mid360`, the robot must publish:

- lidar topic: `/livox/lidar`
  type: `livox_ros_driver2/msg/CustomMsg`
- imu topic: `/livox/imu`
  type: `sensor_msgs/msg/Imu`

With `--lidar jt128`, the default live Unitree A2 front topics are:

- lidar topic: `/rt/unitree/slam_lidar/points1`
  type: `sensor_msgs/msg/PointCloud2`
- imu topic: `/rt/unitree/slam_lidar/imu1`
  type: `sensor_msgs/msg/Imu`

If your robot uses different names, pass explicit topics:

```bash
bash script/run_superodom.sh mapping-live --lidar jt128 \
  --point-topic /your/points \
  --imu-topic /your/imu \
  live_custom_topics
```

For older config-based overrides, create another config file and override:

- `laser_topic`
- `imu_topic`

via `--config-file ...`.

## RViz Initial Pose Behavior

Current localization config is:

- `use_rviz_initial_pose: true`
- `rviz_initial_pose_xy_yaw_only: true`

This means RViz `2D Pose Estimate` is interpreted as:

- use `x`
- use `y`
- use `yaw`
- preserve configured `z`
- preserve configured `roll`
- preserve configured `pitch`

That behavior is intentional for prior maps that are already approximately `z-up`.

## Recommended Startup Sequence

For prior-map localization:

1. Launch localization mode.
2. Confirm the prior map is visible in RViz.
3. Use RViz `2D Pose Estimate` to place the robot start in the prior map.
4. Let the script proceed to playback, or start moving on the real robot.

## Important Frame Semantics

The current codebase estimates the sensor pose directly:

- `T_w_lidar` is effectively `T_map_sensor`

There is no separate public `prior_map` frame maintained at runtime.
The loaded prior map is currently treated as already expressed in `WORLD_FRAME`.

## Current Limitations

- Prior maps are loaded from `PCD`, not `PLY`.
- The public codebase shares one implementation between mapping and localization.
- Some advanced `SuperLoc` paper concepts, especially external-prior fusion beyond the currently exposed branches, are only partially wired in the public code.

## Useful Scripts

- Build:
  - `script/build_superodom_local.sh`
- Unified runtime entrypoint:
  - `script/run_superodom.sh`
- Batch mapping:
  - `script/run_superodom_all_data_bags.sh`
- Common environment helpers:
  - `script/common_env.sh`

## Unified Modes

The preferred runtime entrypoint is:

```bash
bash script/run_superodom.sh <mode> ...
```

Supported modes:

- `mapping-bag`
  Example:
  `bash script/run_superodom.sh mapping-bag --lidar mid360 --skip-build data_bag/ramp1 smoke_ramp1`
- `mapping-live`
  Example:
  `bash script/run_superodom.sh mapping-live --lidar jt128 --skip-build live_map_run`
- `localization-bag`
  Example:
  `bash script/run_superodom.sh localization-bag --lidar mid360 --skip-build data_bag/K-rail1 /abs/path/map.pcd loc_replay`
- `localization-live`
  Example:
  `bash script/run_superodom.sh localization-live --lidar mid360 --skip-build /abs/path/map.pcd loc_live`

## Local Dependency Workspace

For local development, scripts may automatically load:

- `script/config/dependency_workspace.local.sh`

This file is intentionally gitignored and can point to a machine-specific external workspace that provides dependencies such as:

- `livox_ros_driver2`
- `GTSAM`
- `Sophus`

The tracked template is:

- `script/config/dependency_workspace.example.sh`
