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
  - MID360: `bash script/run_superodom.sh mapping-bag --lidar mid360 data_bag/ramp1 smoke_ramp1`
  - JT128: `bash script/run_superodom.sh mapping-bag --lidar jt128 krail smoke_krail`
- Live odometry / mapping:
  - MID360: `bash script/run_superodom.sh mapping-live --lidar mid360 live_mid360`
  - JT128: `bash script/run_superodom.sh mapping-live --lidar jt128 live_jt128`
- Offline prior-map localization / SuperLoc-style replay:
  - `bash script/run_superodom.sh localization-bag --lidar mid360 data_bag/ramp1 /abs/path/map.pcd loc_smoke`
- Live prior-map localization / SuperLoc-style run:
  - `bash script/run_superodom.sh localization-live --lidar mid360 /abs/path/map.pcd live_loc`
- RViz is enabled by default when `DISPLAY` is set. Use `--no-rviz` only for
  headless smoke tests.
- Local Livox bag config lives at:
  - `script/config/livox_mid360_local_bag.yaml`
- Local Livox localization config lives at:
  - `script/config/livox_mid360_localization.yaml`
- That config assumes:
  - lidar topic: `/livox/lidar`
  - imu topic: `/livox/imu`
- Unitree A2 / JT128 bag replay with RViz:
  - `bash script/run_superodom.sh mapping-bag --lidar jt128 ramp`
  - Available bag names under `$ROBOCUP_WS/data/JT128bag`: `ramp`, `krail`, `krail1`
  - JT128 runs build into `.superodom_build/jt128` and `.superodom_install/jt128`.

## Unitree A2 / Hesai JT128 notes

- Keep the JT128 adapter output in the raw `front_hesai_jt128` frame. Do not
  hard-code a z-up adapter extrinsic; `feature_extraction_node` applies
  SuperOdom's IMU-derived gravity alignment internally after IMU init. JT128
  configs should wait for IMU init before accepting LiDAR frames and should
  publish feature clouds in `front_hesai_jt128_gravity`.
- Use `sensor: "jt128"` for JT128 configs. Internally this reuses the generic
  spinning multi-line `PointcloudXYZITR` parsing path after the Hesai adapter;
  do not label JT128 as `velodyne`.
- Treat the JT128 adapter as a strict contract bridge, not a geometry decoder:
  source PointCloud2 must provide decoded `x/y/z`, physical `ring`/channel, and
  per-point time. The adapter converts absolute point timestamps to relative
  scan time and rejects missing time/ring fields by default.
- Keep `hesai_to_superodom_node.enable_min_range_filter: false` in A2 JT128
  configs unless a run explicitly tests adapter-level raw-point filtering.
  `min_range` is ignored unless this flag is true. The adapter should preserve
  point order and scan timing for SuperOdom; use
  `feature_extraction_node.min_range` for LIO feature gating.
- Do not use the JT128 manual vertical-angle table in the normal SuperOdom
  adapter path. It is only relevant for raw packet decoding or a diagnostic
  fallback when a future bridge omits channel data.
- JT128 configs enable lightweight LIO diagnostics by default. Inspect
  `[LIO_DIAG]` lines in the launch log and `/super_odometry_stats` before
  changing algorithm parameters. Key fields are motion rejection reason, plane
  match success/rejection histogram, surface sample count, and observability
  counts for xyz/rpy.
- Keep `laser_mapping_node.auto_voxel_size: false` in JT128 configs when
  evaluating tuned `mapping_plane_resolution` values. The legacy auto voxel
  heuristic can override the configured plane resolution from scene range and
  make JT128 LIO sweeps non-deterministic.
- Current JT128 LIO front-end candidate values are
  `mapping_plane_resolution: 0.4`, `plane_neighbor_distance_factor: 4.0`,
  `plane_pca_min_ratio: 0.05`, and
  `plane_max_point_distance_factor: 0.75`. These are tuning values from
  `krail`/`krail1` sweeps, not a substitute for hand-eye calibration or broader
  scenario validation.
- Do not make `mapping_skip_frame: 2` the JT128 default from a single successful
  `krail` run. It reduced z drift once but was not stable on repeat; keep it as
  an explicit z-drift experiment.
- `use_imu_roll_pitch` is now wired to the yaml parameter but should remain
  false by default for JT128. It reduced `krail` z drift in one sweep but hurt
  `ramp`; use it only as an explicit experiment.
- Keep the JT128 `imu_acc_scale`, `imu_gyr_scale`, and IMU-preintegration
  failure thresholds explicit in yaml. Code defaults preserve original
  SuperOdom behavior; JT128 configs keep `failure_acc_bias_threshold: 2.0`
  and do not use a startup-only wider accelerometer-bias threshold.
- Publish planner-facing robot body pose with `odom_frame_transform_node`
  instead. It converts `/imu_odometry` or `/laser_odometry` into `/body_odometry`
  and can publish `map -> base_link`.
- SuperOdom JT128 odometry is for the gravity-aligned virtual LiDAR frame
  (`front_hesai_jt128_gravity`), not the raw adapter frame. That virtual frame
  appears in RViz as X/red left, Y/green back, Z/blue up. Use a yaw-only
  `lidar_to_body_xyzrpy` rotation from this virtual frame to ROS `base_link`
  (X front, Y left, Z up), while keeping the Unitree structural translation.
- JT128 localization should interpret RViz `/initialpose` as the robot
  `base_link` pose in `map`, then convert it to the internal
  `front_hesai_jt128_gravity` pose with the same structural transform. Keep
  `laser_mapping_node.rviz_initial_pose_frame: "base_link"` and
  `rviz_initial_pose_lidar_to_body_xyzrpy` aligned with the final body-odom
  transform unless calibrated values replace the design extrinsic.
- Unitree's A2 structural-design extrinsic for front JT128 to `base_link` is:
  - translation `[0.33767, 0.0, 0.08134]`
  - rotation `[0,0,1; 1,0,0; 0,1,0]`
- Unitree's A2 structural-design extrinsic for rear JT128 to front JT128 is:
  - translation `[0.0, 0.00599, -0.61764]`
  - rotation `[-1,0,0; 0,1,0; 0,0,-1]`
- These are design values, not hand-eye calibrated values. Prefer calibration
  before final localization/planner evaluation.
- The A2 LiDAR service reports the fused cloud and front cloud origins in the
  front LiDAR frame. Use the raw front cloud plus front IMU as the default
  SuperOdom stability baseline; use fused cloud only when explicitly selected
  and after confirming its per-point timing and motion compensation behavior.
- JT128 configs expose `imu_acc_scale` and `imu_gyr_scale` for feature
  extraction and IMU preintegration. Current A2 JT128 bags use g and deg/s, so
  keep defaults at `9.80665` and `0.017453292519943295` unless verified
  otherwise.

## Validation

- For code changes, prefer a short bag smoke test over blind edits.
- Smallest useful replay in this workspace is usually:
  - `data_bag/ramp1`
- If GUI is unavailable, use:
  - `bash script/run_superodom.sh mapping-bag --lidar mid360 --skip-build --no-rviz --no-keep-rviz data_bag/ramp1 smoke_ramp1`

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
