# Hesai JT128 on Unitree A2

This workspace keeps the Hesai ROS driver outside `super_odometry`. The
integration path is:

```text
JT128 UDP packets
  -> HesaiLidar_ROS_2.0 /hesai/jt128/points + /hesai/jt128/imu
  -> super_odometry hesai_to_superodom_node
  -> /hesai/jt128/points_superodom
  -> feature_extraction_node + laser_mapping_node + imu_preintegration_node
  -> odom_frame_transform_node /body_odometry + map->base_link TF
```

The JT128 SuperOdom configs use `sensor: "jt128"`. The adapter publishes
`PointcloudXYZITR`, so feature extraction can reuse the same generic spinning
multi-line PointCloud2 parsing path internally without treating JT128 as a
Velodyne model or relying on Velodyne manuals.

## JT128 Input Contract

SuperOdom does not consume raw JT128 UDP packets. The expected adapter input is
an already decoded `sensor_msgs/msg/PointCloud2` with:

- `x`, `y`, `z`: Cartesian points in the raw JT128 LiDAR frame
- `intensity` or `reflectivity`
- `ring`, `laser_id`, `channel`, or `line`: physical channel id
- `timestamp`, `time`, `t`, `offset_time`, `timestamp_ns`, or `timeSecond`:
  per-point time

The recorded Unitree A2 JT128 bags satisfy the point-cloud contract directly:
the front and rear clouds contain `x/y/z/intensity/ring/timestamp`,
`ring=0..127`, and a per-frame timestamp span of about `0.1 s`. Therefore the
normal SuperOdom adapter path does not use the JT128 manual's vertical-angle
table and does not recompute `x/y/z`. That table belongs to raw packet decoding
or to a last-resort diagnostic path if a future bridge publishes no physical
channel field.

The adapter's job is intentionally thin:

- convert `PointCloud2` to SuperOdom's `PointcloudXYZITR` layout
- convert absolute per-point timestamps to relative seconds from the cloud
  header stamp
- preserve the physical `ring` field
- optionally drop very near points in the raw LiDAR frame only when
  `enable_min_range_filter: true` and adapter `min_range` is set above zero
- reject clouds that lack required point time or ring fields
- log the first cloud's fields, relative time span, ring range, unique ring
  count, and skipped point counts

For Unitree A2, the committed JT128 configs keep adapter-level filtering
explicitly disabled with `enable_min_range_filter: false`. `min_range` remains
`0.0` and is ignored unless the flag is true. This preserves point order,
per-scan timing, and the input sequence used by SuperOdom's
`filter_point_size` sampling. Use
`feature_extraction_node.min_range` for the normal LIO near-range feature gate.
Adapter-level range filtering should be reserved for explicit diagnostics or
for clearly invalid raw points.

Do not use `i % scan_line` as the normal ring source. The committed configs keep
`allow_ring_fallback: false`; enable a fallback only for a temporary diagnostic
run where the upstream bridge is known to omit channel data.

For controlled debugging, use the raw front JT128 point cloud and front JT128
IMU. The A2 fused cloud can be launched explicitly after its per-point timing
and motion compensation policy are known; do not use it as the default stability
baseline.

The A2 LiDAR service document publishes structural-design extrinsics. The
default A2 adapter output remains in the raw front JT128 frame
(`front_hesai_jt128`). Do not hard-code a z-up adapter extrinsic: the JT128
feature-extraction config waits for IMU initialization, then applies
SuperOdom's IMU-derived gravity alignment to JT128 points and labels those
feature clouds as `front_hesai_jt128_gravity`. `imu_preintegration_node`
applies the same alignment to IMU measurements. SuperOdom odometry is therefore
for the gravity-aligned virtual frame, not the raw adapter frame. In RViz that
virtual frame appears as X/red left, Y/green back, Z/blue up. The planner-facing
`base_link` odometry converts this virtual frame to ROS body axes:

```text
front_lidar_gravity -> base_link:
  t = [0.33767, 0.0, 0.08134]
  R_body_from_gravity = [0,-1,0; 1,0,0; 0,0,1]
  xyzrpy = [0.33767, 0.0, 0.08134, 0.0, 0.0, 1.57079632679]

rear_lidar -> front_lidar:
  t = [0.0, 0.00599, -0.61764]
  R = [-1,0,0; 0,1,0; 0,0,-1]
```

For JT128 localization, RViz `2D Pose Estimate` is configured as a robot-body
initial pose, not as a LiDAR-frame pose:

```yaml
laser_mapping_node:
  rviz_initial_pose_frame: "base_link"
  rviz_initial_pose_lidar_to_body_xyzrpy: [0.33767, 0.0, 0.08134, 0.0, 0.0, 1.57079632679]
```

The arrow you draw in RViz therefore means `base_link` +X, i.e. robot forward,
in the `map` frame. `laser_mapping_node` converts that body pose to the
internal `front_hesai_jt128_gravity` pose before resetting SuperOdom's
localization state. Without this conversion the same RViz arrow would be
interpreted as the virtual JT128 frame's +X axis, which points robot-left.

The same document states that the JT128 internal IMU and LiDAR have no
physical rotation or translation offset. The committed SuperOdom calibration is
therefore identity; gravity alignment is a runtime IMU initialization step, not
a physical LiDAR-to-IMU extrinsic.

The JT128 configs expose `imu_acc_scale` and `imu_gyr_scale` in both
`feature_extraction_node` and `imu_preintegration_node`. The current A2 JT128
bags store acceleration in g and angular velocity in deg/s, while GTSAM IMU
preintegration expects m/s^2 and rad/s. The committed JT128 defaults therefore
use:

```yaml
imu_acc_scale: 9.80665
imu_gyr_scale: 0.017453292519943295
```

Applying only the acceleration scale is not enough; the gyro also has to be
converted or preintegration over-rotates the state and the IMU graph drives the
bias estimate into reset.

The JT128 config also exposes IMU-preintegration failure thresholds. Defaults
in code remain compatible with the original SuperOdom behavior, and the JT128
yaml keeps those thresholds explicit instead of widening them. This keeps reset
diagnostics meaningful: a clean run should come from consistent JT128 units and
frames, not from a relaxed failure gate.

```yaml
failure_velocity_threshold: 30.0
failure_acc_bias_threshold: 2.0
failure_gyr_bias_threshold: 1.0
failure_startup_acc_bias_threshold: 2.0
failure_startup_key_count: 0
```

To repeat that experiment without editing the committed config:

```bash
JT128_IMU_ACC_SCALE=9.8105 bash script/run_superodom.sh mapping-bag --lidar jt128 --skip-build --no-rviz krail
```

To override the gyro scale as well:

```bash
JT128_IMU_GYR_SCALE=1.0 bash script/run_superodom.sh mapping-bag --lidar jt128 --skip-build --no-rviz krail
```

## Install Hesai Driver

The upstream driver is `HesaiTechnology/HesaiLidar_ROS_2.0`, based on
`HesaiLidar_SDK_2.0`. It supports JT128 and ROS 2 Humble.

```bash
sudo apt update
sudo apt install -y libboost-all-dev libyaml-cpp-dev

mkdir -p ~/hesai_ws/src
cd ~/hesai_ws/src
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git

cd ~/hesai_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

This package installs ready-to-use Hesai driver configs:

- [jt128_driver.yaml](../super_odometry/config/hesai_driver/jt128_driver.yaml)
- [jt128_dual_driver.yaml](../super_odometry/config/hesai_driver/jt128_dual_driver.yaml)

Check these hardware values before running:

- `device_ip_address`
- `udp_port`
- `ptc_port`
- host network interface IP and route
- `correction_file_path` / `firetimes_path` if PTC cannot fetch calibration

For a single JT128:

```bash
source /opt/ros/humble/setup.bash
source ~/hesai_ws/install/local_setup.bash
source install/setup.bash
ros2 launch super_odometry hesai_jt128_driver.launch.py
```

Expected topics from the provided config:

- `/hesai/jt128/points`
- `/hesai/jt128/imu`
- `/hesai/jt128/packets_loss`

For front/rear JT128s:

```bash
source /opt/ros/humble/setup.bash
source ~/hesai_ws/install/local_setup.bash
source install/setup.bash
ros2 launch super_odometry hesai_jt128_dual_driver.launch.py
```

Expected dual topics from the provided config:

- `/front/hesai/jt128/points`
- `/front/hesai/jt128/imu`
- `/rear/hesai/jt128/points`
- `/rear/hesai/jt128/imu`

## Run SuperOdom

For the recorded bags under `$ROBOCUP_WS/data/JT128bag`, use the
unified one-command runner. It starts SuperOdom, RViz, and `ros2 bag play`:

```bash
cd $ROBOCUP_WS/SuperOdom
bash script/run_superodom.sh mapping-bag --lidar jt128 ramp
```

The JT128 mode uses `super_odometry/rviz_jt128_debug.rviz` by default. That RViz
view uses `map` as the fixed frame and keeps the raw adapter cloud disabled by
default, so it shows the gravity-aligned mapping products without mixing them
with pre-initialization raw LiDAR-frame clouds. Enable `Raw JT128 Adapter Cloud`
only when you intentionally want to inspect `/hesai/jt128/points_superodom` in
the raw `front_hesai_jt128` frame.

### Mid360-style JT128 experiment

Do not set JT128 to `sensor: "livox"` unless the adapter publishes
`livox_ros_driver2::msg::CustomMsg`; SuperOdom's Livox branch subscribes that
message type and will not consume the JT128 `PointCloud2` adapter output.

For an A/B experiment, keep `sensor: "jt128"` and run the JT128 adapter with
Mid360-style feature/mapping parameters:

```bash
bash script/run_superodom.sh mapping-bag --lidar jt128-mid360 krail jt128_mid360_krail
```

This uses [hesai_jt128_mid360_params.yaml](../super_odometry/config/hesai_jt128_mid360_params.yaml):
raw JT128 adapter frame, JT128 calibration, `PointCloud2` input, physical
`scan_line: 128`, adapter `enable_min_range_filter: false`, adapter
`min_range: 0.0`,
`feature_extraction_node.min_range: 0.2`, `mapping_plane_resolution: 0.1`, and
`max_surface_features: 4000`.

The first run builds into isolated JT128 directories so stale colcon caches in
`build/`, `install/`, or `log/` do not affect this workflow:

- `.superodom_build/jt128`
- `.superodom_install/jt128`
- `.superodom_log/jt128`

Available bag names:

- `ramp`
- `krail`
- `krail1`

Use `--rear` to replay the rear JT128 instead of the default front JT128.

```bash
bash script/run_superodom.sh mapping-bag --lidar jt128 --skip-build --rear ramp
```

### LIO diagnostics

The JT128 configs enable lightweight LIO diagnostics with:

```yaml
laser_mapping_node:
  lio_diagnostics_enabled: true
  lio_diagnostics_period: 10
```

The launch log contains `[LIO_DIAG]` lines. These are intended for comparing
bags such as `krail` and `krail1` before tuning parameters:

- `motion`: whether the scan was accepted, rejected as `too_small`, rejected as
  `too_large`, rejected for invalid dt, or skipped for not enough local-map
  features
- `surf_scan/surf_map/surf_sampled`: surface features from the current scan,
  local-map surface features, and the number actually evaluated after sampling
- `plane_ok` and `plane_rej`: scan-to-map plane correspondence success count
  and rejection causes (`far`, `pca`, `mse`, etc.)
- `obs_xyz` and `obs_rpy`: observability distribution from accepted plane
  correspondences

The Hesai adapter also logs `enable_min_range_filter`, `min_range`, and
`skipped_range` in the first-cloud contract line. In the default configuration,
`enable_min_range_filter` should be `false`, `min_range` should be `0.000`, and
`skipped_range` should stay `0`; use that line to confirm adapter-level
filtering has not been enabled accidentally.

The same fields are also published in `/super_odometry_stats`, including line
and plane rejection histograms, motion status, sampling rate, and observability
counts. A useful terminal view is:

```bash
source /opt/ros/humble/setup.bash
source .superodom_install/jt128/setup.bash
python3 script/monitor_superodom_stats_live.py --log-file /tmp/superodom_stats.txt
```

### JT128 LIO tuning knobs

The JT128 front-end config exposes the plane correspondence gates used by
`laser_mapping_node`:

```yaml
laser_mapping_node:
  mapping_plane_resolution: 0.4
  plane_neighbor_distance_factor: 4.0
  plane_pca_min_ratio: 0.05
  plane_max_point_distance_factor: 0.75
  plane_loss_distance_factor: 3.0
```

These values are the current conservative JT128 candidate from `krail` and
`krail1` bag sweeps. Compared with the original default
`mapping_plane_resolution: 0.2`, `plane_neighbor_distance_factor: 3.0`,
`plane_pca_min_ratio: 0.1`, and `plane_max_point_distance_factor: 0.5`, this
candidate primarily reduces `far` and `pca` plane correspondence rejections.
Keep `plane_loss_distance_factor` at `3.0` unless residual/outlier behavior is
being tested explicitly.

For z-axis drift analysis, `[LIO_DIAG]` also includes `pose_xyz` and
`pose_rpy`. In one `krail` sweep, `mapping_skip_frame: 2` and
`use_imu_roll_pitch: true` both reduced final z drift, which points to local-map
pollution and roll/pitch-to-z coupling while vertical constraints are weak.
Neither is a default yet: repeated `mapping_skip_frame: 2` runs were not stable,
and `use_imu_roll_pitch: true` hurt `ramp` correspondence quality and final z.

Because ROS bag replay startup and node scheduling can shift which early frames
seed the local map, compare parameter sets over repeated runs or slower replay
before treating a single best `plane_ok` value as final.

Build and source this workspace, then launch:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch super_odometry hesai_jt128_superodom.launch.py
```

SuperOdom's internal odometry remains in the front JT128 frame:

- `/laser_odometry`
- `/imu_odometry`

The planner-facing output is published separately:

- `/body_odometry`
- TF: `map -> base_link`

For Unitree A2 built-in fused cloud mode, explicitly set the fused point cloud
topic from the A2 SDK guide and the primary IMU topic selected for SuperOdom:

```bash
ros2 launch super_odometry hesai_jt128_superodom.launch.py \
  input_point_cloud_topic:=/replace/with/a2_fused_point_cloud \
  imu_topic:=/replace/with/primary_imu
```

This mode does not launch `dual_lidar_fusion_node`. Use it only after checking
that the fused cloud still carries valid per-point timestamps and physical ring
or channel fields.

The A2 document lists these raw DDS channel names:

- fused cloud: `rt/unitree/slam_lidar/points`, origin at the front LiDAR frame
- front cloud: `rt/unitree/slam_lidar/points1`, origin at the front LiDAR frame
- front IMU: `rt/unitree/slam_lidar/imu1`, origin at the front LiDAR frame
- rear cloud: `rt/unitree/slam_lidar/points2`, origin at the front LiDAR frame
- rear IMU: `rt/unitree/slam_lidar/imu2`, origin at the rear LiDAR frame

If these channels are bridged into ROS 2 with names that omit the `rt/` DDS
prefix, pass the bridged ROS topic names to the launch arguments.

For front/rear JT128 pre-fusion:

```bash
ros2 launch super_odometry hesai_jt128_dual_superodom.launch.py
```

In this fallback path, raw rear points are transformed into the front JT128
frame before SuperOdom sees them. The body pose is still produced only by
`odom_frame_transform_node`.

The dual mode still uses one primary IMU:

```yaml
imu_topic: "/front/hesai/jt128/imu"
```

Change this only after choosing the IMU that should define the SuperOdom body
state.

## Calibration Required Before Real Use

The committed A2 body transform values are Unitree structural-design
extrinsics, not an on-robot hand-eye calibration. Replace or refine them before
final planner integration:

- front JT128 frame to `base_link`:
  - `odom_frame_transform_node.lidar_to_body_xyzrpy`
- raw rear JT128 frame to front JT128 frame, only for the front/rear fusion fallback:
  - `dual_lidar_fusion_node.rear_lidar_to_output_xyzrpy`
- front JT128 LiDAR to front JT128 IMU:
  - `config/hesai/jt128_calibration.yaml`, identity by default

SuperOdom currently models one primary IMU. A second JT128 IMU should be treated
as an external odometry/prior source only after a separate estimator is added.

When using the A2 fused point cloud, the document says its origin is the front
LiDAR frame. The default adapter leaves it in that frame, and only the final
planner odometry is transformed to `base_link`.

## References

- Unitree A2 SDK guide: <https://support.unitree.com/home/zh/A2_SDK_Development_Guide/about_a2>
- Unitree A2 LiDAR service interface: <https://support.unitree.com/home/zh/A2_SDK_Development_Guide/lidar_service_interface>
- Hesai ROS 2 driver: <https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0>
