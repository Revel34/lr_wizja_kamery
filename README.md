# lr_wizja_kamery: depth-camera obstacle detection for the rover

This project replaces the lidar with 3 Intel RealSense D455 depth cameras (86° HFOV, one per side). It fuses their point clouds into a single cloud in `base_footprint` (ground level, the same frame Nav2 uses on rover 3.0) and detects obstacles relative to the locally estimated ground, so slopes and hills are not reported as obstacles. It is based on the rover 2.2 code (`kick-autonomy`, `rover_perception/height_filter_node.py`), extended to 3 cameras and with the fixed height slice replaced by ground-relative analysis.

ROS 2 Humble · Ubuntu 22.04 · C++17 / PCL · target computer: Geekom (x86_64) · rover: Rover 3.0 (Aut-Main `rover30`).

```
 D455 front ─┐  depth image from the main stack's driver (shared camera)
 D455 left  ─┼─► cloud_fusion ──► /wizja/fused_points ──► obstacle_detector ─┬─► /obstacles/points    (PointCloud2)
 D455 right ─┘   (TF to base_footprint, (base_footprint)   (ground plane +
   /camera_left|right/depth/color/points (our drivers)     ├─► /obstacles/grid      (OccupancyGrid)
                  range clip, self-                          local ground grid,  ├─► /obstacles/clusters  (wizja_msgs/ObstacleArray)
                  filter, voxel)                             height above it)    ├─► /obstacles/markers   (MarkerArray, RViz)
                                                                                 └─► /wizja/ground_points (debug)
```

The two perception nodes are composable components in one `component_container_mt`, so the fused cloud is handed to the detector without being serialized.

## Packages

| Package | Contents |
|---|---|
| `wizja_msgs` | `Obstacle.msg`, `ObstacleArray.msg` |
| `wizja_perception` | `CloudFusionNode`, `ObstacleDetectorNode`, ground segmentation library, unit tests, `config/perception.yaml` |
| `wizja_description` | Rover 3.0 model (geometry, joints and simplified meshes copied from Aut-Main `rover30_description`) plus the camera post and the 3 D455s. **The camera positions are defined here** (`urdf/wizja_rover.urdf.xacro`, "CAMERA POSITIONS" block) |
| `wizja_bringup` | launch files, `config/cameras.yaml` (which cameras exist: serials, driver/external, topics, driver settings), RViz config, the `fake_cameras.py` simulator and the `coverage_map.py` tool |

## Installation on the Geekom

```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-librealsense2-tools \
                 ros-humble-pcl-conversions ros-humble-tf2-eigen libpcl-dev
# udev rules so the cameras are accessible without root:
sudo curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
     -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# copy the workspace (sources only), e.g. from a laptop:
rsync -av --exclude build --exclude install --exclude log lr_wizja_kamery/ geekom:~/lr_wizja_kamery/

cd ~/lr_wizja_kamery
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y     # optional, installs anything still missing
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Running

**On the rover (3× D455):**

The serial numbers are already set in `cameras.yaml`:

| Camera | Serial | Looks | Driver |
|---|---|---|---|
| `front` (Center) | `254122303704` | forward, level | **the main stack's** (shared camera, see below) |
| `left` | `253822300711` | left, yaw +80° | ours |
| `right` | `253822300777` | right, yaw −80° | ours |

They are the 12-digit serials from the labels (`253822300711000000N14281-201` etc.). Check them once with `rs-enumerate-devices -s`; if one doesn't match, the driver for that camera won't find it.

**The front camera is shared.** It is the D455 the rover already had, and the main stack's driver (`rover30_depth_camera/launch/rs/rs.launch.py`, node `realsense_camera`) owns it: besides our obstacle detection it feeds `rover30_vision`, RTAB-Map visual odometry (`/odom/visual` → EKF), `/depth/points` and `depth_to_laserscan`. Only one process can open a RealSense, so this project starts **no driver** for it (`source: external` in `cameras.yaml`) and reads its depth image `/camera/realsense_camera/depth/image_rect_raw` + `camera_info` instead. The main stack's RealSense launch must therefore be running (e.g. `depth_camera_irl.launch.py use_rs:=true` / the `rover-depth-camera` service).

```bash
ros2 launch wizja_bringup wizja.launch.py
# add use_rviz:=true to open RViz (if the Geekom has a screen)
# override a serial without editing the file: serial_front:=<serial>
```

Launch arguments: `cameras_config`, `serial_<name>`, `rover_model`, `use_fastdds_udp_profile`, `use_rviz`. You can also start the parts separately with `cameras.launch.py` (drivers + robot model) and `perception.launch.py` (fusion + detection; it also takes `perception_params`, `log_level` and `use_sim_time`).

**Rover model and TF (`rover_model`):**

| Value | Publishes | Use it when |
|---|---|---|
| `cameras_only` (default in `wizja.launch.py`) | `base_link` → camera post + side cameras (`camera_left_link`, `camera_right_link`) | the main Aut-Main stack is running and its `robot_state_publisher` already publishes the rover (`base_footprint` → `base_link`, wheels) and the front camera (`camera_link`) |
| `full` | the whole rover30 model + all 3 cameras (the front one under its main-stack names `camera_link` …), root `base_footprint` | nothing else publishes the rover (laptop, simulator) |
| `none` | nothing | the rover's own URDF already contains these cameras |

Our `robot_state_publisher` runs in namespace `wizja` (`/wizja/robot_description`), so it never clashes with the main stack's `/robot_description`. The realsense driver adds `camera_<name>_link` → optical frames itself.

**View the model only (no cameras needed):**

```bash
ros2 launch wizja_bringup view_model.launch.py
```

RViz shows the rover with the camera post and the 3 cameras ("Rover + cameras" display). The "Main stack model" display (disabled by default) shows the main stack's `/robot_description` when it runs.

**Without hardware (synthetic cameras):**

```bash
ros2 launch wizja_bringup sim_test.launch.py            # use_rviz:=false without a screen
```

`fake_cameras.py` ray-casts each camera, at its pose in the rover URDF (D455 FOV, 320×240 after decimation, depth noise ∝ z²), against a test scene: two boxes, a pole, a rock on a hill, a 14° hill and an 8 cm bump. The front camera is simulated like the main stack's driver (a 640×480 16UC1 depth image + CameraInfo on the `/camera/realsense_camera/...` topics), so the depth-image input is tested too; the side cameras publish point clouds like our drivers. The rover's own body (deck, rockers, wheels, post, mast) is part of the scene; about 10% of each side camera's pixels see it, which tests the self-filter. The detector should report exactly 4 obstacles; the hill, the bump and the rover itself must not be among them.

**Driving the simulated rover with the keyboard:**

```bash
ros2 launch wizja_bringup sim_drive.launch.py                    # terminal 1: sim + perception + RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard             # terminal 2: keep this one focused
```

| Key | Action |
|---|---|
| `i` / `,` | forward / back |
| `j` / `l` | turn left / right on the spot |
| `Shift+J` / `Shift+L` | drive sideways (4-wheel steering) |
| `u` `o` `m` `.` | arcs |
| `k` | stop |
| `q` / `z` | faster / slower (the sim caps at 1 m/s, 1 rad/s) |

RViz (`wizja_drive.rviz`) uses `odom` as its fixed frame and the view follows the rover. It shows:
- the rover model with its **steering wheels** turning and spinning (4-wheel independent steering kinematics, like Rover 3.0);
- the ground and obstacle points and the obstacle boxes;
- the true scene objects as translucent markers (`/sim/scene`).

There are extra objects 9–10 m from the start (a big box and a rock ahead, a low wall at the far left, a pole and a box behind), so there is something to drive to. Without RViz: `use_rviz:=false`.

The simulation is **kinematic**: no physics and no collisions (the rover drives through objects), and the rover stays at z = 0, so it does not climb the hill. Commands time out after 0.5 s. In drive mode the cameras are re-rendered from the current pose every frame at a lower resolution (`drive_resolution_scale`, 0.5); fused data runs at ~14 Hz on the laptop. Topics added in drive mode: `/cmd_vel` (input), `/odom`, TF `odom` → `base_footprint`, `/wizja/joint_states`, `/sim/scene`.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera_<name>/depth/color/points` | PointCloud2 | input from our side-camera drivers (BEST_EFFORT) |
| `/camera/realsense_camera/depth/image_rect_raw` + `.../depth/camera_info` | Image + CameraInfo | input from the shared front camera (main stack's driver) |
| `/wizja/fused_points` | PointCloud2 | fused cloud in `base_footprint`, rover body removed, voxel 5 cm |
| `/obstacles/points` | PointCloud2 | obstacle points (same topic as rover 2.2, so it works directly as an observation source for the Nav2 costmap) |
| `/obstacles/grid` | OccupancyGrid | 16×16 m, 10 cm, centred on `base_footprint`: 100 obstacle, 0 ground seen, −1 unknown |
| `/wizja/robot_description` | String | the model published by our `robot_state_publisher` |
| `/obstacles/clusters` | wizja_msgs/ObstacleArray | obstacles: centroid, bbox, height above ground, distance, sorted from nearest |
| `/obstacles/markers` | MarkerArray | boxes and labels for RViz |
| `/wizja/ground_points` | PointCloud2 | ground points (debug, `publish_ground`) |

All outputs use RELIABLE QoS with depth 1 and carry the timestamp of the newest camera cloud.

Example Nav2 costmap observation source (the same as rover 2.2):

```yaml
observation_sources: obstacles
obstacles:
  topic: /obstacles/points
  data_type: "PointCloud2"
  marking: True
  clearing: True
  # Heights are already filtered relative to the local ground; keep Nav2's band wide so points
  # on hills/slopes (z far from 0 in base_footprint) are not cut off again.
  min_obstacle_height: -1.0
  max_obstacle_height: 3.0
  obstacle_max_range: 6.0
  raytrace_max_range: 8.0
```

## How detection works

1. **Fusion:** each camera's cloud is transformed into `base_footprint`. The camera transforms are static, so they are looked up once and cached, and then the TF listener is released. Points are clipped to `range_min`–`range_max`, points inside the rover's box (`self_filter`) are removed, and the result goes through a voxel filter. A camera that sends nothing for longer than `stale_timeout` is left out of the fusion with a warning, and the rest keep working.
2. **Ground plane:** the cloud is divided into 25 cm cells, and each cell's ground estimate is its 10th z percentile. RANSAC fits a plane through these estimates with a tilt of at most 20°, which absorbs the rover's own pitch and roll. If the fit fails, a level plane at `expected_ground_z` is used instead.
3. **Local ground:** cells close to the plane are trusted ground. The ground under every other cell (one that is raised, sparse or not visible) is taken from its neighbours, rising no faster than `max_slope_deg` (25°) allows. As a result, a hill gentler than 25° stays ground, while the top of a rock whose base is hidden is still pulled down to the surrounding terrain.
4. **Obstacle:** a point between `obstacle.min_height` (0.15 m) and `obstacle.max_height` (2.0 m) above the local ground. Points above that are overhangs, such as branches, and are ignored.
5. **Clusters:** Euclidean clustering with a 0.3 m tolerance.
6. **Holes** (`negative.enabled`, experimental): cells far below the ground plane.

## Configuration

Cameras are configured in two places, each with one job:

- `src/wizja_bringup/config/cameras.yaml`: **which** cameras exist (serial, `source: driver|external`, topics, driver settings). The driver nodes, the fusion node topics, the simulator and the coverage tool are generated from it.
- `src/wizja_description/urdf/wizja_rover.urdf.xacro`: **where** they are mounted ("CAMERA POSITIONS" block, see below).

To add a 4th camera, add an entry to `cameras.yaml` and an `<xacro:d455 …>` line to the URDF; no code changes are needed. If a camera is in the yaml but not in the URDF, the launch files stop with an error that says so.

`src/wizja_perception/config/perception.yaml` holds the algorithm parameters, each with a comment. The most important ones:

| Parameter | Default | Effect |
|---|---|---|
| `obstacle.min_height` | 0.15 m | lowest detected obstacle (match it to what the wheels can drive over) |
| `ground.max_slope_deg` | 25° | steeper terrain counts as an obstacle |
| `ground.trust_band` | 0.12 m | cells up to this far above the plane are trusted as ground |
| `ground.max_tilt_deg` | 20° | maximum rover pitch/roll for the plane fit |
| `self_filter.min/max` | x −0.56…0.66, y ±0.50, z −0.1…1.25 m | the Rover 3.0 body, wheels, post and mast (from the rover30 meshes) plus ~5 cm |
| `voxel_leaf` | 0.05 m | fused cloud density (CPU vs. detail) |
| `range_max` | 8 m | the D455 has large depth error beyond ~6–8 m |

### Rover 3.0 dimensions (from Aut-Main `rover30_description`)

| | |
|---|---|
| `base_footprint` → `base_link` | z = 0.37956 m (base_link is above the ground; `base_footprint` is the wheel contact plane) |
| Deck | x −0.41…0.40, y ±0.217 m, top 0.18 m above `base_link` |
| Wheels | radius 0.13 m, width 0.14 m, axles at x +0.48 / −0.36 m, outer faces at y ±0.454 m |
| Overall | x −0.51…0.61, y ±0.454 m, up to 1.17 m above the ground (rear mast) |

### Camera mounting

- **Front:** the rover's existing D455 at the front edge of the deck, **level**, exactly where the rover30 URDF has it (`camera_joint`: bottom screw at `base_link` (0.42, 0, 0.18)). It is 0.57 m above the ground and sees the ground from about 1.06 m ahead. Tilting it ~15° down would bring that to ~0.62 m, but vision and visual odometry use the same camera, so change that only together with them (in this project's URDF **and** in Aut-Main `rover30_description`).
- **Left / right:** on a plate on a **~0.20 m post where the lidar used to be** (`base_link` x 0.33, y 0, on top of the deck), yaw ±80°, 20° down. The post matters: lying flat on the deck, the deck, rocker arms and wheels would hide the side cameras' view of the ground out to about 3 m. On the post they are about 0.78 m above the ground and see it from about 0.7 m out.

Check any layout with:

```bash
ros2 run wizja_bringup coverage_map.py --height 0.2        # ASCII map, includes occlusion by the rover body
```

| Camera | Tripod screw (base_link) | Yaw | Pitch |
|---|---|---|---|
| front | (0.420, 0.000, 0.180), = rover30 `camera_joint` | 0° | 0° (level) |
| left | (0.289, +0.083, 0.385), on the post | +80° | 20° down |
| right | (0.289, −0.083, 0.385), on the post | −80° | 20° down |

The sides are at ±80°, not ±90°, so their views overlap the front camera's on the front diagonals. The whole front half is covered; the rear (~125°) is not.

### Changing camera positions

The camera positions live **in the URDF**: `src/wizja_description/urdf/wizja_rover.urdf.xacro`, "CAMERA POSITIONS" block. There is one line per camera:

```xml
<xacro:d455 name="camera_left"  parent="base_link" xyz="0.28910 0.08254 0.38520" rpy="0 0.35 1.3963"/>
<xacro:d455 name="camera_right" parent="base_link" xyz="0.28854 -0.08304 0.38520" rpy="0 0.35 -1.3963"/>
```

- `xyz` is the position of the camera's **tripod screw** (the threaded hole in the bottom centre of the D455), measured from `base_link`: x forward, y left, z up, in metres. `base_link` is the centre of the rover, 0.38 m above the ground; the deck top is at z = 0.18. The macro adds the fixed offset from the screw to the camera's depth origin (`<name>_link`: 1.1 cm forward, 4.75 cm left, 1.45 cm up in the camera's own frame), the same way rover30's `rs_d455.urdf.xacro` does.
- `rpy` is roll, pitch, yaw in radians (URDF convention). `pitch > 0` tilts the camera **down** (0.35 rad ≈ 20°). `yaw > 0` turns it left: +1.5708 faces left, −1.5708 faces right.
- The **front** camera (`name="camera"`) is the main stack's: on the rover its TF comes from Aut-Main `rover30_description` (`camera_joint`). If you move it, change it in both files so they stay equal.

After editing:

```bash
cd ~/lr_wizja_kamery
colcon build --packages-select wizja_description     # the URDF is installed; rebuild after every edit
source install/setup.bash                            # (or build once with --symlink-install to skip this)
ros2 launch wizja_bringup view_model.launch.py       # do the camera models sit where the real cameras are?
ros2 run wizja_bringup coverage_map.py --height 0.2  # what they see and where the blind spots are
colcon test --packages-select wizja_bringup          # test_description.py pins the current poses (EXPECTED_LINK_POSES):
                                                     # update it when you move cameras on purpose
```

Fine-tuning on the rover:

1. Measure the screw positions with a tape measure from `base_link`, and measure the tilt with an inclinometer. Enter them and check with `view_model.launch.py`.
2. In RViz on a flat area, `/wizja/ground_points` should be one flat surface with no "steps" where the cameras meet. Place a box in the overlap of two cameras; the two views must line up. If they don't, adjust the yaw or pitch in steps of about 0.01 rad.
3. With the rover standing level, `/obstacles/points` should be empty on flat ground. If the edges flicker, correct the pitch of the camera concerned.

### Integration with the main rover stack (Aut-Main)

- Run `wizja.launch.py` with the default `rover_model:=cameras_only` next to the main stack, **including its RealSense launch** (the front camera's depth comes from it). Our outputs are in `base_footprint`, the same frame as Nav2's `robot_base_frame`.
- **Give the main stack's driver the front camera's serial.** `rs.launch.py` currently starts `realsense_camera` without `serial_no`, so it opens whichever RealSense it finds first, which can be one of the side cameras (then our driver for that side fails, and the "front" stream shows the side view). Add one line to its parameters in Aut-Main `src/rover30_depth_camera/launch/rs/rs.launch.py`:

  ```python
  realsense_node = Node(
      package='realsense2_camera',
      executable='realsense2_camera_node',
      name='realsense_camera',
      ...
      parameters=[{
          'serial_no': '_254122303704',   # front (Center) D455 - leaves the side cameras to lr_wizja_kamery
          'align_depth.enable': True,
          ...
  ```

  Our side-camera drivers already open their cameras by serial.
- The rover30 URDF still contains the lidar (`laser_frame`); it should eventually be removed from `rover30_description` (not part of this repo).
- To use the cameras in Nav2, replace the lidar observation source with `/obstacles/points` (example above).

## Performance and USB

- Driver settings in `cameras.yaml`: depth only (colour and IMU off), 640×480 at 15 fps, decimation ×2, then the spatial and temporal filters. That gives about 60–75k points per camera.
- Measured on a laptop (i5, 8 threads) with the simulator at 15 Hz and 3 cameras: the perception container uses about 0.6 of one core, and the detector takes 10–17 ms per frame.
- Ways to reduce the load: `decimation_filter.filter_magnitude: 3–4`, a larger `voxel_leaf`, `point_stride: 2`, or a lower `depth_profile` fps.
- Each camera should be on USB 3 (`rs-enumerate-devices` shows the USB type). With 3 cameras, spread them across the Geekom's different USB controllers or ports and avoid unpowered hubs. If frames drop, lower the fps or the resolution.
- `initial_reset: true` and `respawn` restart a camera that hangs, and the fusion keeps working without it in the meantime.
- `use_fastdds_udp_profile` is the same workaround as in rover 2.2: Fast-DDS without SHM or data-sharing, because of a busy loop with large messages (eProsima/Fast-DDS#6338).

## Tests

```bash
colcon test && colcon test-result --verbose
```

- `wizja_perception/test/test_ground_segmentation.cpp`: flat ground, a box, an object below the threshold, a 15° slope, a box on a slope, a step, a box with a hidden base, a tilted rover, a ditch, an overhang, and the occupancy grid.
- `wizja_bringup/test/test_camera_config.py`: the `cameras.yaml` format, serials, frame and topic names, and the rotation convention.
- `wizja_bringup/test/test_description.py`: the URDF in `full` and `cameras_only` modes (the shared front camera only in `full`), every camera of `cameras.yaml` placed in the URDF, the current camera poses (`EXPECTED_LINK_POSES`), camera headings (front/left/right), and no camera inside the rover body.
- `wizja_bringup/test/test_sim_kinematics.py`: the simulated rover's 4-wheel steering (forward, sideways, turning on the spot, angle flip at the ±90° joint limit, holding the angle when stopped) and the pose integration (limits, acceleration, command timeout).
- `wizja_perception/test/test_depth_projection.cpp`: depth-image back-projection (pixel → xyz, decimation, invalid depth, 16UC1 vs 32FC1, padded rows).

## Known limitations

- **No memory:** each frame is analysed independently. An obstacle that leaves the field of view disappears from `/obstacles/*`; the Nav2 costmap keeps it.
- **Blind areas:** about 125° at the rear, the strips right beside the rover body, the first ~1.06 m in front of the (level) front camera and ~0.7 m beside the side cameras.
- **Ditch edges:** where the ground drops sharply (more than `obstacle.min_height`), the upper rim of the drop is reported as an obstacle. That is conservative, and in practice it marks the ditch. The ditch floor itself is not an obstacle.
- **Front camera stream:** it comes from the main stack's driver settings (640×480 at 30 fps, no spatial/temporal filter), so it is noisier than the side cameras and is processed at 30 Hz (decimation 2 → ~77k points per frame). Raise `decimation` in `cameras.yaml` if CPU is short.
- **Surfaces:** stereo depth fails on smooth, shiny or transparent surfaces and in dust. The D455's active projector helps indoors; outdoors the D455 works well in sunlight because it is a stereo camera.
- The algorithm does not use an IMU. The rover's tilt is estimated from the ground plane, which works as long as part of the visible ground is flat.
