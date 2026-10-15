# lr_wizja_kamery: depth-camera obstacle detection for the rover

This project replaces the lidar with 3 Intel RealSense D455 depth cameras (86° HFOV, one per side). It fuses their point clouds into a single cloud in `base_link` and detects obstacles relative to the locally estimated ground, so slopes and hills are not reported as obstacles. It is based on the rover 2.2 code (`kick-autonomy`, `rover_perception/height_filter_node.py`), extended to 3 cameras and with the fixed height slice replaced by ground-relative analysis.

ROS 2 Humble · Ubuntu 22.04 · C++17 / PCL · target computer: Geekom (x86_64).

```
 D455 front ─┐  /camera_front/depth/color/points
 D455 left  ─┼─► cloud_fusion ──► /wizja/fused_points ──► obstacle_detector ─┬─► /obstacles/points    (PointCloud2)
 D455 right ─┘   (TF to base_link,     (base_link)          (ground plane +     ├─► /obstacles/grid      (OccupancyGrid)
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
| `wizja_bringup` | launch files, `config/cameras.yaml` (cameras, serials, mounting poses, driver settings), RViz config, the `fake_cameras.py` simulator and the `coverage_map.py` tool |

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

```bash
rs-enumerate-devices -s              # serial numbers, one per camera
ros2 launch wizja_bringup wizja.launch.py \
    serial_front:=<serial> serial_left:=<serial> serial_right:=<serial>
# or put the serials in src/wizja_bringup/config/cameras.yaml (as quoted strings) and run:
ros2 launch wizja_bringup wizja.launch.py
# add use_rviz:=true to open RViz (if the Geekom has a screen)
```

Launch arguments: `cameras_config`, `serial_<name>`, `publish_camera_tf` (set `false` if the rover's URDF already defines the camera links), `use_fastdds_udp_profile`, `use_rviz`. You can also start the parts separately with `cameras.launch.py` (drivers + TF) and `perception.launch.py` (fusion + detection; it also takes `perception_params`, `log_level` and `use_sim_time`).

**Without hardware (synthetic cameras):**

```bash
ros2 launch wizja_bringup sim_test.launch.py            # use_rviz:=false without a screen
```

`fake_cameras.py` ray-casts each camera from `cameras.yaml` (D455 FOV, 320×240 after decimation, depth noise ∝ z²) against a test scene: two boxes, a pole, a rock on a hill, a 14° hill and an 8 cm bump. The detector should report exactly 4 obstacles; the hill and the bump must not be among them.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera_<name>/depth/color/points` | PointCloud2 | driver input (BEST_EFFORT) |
| `/wizja/fused_points` | PointCloud2 | fused cloud in `base_link`, rover body removed, voxel 5 cm |
| `/obstacles/points` | PointCloud2 | obstacle points (same topic as rover 2.2, so it works directly as an observation source for the Nav2 costmap) |
| `/obstacles/grid` | OccupancyGrid | 16×16 m, 10 cm, centred on `base_link`: 100 obstacle, 0 ground seen, −1 unknown |
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
  # on hills/slopes (z far from 0 in base_link) are not cut off again.
  min_obstacle_height: -1.0
  max_obstacle_height: 3.0
  obstacle_max_range: 6.0
  raytrace_max_range: 8.0
```

## How detection works

1. **Fusion:** each camera's cloud is transformed into `base_link`. The camera transforms are static, so they are looked up once and cached, and then the TF listener is released. Points are clipped to `range_min`–`range_max`, points inside the rover's box (`self_filter`) are removed, and the result goes through a voxel filter. A camera that sends nothing for longer than `stale_timeout` is left out of the fusion with a warning, and the rest keep working.
2. **Ground plane:** the cloud is divided into 25 cm cells, and each cell's ground estimate is its 10th z percentile. RANSAC fits a plane through these estimates with a tilt of at most 20°, which absorbs the rover's own pitch and roll. If the fit fails, a level plane at `expected_ground_z` is used instead.
3. **Local ground:** cells close to the plane are trusted ground. The ground under every other cell (one that is raised, sparse or not visible) is taken from its neighbours, rising no faster than `max_slope_deg` (25°) allows. As a result, a hill gentler than 25° stays ground, while the top of a rock whose base is hidden is still pulled down to the surrounding terrain.
4. **Obstacle:** a point between `obstacle.min_height` (0.15 m) and `obstacle.max_height` (2.0 m) above the local ground. Points above that are overhangs, such as branches, and are ignored.
5. **Clusters:** Euclidean clustering with a 0.3 m tolerance.
6. **Holes** (`negative.enabled`, experimental): cells far below the ground plane.

## Configuration

`src/wizja_bringup/config/cameras.yaml` is the only list of cameras. The driver nodes, the TFs, the fusion node topics and the simulator are all generated from it. To use 4 cameras later, add an entry here; no code changes are needed.

`src/wizja_perception/config/perception.yaml` holds the algorithm parameters, each with a comment. The most important ones:

| Parameter | Default | Effect |
|---|---|---|
| `obstacle.min_height` | 0.15 m | lowest detected obstacle (match it to what the wheels can drive over) |
| `ground.max_slope_deg` | 25° | steeper terrain counts as an obstacle |
| `ground.trust_band` | 0.12 m | cells up to this far above the plane are trusted as ground |
| `ground.max_tilt_deg` | 20° | maximum rover pitch/roll for the plane fit |
| `self_filter.min/max` | ±0.7 × ±0.65 m | **PLACEHOLDER**: rover dimensions plus a margin |
| `voxel_leaf` | 0.05 m | fused cloud density (CPU vs. detail) |
| `range_max` | 8 m | the D455 has large depth error beyond ~6–8 m |

### Calibrating the camera poses (`cameras.yaml`)

The poses in `cameras.yaml` are **placeholders**. `pose` is the transform `base_link → camera_<name>_link`, where `camera_<name>_link` is the RealSense body frame (X out of the lens, origin at the depth module). Angles are in radians in URDF order (roll, pitch, yaw), and `pitch > 0` tilts the camera **down**.

1. Measure x/y/z with a tape measure from `base_link` (the centre of the rover at ground level) and measure the tilt with an inclinometer.
2. Choose the layout before mounting: `ros2 run wizja_bringup coverage_map.py --height 0.2` draws an ASCII map of what the cameras see and lists the blind sectors. **Watch out:** side cameras mounted on the rover's sides (behind the front camera) leave blind slivers on the front diagonals even at a ±90° yaw. A compact cluster of all three cameras at the front with the sides at ±80° (the placeholder in `cameras.yaml`) covers the whole front.
3. Check it with RViz on a flat area: `/wizja/ground_points` should be one flat surface with no "steps" where the cameras meet. Place a box in the overlap of two cameras; the two views must line up. If they don't, adjust the yaw or pitch in steps of about 0.01 rad.
4. With the rover standing level, `/obstacles/points` should be empty on flat ground. If the edges flicker, correct the pitch of the camera concerned.

**Tilt:** about 20° down (0.35 rad) is recommended. At a mounting height of 0.5 m and a 58° VFOV, the ground is visible from about 0.43 m in front of the lens, and the camera still sees about 9° above the horizon. A level camera would only see the ground from about 0.9 m.

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
- `wizja_bringup/test/test_camera_config.py`: the `cameras.yaml` format, frame and topic names, and the rotation convention.

## Known limitations

- **No memory:** each frame is analysed independently. An obstacle that leaves the field of view disappears from `/obstacles/*`; the Nav2 costmap keeps it.
- **Blind areas:** about 130° at the rear in the placeholder layout, and about 0.4 m directly in front of each lens (the D455's minimum depth).
- **Ditch edges:** where the ground drops sharply (more than `obstacle.min_height`), the upper rim of the drop is reported as an obstacle. That is conservative, and in practice it marks the ditch. The ditch floor itself is not an obstacle.
- **Surfaces:** stereo depth fails on smooth, shiny or transparent surfaces and in dust. The D455's active projector helps indoors; outdoors the D455 works well in sunlight because it is a stereo camera.
- The algorithm does not use an IMU. The rover's tilt is estimated from the ground plane, which works as long as part of the visible ground is flat.
