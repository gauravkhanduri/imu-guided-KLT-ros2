# gyro_aided_tracker_cpp

A ROS2 (Humble) C++ node that fuses IMU gyroscope data with KLT optical-flow tracking to produce robust, low-latency visual feature tracking for a ZED2 stereo-inertial camera. Pose estimates are forwarded to a flight controller via MAVROS with covariance scaled by tracking health.

---

## How It Works

Standard KLT tracking fails during fast motion because features move far between frames. This node solves that by:

1. **Integrating gyro samples** between two image frames using Rodrigues' rotation formula to get a rotation matrix `R`.
2. **Predicting feature positions** in the new frame via the homography `H = K · R · K⁻¹`, projecting each tracked point through the camera intrinsics.
3. **Running KLT** (`calcOpticalFlowPyrLK`) with the gyro-predicted positions as the initial guess — smaller search window needed, fewer false matches.
4. **Rejecting outliers** with RANSAC (`findHomography`) on the surviving point pairs.
5. **Replenishing features** with grid-based `goodFeaturesToTrack` when the count drops below a threshold.
6. **Scaling pose covariance** based on tracking health before publishing to MAVROS.

---

## Architecture

```
ZED2 IMU  (400 Hz)  ──►  imuCallback  ──►  imu_buffer_  (mutex-protected)
                                                  │
ZED2 Image (60 Hz)  ──►  imageCallback ───────────┘
                              │
                         GyroAidedTracker::track()
                              │
                    ┌─────────┴──────────┐
               TrackResult           PoseHealth
                    │                    │
              odomCallback         /pose_health
                    │
        /mavros/vision_pose/pose_cov
```

### Classes

| Class | Role |
|-------|------|
| `IMUSample` | Timestamped gyro + accel measurement |
| `TrackResult` | Per-frame stats: feature count, lost, new, median prediction error |
| `GyroAidedTracker` | Pure CV/Eigen algorithm class — no ROS dependency |
| `TrackerNode` | ROS2 node — wires subscriptions, publishes results |

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/zed2/zed_node/imu/data` | `sensor_msgs/Imu` | Sub | Raw IMU at ~400 Hz |
| `/zed2/zed_node/left/image_rect_gray` | `sensor_msgs/Image` | Sub | Rectified grayscale at ~60 Hz |
| `/zed2/zed_node/odom` | `nav_msgs/Odometry` | Sub | ZED2 visual odometry |
| `/mavros/vision_pose/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | Pub | Covariance-scaled pose for FCU |
| `/pose_health` | `gyro_aided_tracker_cpp/PoseHealth` | Pub | Tracking diagnostics at image rate |

---

## Custom Message — `PoseHealth.msg`

```
std_msgs/Header header
int32   n_features            # active tracked features
int32   n_lost                # features dropped this frame
int32   n_new                 # features detected this frame
float32 median_pred_error_px  # median gyro-prediction vs KLT residual (px)
float32 gyro_magnitude_dps    # gyro magnitude (deg/s)
float32 accel_magnitude       # accelerometer magnitude (m/s²)
uint8   tracking_state        # 0=HEALTHY 1=MARGINAL 2=DEGRADED 3=LOST
float32 covariance_scale      # multiplier applied to position covariance
```

---

## Tracking States & Covariance Scaling

| State | Condition | Covariance scale |
|-------|-----------|-----------------|
| `0` HEALTHY | ≥ 80 features **and** pred_err < 5 px | × 1 |
| `1` MARGINAL | ≥ 30 features **or** pred_err ≥ 5 px | × 3 |
| `2` DEGRADED | 6 – 29 features | × 10 |
| `3` LOST | ≤ 5 features | pose suppressed |

When LOST, the node logs a throttled warning and skips publishing to MAVROS entirely.

---

## Parameters (hardcoded — edit `TrackerNode` constructor)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `fx`, `fy` | 527.0 | Focal lengths (px) |
| `cx`, `cy` | 640.0, 360.0 | Principal point (px) |
| `max_features` | 200 | Maximum tracked features |
| `min_features` | 50 | Replenishment threshold |
| `grid_size` | 8 | N×N grid for feature detection |
| `klt_win_size` | 7 | KLT half-window → 15×15 patch |
| `ransac_thresh` | 1.5 px | RANSAC inlier distance |

---

## Dependencies

- ROS2 Humble
- OpenCV (4.x)
- Eigen3
- `cv_bridge`, `image_transport`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `rosidl_default_generators` / `rosidl_default_runtime`

---

## Build

```bash
cd ~/Advance_C++_ROS2/vio_ws
colcon build --packages-select gyro_aided_tracker_cpp
source install/setup.bash
```

## Run

```bash
ros2 run gyro_aided_tracker_cpp gyro_aided_tracker_node
```

Make sure the ZED2 ROS2 driver is running and publishing on the expected topics before starting this node.

---

## Hardware

Tested with a **ZED2** stereo-inertial camera connected to a companion computer. The pose output targets a **PX4 / ArduPilot** flight controller via **MAVROS** for vision-based position hold.
