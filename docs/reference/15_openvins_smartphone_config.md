# OpenVINS Smartphone Configuration Guide

A detailed configuration guide for using smartphone camera/IMU data with OpenVINS.

---

## 1. Overview

### 1.1 OpenVINS Features

| Feature | OpenVINS | VINS-Mono | ORB-SLAM3 |
|---------|----------|-----------|-----------|
| Filter type | **MSCKF (EKF)** | Optimization | Optimization |
| CPU usage | **Low** | Medium | High |
| Online calibration | **Comprehensive** | Partial | Not supported |
| Documentation | **Best** | Medium | Low |
| Loop Closure | Not supported | Supported | Supported |

### 1.2 Why OpenVINS is Suitable for Smartphones

1. **Low computational load**: CPU efficient with MSCKF-based filter
2. **Comprehensive online calibration**:
   - Camera-IMU extrinsic parameters (`calib_cam_extrinsics`)
   - Camera intrinsic parameters (`calib_cam_intrinsics`)
   - Time offset (`calib_cam_timeoffset`)
   - IMU intrinsic parameters (`calib_imu_intrinsics`)
3. **Excellent documentation**: Detailed calibration guide provided
4. **Kalibr compatible**: Uses standard calibration format

### 1.3 Limitations

- **No Loop Closure support**: VIO only (long-range drift accumulation)
- **Accuracy compared to optimization-based**: Slightly lower than VINS-Mono

---

## 2. Configuration File Structure

### 2.1 Required Configuration Files

```
config/
├── smartphone/
│   ├── estimator_config.yaml      # OpenVINS core settings
│   ├── kalibr_imu_chain.yaml      # IMU noise parameters
│   └── kalibr_imucam_chain.yaml   # Camera-IMU transform and camera intrinsics
```

### 2.2 estimator_config.yaml (Main Settings)

```yaml
%YAML:1.0

#=============================================================================
# OpenVINS Smartphone Configuration
# Test environment: Android/iOS smartphones
#=============================================================================

#-----------------------------------------------------------------------------
# General Settings
#-----------------------------------------------------------------------------
verbosity: "INFO"                    # ALL, DEBUG, INFO, WARNING, ERROR, SILENT

#-----------------------------------------------------------------------------
# State Options
#-----------------------------------------------------------------------------
use_fej: true                        # Use First Estimates Jacobian
use_imuavg: true                     # IMU measurement averaging
use_rk4_integration: true            # RK4 integration (false: Euler)
use_stereo: false                    # Stereo mode (smartphone: false)
max_cameras: 1                       # Number of cameras (smartphone: 1)
calib_cam_extrinsics: true           # Online camera-IMU extrinsic estimation
calib_cam_intrinsics: true           # Online camera intrinsic estimation
calib_cam_timeoffset: true           # Online time offset estimation
calib_imu_intrinsics: false          # IMU intrinsic estimation (advanced)
max_clones: 11                       # Sliding window size
max_slam: 50                         # Maximum SLAM features
max_slam_in_update: 25               # SLAM features per update
max_msckf_in_update: 40              # MSCKF features per update
dt_slam_delay: 1                     # SLAM delay (frames)

#-----------------------------------------------------------------------------
# Initialization Options
#-----------------------------------------------------------------------------
init_window_time: 1.0                # Initialization window size (seconds)
init_imu_thresh: 1.0                 # IMU motion threshold
init_max_disparity: 1.5              # Initialization maximum disparity
init_max_features: 50                # Initialization feature count

#-----------------------------------------------------------------------------
# Noise/Covariance Recording
#-----------------------------------------------------------------------------
record_timing_information: false
record_timing_filepath: "/tmp/timing_ov.txt"

#-----------------------------------------------------------------------------
# Feature Tracking
#-----------------------------------------------------------------------------
use_klt: true                        # KLT tracking (true recommended)
num_pts: 200                         # Features per camera
fast_threshold: 15                   # FAST detection threshold
grid_x: 5                            # Horizontal grid division
grid_y: 5                            # Vertical grid division
min_px_dist: 10                      # Minimum distance between features (pixels)
knn_ratio: 0.70                      # KNN matching ratio (when using ORB)
track_frequency: 20.0                # Tracking frequency (Hz)
downsample_cameras: false            # Image downsampling
multi_threading_subs: false          # Multi-threaded subscription

# Histogram equalization
histogram_method: "CLAHE"            # NONE, HISTOGRAM, CLAHE

#-----------------------------------------------------------------------------
# State Initialization
#-----------------------------------------------------------------------------
init_state:
  p_IinG: [0, 0, 0]
  q_GtoI: [0, 0, 0, 1]
  v_IinG: [0, 0, 0]
  bg: [0, 0, 0]
  ba: [0, 0, 0]

#-----------------------------------------------------------------------------
# MSCKF/SLAM Update Options
#-----------------------------------------------------------------------------
up_msckf_sigma_px: 1.0               # MSCKF pixel noise
up_msckf_chi2_multipler: 1.0         # MSCKF Chi2 multiplier
up_slam_sigma_px: 1.0                # SLAM pixel noise
up_slam_chi2_multipler: 1.0          # SLAM Chi2 multiplier

#-----------------------------------------------------------------------------
# Zero Velocity Update (ZUPT)
#-----------------------------------------------------------------------------
try_zupt: true                       # Attempt ZUPT
zupt_chi2_multipler: 1.0             # ZUPT Chi2 multiplier
zupt_max_velocity: 0.1               # Stationary judgment velocity threshold (m/s)
zupt_noise_multiplier: 10.0          # ZUPT noise multiplier
zupt_max_disparity: 0.5              # ZUPT maximum disparity

#-----------------------------------------------------------------------------
# File Paths (can be overridden at runtime)
#-----------------------------------------------------------------------------
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

### 2.3 kalibr_imu_chain.yaml (IMU Noise)

```yaml
%YAML:1.0

#=============================================================================
# IMU Noise Parameters (Kalibr format)
# Important: Setting 10-20x actual values recommended (accounting for unmodeled errors)
#=============================================================================

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: 0.1      # m/s²/√Hz (σ_a)
  accelerometer_random_walk: 0.002      # m/s³/√Hz (σ_ba)
  gyroscope_noise_density: 0.01         # rad/s/√Hz (σ_g)
  gyroscope_random_walk: 0.0002         # rad/s²/√Hz (σ_bg)
  model: calibrated
  rostopic: /smartphone/imu
  time_offset: 0.0
  update_rate: 200.0                    # IMU update rate (Hz)
```

### 2.4 kalibr_imucam_chain.yaml (Camera-IMU)

```yaml
%YAML:1.0

#=============================================================================
# Camera-IMU Calibration (Kalibr format)
#=============================================================================

cam0:
  T_imu_cam:                            # Transform from IMU to camera
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.0]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole                 # pinhole or omni
  distortion_coeffs: [-0.28, 0.08, 0.0, 0.0]
  distortion_model: radtan              # radtan, equidistant
  intrinsics: [900.0, 900.0, 640.0, 360.0]  # [fx, fy, cx, cy]
  resolution: [1280, 720]
  rostopic: /smartphone/camera/image_raw
  timeshift_cam_imu: 0.02               # Camera-IMU time offset (seconds)
```

---

## 3. Key Parameter Details

### 3.1 IMU Noise Parameters

#### Parameter Definitions

| Parameter | Symbol | Unit | Description |
|-----------|--------|------|-------------|
| `accelerometer_noise_density` | σ_a | m/s²/√Hz | Accelerometer white noise |
| `gyroscope_noise_density` | σ_g | rad/s/√Hz | Gyroscope white noise |
| `accelerometer_random_walk` | σ_ba | m/s³/√Hz | Accelerometer bias random walk |
| `gyroscope_random_walk` | σ_bg | rad/s²/√Hz | Gyroscope bias random walk |

#### Extracting Values from Allan Variance

```
From Allan Deviation Plot:

1. White Noise (σ): Value at slope=-1/2 line at τ=1
2. Random Walk: Extract from slope=+1/2 line

              │
  log(σ(τ))   │   slope = -1/2
              │        ↘
              │          ↘_____ slope ≈ 0
              │                ↘
              │                  ↘ slope = +1/2
              └───────────────────────→ log(τ)
                        τ=1
```

#### Recommended Values by Smartphone Tier

```yaml
# Flagship smartphones (with 10-20x inflation applied)
accelerometer_noise_density: 0.08
accelerometer_random_walk: 0.001
gyroscope_noise_density: 0.008
gyroscope_random_walk: 5.0e-5

# Mid-range smartphones
accelerometer_noise_density: 0.1
accelerometer_random_walk: 0.002
gyroscope_noise_density: 0.01
gyroscope_random_walk: 0.0002

# Budget smartphones (conservative)
accelerometer_noise_density: 0.2
accelerometer_random_walk: 0.005
gyroscope_noise_density: 0.02
gyroscope_random_walk: 0.0005
```

**Important**: OpenVINS documentation recommends "**inflating these noise values by 10-20x** the actual values to account for unmodeled errors."

### 3.2 Online Calibration Options

```yaml
# Camera-IMU extrinsic parameters (R_ItoC, p_CinI)
calib_cam_extrinsics: true    # Smartphone: Enable recommended

# Camera intrinsic parameters (focal length, center point, distortion)
calib_cam_intrinsics: true    # Enable if initial calibration is inaccurate

# Camera-IMU time offset
calib_cam_timeoffset: true    # Smartphone: Must enable

# IMU intrinsic parameters (rotation, scale errors)
calib_imu_intrinsics: false   # Advanced option, generally disabled
```

### 3.3 Feature Tracking Parameters

#### Key Parameters

```yaml
# Tracking method
use_klt: true                 # KLT (true) vs ORB+matching (false)

# Number of features (performance vs accuracy)
num_pts: 200                  # Smartphone recommended: 150-250

# FAST detection threshold (lower = more features, slower)
fast_threshold: 15            # 15-25 recommended

# Grid for uniform distribution
grid_x: 5                     # Divide image into 5x5 grid
grid_y: 5

# Minimum distance between features
min_px_dist: 10               # 10-15 pixels

# Tracking frequency (separate from camera FPS)
track_frequency: 20.0         # 20Hz or higher recommended
```

#### Histogram Equalization

```yaml
# Handling low-light/high-contrast environments
histogram_method: "CLAHE"     # NONE, HISTOGRAM, CLAHE

# CLAHE: Contrast Limited Adaptive Histogram Equalization
# - Local contrast enhancement
# - Limits noise amplification
# - Effective for smartphones
```

### 3.4 Initialization Parameters

```yaml
# Initialization window
init_window_time: 1.0         # Data time used for initialization (seconds)

# IMU motion threshold
init_imu_thresh: 1.0          # Acceleration change to trigger initialization

# Disparity-based initialization (static initialization)
init_max_disparity: 1.5       # Maximum allowed disparity (pixels)

# Initialization features
init_max_features: 50         # Number of features for initialization
```

**Initialization behavior**:
1. Attempt dynamic initialization (requires IMU motion)
2. If failed, attempt static initialization (disparity-based)
3. If stationary, wait for motion

### 3.5 ZUPT (Zero Velocity Update)

```yaml
# Enable ZUPT
try_zupt: true

# Stationary judgment thresholds
zupt_max_velocity: 0.1        # Judge as stationary if below m/s
zupt_max_disparity: 0.5       # Pixel disparity threshold

# ZUPT noise settings
zupt_noise_multiplier: 10.0   # ZUPT measurement noise multiplier
zupt_chi2_multipler: 1.0      # Chi2 test multiplier
```

**ZUPT on smartphones**:
- Stationary state is rare when held in hand
- Effective when stationary on a table
- Helps with drift correction

---

## 4. Smartphone-Specific Configuration Templates

### 4.1 Android Smartphone

**estimator_config.yaml**:
```yaml
%YAML:1.0

#=============================================================================
# Android Smartphone Configuration for OpenVINS
#=============================================================================

verbosity: "INFO"

# State options
use_fej: true
use_imuavg: true
use_rk4_integration: true
use_stereo: false
max_cameras: 1
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
calib_imu_intrinsics: false
max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

# Initialization (slightly relaxed for Android)
init_window_time: 1.5
init_imu_thresh: 0.8
init_max_disparity: 2.0
init_max_features: 50

# Feature tracking
use_klt: true
num_pts: 200
fast_threshold: 15
grid_x: 5
grid_y: 5
min_px_dist: 10
knn_ratio: 0.70
track_frequency: 20.0
downsample_cameras: false
histogram_method: "CLAHE"

# Update options
up_msckf_sigma_px: 1.0
up_msckf_chi2_multipler: 1.0
up_slam_sigma_px: 1.0
up_slam_chi2_multipler: 1.0

# ZUPT
try_zupt: true
zupt_chi2_multipler: 1.0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10.0
zupt_max_disparity: 0.5

# Configuration file paths
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

**kalibr_imu_chain.yaml**:
```yaml
%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  # Android mid-range smartphone (with inflation applied)
  accelerometer_noise_density: 0.1
  accelerometer_random_walk: 0.002
  gyroscope_noise_density: 0.01
  gyroscope_random_walk: 0.0002
  model: calibrated
  rostopic: /android/imu
  time_offset: 0.0
  update_rate: 200.0
```

**kalibr_imucam_chain.yaml**:
```yaml
%YAML:1.0

cam0:
  T_imu_cam:
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.02]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.25, 0.06, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [920.0, 920.0, 640.0, 360.0]
  resolution: [1280, 720]
  rostopic: /android/camera/image_raw
  timeshift_cam_imu: 0.025
```

### 4.2 iPhone

**estimator_config.yaml**:
```yaml
%YAML:1.0

#=============================================================================
# iPhone Configuration for OpenVINS
#=============================================================================

verbosity: "INFO"

# State options
use_fej: true
use_imuavg: true
use_rk4_integration: true
use_stereo: false
max_cameras: 1
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
calib_imu_intrinsics: false
max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

# Initialization (iPhone has better synchronization)
init_window_time: 1.0
init_imu_thresh: 1.0
init_max_disparity: 1.5
init_max_features: 50

# Feature tracking
use_klt: true
num_pts: 180
fast_threshold: 18
grid_x: 5
grid_y: 5
min_px_dist: 12
knn_ratio: 0.70
track_frequency: 25.0
downsample_cameras: false
histogram_method: "CLAHE"

# Update options
up_msckf_sigma_px: 1.0
up_msckf_chi2_multipler: 1.0
up_slam_sigma_px: 1.0
up_slam_chi2_multipler: 1.0

# ZUPT
try_zupt: true
zupt_chi2_multipler: 1.0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10.0
zupt_max_disparity: 0.5

# Configuration file paths
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

**kalibr_imu_chain.yaml**:
```yaml
%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  # iPhone (good quality, with inflation applied)
  accelerometer_noise_density: 0.08
  accelerometer_random_walk: 0.001
  gyroscope_noise_density: 0.008
  gyroscope_random_walk: 5.0e-5
  model: calibrated
  rostopic: /iphone/imu
  time_offset: 0.0
  update_rate: 200.0
```

**kalibr_imucam_chain.yaml**:
```yaml
%YAML:1.0

cam0:
  T_imu_cam:
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.0]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.20, 0.04, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [1400.0, 1400.0, 960.0, 540.0]
  resolution: [1920, 1080]
  rostopic: /iphone/camera/image_raw
  timeshift_cam_imu: 0.015
```

---

## 5. Calibration Workflow

### 5.1 IMU Noise Calibration

```bash
# 1. Collect IMU data in stationary state (minimum 2 hours, recommended 20+ hours)
rosbag record /smartphone/imu -o imu_static.bag

# 2. Allan Variance analysis
# Using allan_variance_ros
rosrun allan_variance_ros allan_variance \
  /smartphone/imu \
  imu_static.bag \
  --output imu_allan.yaml

# 3. Check results and generate YAML
# Extract σ_g, σ_a, σ_bg, σ_ba values
# Inflate by 10-20x and enter into kalibr_imu_chain.yaml
```

### 5.2 Camera Intrinsic Calibration

```bash
# 1. Prepare Aprilgrid (6x6, 0.8m x 0.8m recommended)

# 2. Collect calibration data
rosbag record /smartphone/camera/image_raw -o camera_calib.bag
# Capture for 30-60 seconds at various angles and distances

# 3. Run Kalibr
rosrun kalibr kalibr_calibrate_cameras \
  --bag camera_calib.bag \
  --topics /smartphone/camera/image_raw \
  --models pinhole-radtan \
  --target aprilgrid_6x6.yaml \
  --bag-freq 10.0

# 4. Check results
# Reprojection error should be < 0.2-0.5 pixels
```

### 5.3 Camera-IMU Extrinsic Calibration

```bash
# 1. Collect dynamic calibration data
rosbag record /smartphone/camera/image_raw /smartphone/imu -o imu_cam_calib.bag
# 30-60 seconds, various rotations and movements

# 2. Data collection tips:
# - Minimize motion blur (smooth movements)
# - Need at least 1 translation + 2 or more rotations
# - Rotate in as many directions as possible

# 3. Run Kalibr
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag imu_cam_calib.bag \
  --cam camera_calib-results.yaml \
  --imu kalibr_imu_chain.yaml \
  --target aprilgrid_6x6.yaml

# 4. Verify results
# - Check that accelerometer/gyroscope errors are within 3σ range
# - Check reprojection error
```

---

## 6. Execution Methods

### 6.1 ROS 1

```bash
# 1. Place configuration files
mkdir -p ~/catkin_ws/src/open_vins/config/smartphone/
# Copy estimator_config.yaml, kalibr_*.yaml

# 2. Run
roscore

# 3. Run OpenVINS
roslaunch ov_msckf subscribe.launch \
  config_path:=$(rospack find ov_msckf)/config/smartphone/estimator_config.yaml \
  use_stereo:=false \
  max_cameras:=1

# 4. RViz
rviz -d $(rospack find ov_msckf)/launch/display.rviz

# 5. Data playback or real-time streaming
rosbag play smartphone_data.bag
```

### 6.2 ROS 2

```bash
# 1. Run
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/path/to/smartphone/estimator_config.yaml \
  use_stereo:=false \
  max_cameras:=1

# 2. Data playback
ros2 bag play smartphone_data
```

### 6.3 Launch Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `verbosity` | string | INFO | Log level |
| `config_path` | string | - | Configuration file path |
| `use_stereo` | bool | true | Stereo mode |
| `max_cameras` | int | 2 | Maximum number of cameras |

---

## 7. Troubleshooting

### 7.1 Initialization Failure

**Symptom**: System stays in "Waiting for initialization..." state

**Solutions**:

| Cause | Solution |
|-------|----------|
| Insufficient motion | Move/rotate in various directions |
| IMU threshold too high | Decrease `init_imu_thresh: 0.5` |
| Insufficient features | Increase `num_pts: 250`, decrease `fast_threshold: 12` |
| Static initialization failure | Increase `init_max_disparity: 2.0` |

```yaml
# Relaxed initialization settings
init_window_time: 2.0
init_imu_thresh: 0.5
init_max_disparity: 2.0
init_max_features: 75
```

### 7.2 Filter Divergence

**Symptom**: Trajectory deviates abruptly, pose estimation unstable

**Solutions**:

```yaml
# 1. Increase IMU noise (improve filter robustness)
accelerometer_noise_density: 0.2    # 2x increase
gyroscope_noise_density: 0.02       # 2x increase

# 2. Increase Chi2 multiplier (allow outliers)
up_msckf_chi2_multipler: 2.0
up_slam_chi2_multipler: 2.0

# 3. Adjust feature settings
num_pts: 250                        # Increase
min_px_dist: 8                      # Decrease
```

### 7.3 Severe Drift

**Symptom**: Gradual position/orientation error accumulation over time

**Checklist**:

```yaml
# 1. Check IMU noise parameters (too small causes drift)
accelerometer_random_walk: 0.005    # Try increasing
gyroscope_random_walk: 0.0005       # Try increasing

# 2. Verify online calibration is enabled
calib_cam_extrinsics: true
calib_cam_timeoffset: true

# 3. Enable ZUPT (drift correction when stationary)
try_zupt: true

# 4. Increase sliding window size
max_clones: 15                      # Increase from default 11
```

### 7.4 Unstable Feature Tracking

**Symptom**: Feature count fluctuates suddenly, tracking interruptions

```yaml
# Low-light environment
histogram_method: "CLAHE"
fast_threshold: 12                  # Decrease

# Low texture
num_pts: 300                        # Increase
grid_x: 7                           # Denser grid
grid_y: 7

# Fast motion
track_frequency: 30.0               # Increase
min_px_dist: 8                      # Decrease
```

### 7.5 Abnormal Time Offset

**Symptom**: `timeshift_cam_imu` value is abnormal

**Normal range**: -0.05s ~ +0.05s

```yaml
# Adjust initial value
timeshift_cam_imu: 0.02             # Start with 20ms

# Verify online estimation is enabled
calib_cam_timeoffset: true

# Verify data quality
# - Camera: minimum 15Hz, recommended 20-30Hz
# - IMU: minimum 100Hz, recommended 200Hz
```

---

## 8. VINS-Mono vs OpenVINS Comparison

| Item | VINS-Mono | OpenVINS |
|------|-----------|----------|
| **Filter type** | Optimization (Ceres) | MSCKF (EKF) |
| **CPU usage** | Medium | **Low** |
| **Memory usage** | Medium | **Low** |
| **Accuracy** | Slightly higher | High |
| **Initialization** | Self-implemented | Both static/dynamic supported |
| **Loop Closure** | **Supported** | Not supported |
| **Rolling Shutter** | **Supported** | Not supported |
| **Online calibration** | Extrinsic+time | **Extrinsic+intrinsic+time+IMU** |
| **Documentation** | Medium | **Best** |
| **Kalibr compatibility** | Partial | **Full** |

**Smartphone selection guide**:
- **Choose VINS-Mono**: Loop Closure needed, Rolling Shutter compensation needed
- **Choose OpenVINS**: Low-spec PC, comprehensive calibration needed, research/learning purposes

---

## 9. References

### Official Documentation
- [OpenVINS Documentation](https://docs.openvins.com/)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [OpenVINS Tutorial](https://docs.openvins.com/gs-tutorial.html)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)

### Calibration Tools
- [Kalibr](https://github.com/ethz-asl/kalibr)
- [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)

### Papers
- [OpenVINS: A Research Platform for Visual-Inertial Estimation](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)
- [OpenVINS State Initialization](https://pgeneva.com/downloads/reports/tr_init.pdf)

### Related Documents
- [05_vislam_frameworks.md](05_vislam_frameworks.md)
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)
- [14_vins_mono_smartphone_config.md](14_vins_mono_smartphone_config.md)
