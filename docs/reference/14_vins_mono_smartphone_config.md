# VINS-Mono Smartphone Configuration Guide

A detailed configuration guide for using smartphone camera/IMU data with VINS-Mono.

---

## 1. Overview

### 1.1 Why VINS-Mono?

| Feature | VINS-Mono | ORB-SLAM3 | OpenVINS |
|---------|-----------|-----------|----------|
| Automatic time offset estimation | **Supported** | Not supported | Supported |
| Asynchronous sensor tolerance | **Yes** | No | Yes |
| Rolling Shutter | **Supported** | Not supported | Supported |
| Smartphone suitability | **High** | Low | Medium |

**Why VINS-Mono is suitable for smartphone data**:
- Online camera-IMU time offset estimation (`estimate_td`)
- Online extrinsic parameter calibration (`estimate_extrinsic`)
- Rolling Shutter model support
- Robust initialization and failure recovery

### 1.2 Sensor Quality Ranking

Expected performance ranking suggested by the VINS-Mono development team:

```
Best → Worst:

1. Global Shutter + synchronized high-end IMU (VI-Sensor)
2. Global Shutter + synchronized low-cost IMU
3. Global Shutter + asynchronous high-frequency IMU
4. Global Shutter + asynchronous low-frequency IMU
5. Rolling Shutter + asynchronous low-frequency IMU ← Smartphones
```

**Note**: Do not use web cameras. ("Don't try web camera, the web camera is so awful.")

---

## 2. Configuration File Structure

### 2.1 Complete Structure

```yaml
%YAML:1.0

#------------------------------------------------------------------------------
# Topic and Output Settings
#------------------------------------------------------------------------------
imu_topic: "/smartphone/imu"
image_topic: "/smartphone/camera/image_raw"
output_path: "/home/user/vins_output/"

#------------------------------------------------------------------------------
# Camera Parameters
#------------------------------------------------------------------------------
model_type: PINHOLE
camera_name: "smartphone_camera"
image_width: 1280
image_height: 720

distortion_parameters:
   k1: -0.28
   k2: 0.08
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 900.0
   fy: 900.0
   cx: 640.0
   cy: 360.0

#------------------------------------------------------------------------------
# Extrinsic Parameters (Camera-IMU)
#------------------------------------------------------------------------------
estimate_extrinsic: 2  # 0: fixed, 1: optimize, 2: auto calibration

# Initial estimate (when estimate_extrinsic: 1 or 2)
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1,
          -1, 0, 0,
          0, -1, 0]
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

#------------------------------------------------------------------------------
# Time Synchronization
#------------------------------------------------------------------------------
estimate_td: 1         # Online time offset estimation
td: 0.02               # Initial estimate (seconds)

#------------------------------------------------------------------------------
# Rolling Shutter
#------------------------------------------------------------------------------
rolling_shutter: 1     # 1: enabled
rolling_shutter_tr: 0.033  # Rolling shutter readout time (seconds)

#------------------------------------------------------------------------------
# IMU Parameters
#------------------------------------------------------------------------------
acc_n: 0.1             # Accelerometer noise
gyr_n: 0.01            # Gyroscope noise
acc_w: 0.002           # Accelerometer random walk
gyr_w: 0.0002          # Gyroscope random walk
g_norm: 9.81           # Gravity magnitude

#------------------------------------------------------------------------------
# Feature Tracking
#------------------------------------------------------------------------------
max_cnt: 200           # Maximum number of features
min_dist: 25           # Minimum distance between features (pixels)
freq: 10               # Output frequency (Hz)
F_threshold: 1.0       # RANSAC threshold (pixels)
show_track: 1          # Tracking visualization
equalize: 1            # Histogram equalization
fisheye: 0             # Fisheye lens

#------------------------------------------------------------------------------
# Optimization
#------------------------------------------------------------------------------
max_solver_time: 0.04  # Maximum solver time (seconds)
max_num_iterations: 8  # Maximum iterations
keyframe_parallax: 10.0  # Keyframe parallax (pixels)

#------------------------------------------------------------------------------
# Loop Closure
#------------------------------------------------------------------------------
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/user/vins_output/pose_graph/"

#------------------------------------------------------------------------------
# Visualization
#------------------------------------------------------------------------------
save_image: 1
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

---

## 3. Key Parameter Details

### 3.1 Camera Calibration

#### Supported Models
- `PINHOLE`: Pinhole model (standard for smartphones)
- `MEI`: Unified camera model (fisheye)

#### Calibration Methods

```bash
# 1. Calibration with OpenCV
rosrun camera_calibration cameracalibrator.py \
  --size 9x6 \
  --square 0.025 \
  image:=/smartphone/camera/image_raw

# 2. Calibration with Kalibr
rosrun kalibr kalibr_calibrate_cameras \
  --bag /path/to/calibration.bag \
  --topics /smartphone/camera/image_raw \
  --models pinhole-radtan \
  --target april_6x6.yaml
```

#### Typical Smartphone Values

```yaml
# Approximate initial values for 1280x720 resolution
image_width: 1280
image_height: 720

projection_parameters:
   fx: 900.0    # Horizontal focal length (pixels)
   fy: 900.0    # Vertical focal length (pixels)
   cx: 640.0    # Principal point x (image center)
   cy: 360.0    # Principal point y (image center)

distortion_parameters:
   k1: -0.28    # Radial distortion 1st order
   k2: 0.08     # Radial distortion 2nd order
   p1: 0.0      # Tangential distortion
   p2: 0.0
```

**Important**: Rolling Shutter cameras require precise calibration with reprojection error **below 0.5 pixels**.

### 3.2 Extrinsic Parameters (Camera-IMU Transform)

#### estimate_extrinsic Options

| Value | Meaning | When to Use |
|-------|---------|-------------|
| **0** | Use fixed values | Accurate calibration completed |
| **1** | Optimize around initial values | Approximate calibration completed |
| **2** | Fully automatic estimation | **Recommended for smartphones** |

#### Recommended Smartphone Settings

```yaml
# Smartphone: Start without calibration
estimate_extrinsic: 2

# Rotation motion required for a few seconds during initialization!
```

**Usage**:
1. Set `estimate_extrinsic: 2`
2. **Rotate around various axes for 3-5 seconds** at system startup
3. Results are automatically saved on success

#### Typical Smartphone Camera-IMU Layout

```yaml
# Smartphone rear camera + IMU typical layout
# (Varies by device, accurate values require calibration)

# Camera → IMU rotation (approximate)
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1,      # Camera Z → IMU X
          -1, 0, 0,     # Camera X → IMU -Y
          0, -1, 0]     # Camera Y → IMU -Z

# Camera → IMU translation (in cm, approximate)
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.02, 0.0]  # Approximately 2cm offset
```

### 3.3 Time Synchronization

#### Time Offset on Smartphones

```
Smartphone characteristics:
- Camera and IMU are asynchronous
- Typical time offset: 10-50ms
- Varies by device/OS version
```

#### Settings

```yaml
# Required: Enable time offset estimation
estimate_td: 1

# Initial value (positive = camera lags behind IMU)
td: 0.02  # 20ms initial estimate

# Formula: corrected_image_time = original_image_time + td
```

#### Verifying Time Offset Estimation

```bash
# Check estimated td from ROS topic
rostopic echo /vins_estimator/td

# Normal range: -0.05 ~ 0.05 (±50ms)
# Values outside this range indicate data issues
```

### 3.4 Rolling Shutter Settings

#### What is Rolling Shutter?

```
Global Shutter: All pixels exposed simultaneously
Rolling Shutter: Sequential exposure row by row

    ┌─────────────────┐
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ ← t = 0
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ ← t = Δt
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ ← t = 2Δt
    │       ...       │
    │ ░░░░░░░░░░░░░░░ │ ← t = tr (readout time)
    └─────────────────┘
```

#### Smartphone Settings

```yaml
# Most smartphone cameras use Rolling Shutter
rolling_shutter: 1

# Readout time (refer to sensor datasheet)
# Typical range: 0.01 ~ 0.05 seconds
rolling_shutter_tr: 0.033  # Approximately 33ms (for 30fps)
```

**Rolling Shutter Readout Time Estimation**:
```
30fps camera: tr ≈ 1/30 = 0.033 seconds
60fps camera: tr ≈ 1/60 = 0.017 seconds
```

### 3.5 IMU Noise Parameters

#### Parameter Definitions

| Parameter | Meaning | Unit | Typical Smartphone Value |
|-----------|---------|------|--------------------------|
| `acc_n` | Accelerometer noise density | m/s²/√Hz | 0.05-0.2 |
| `gyr_n` | Gyroscope noise density | rad/s/√Hz | 0.005-0.02 |
| `acc_w` | Accelerometer bias random walk | m/s³/√Hz | 0.001-0.005 |
| `gyr_w` | Gyroscope bias random walk | rad/s²/√Hz | 1e-5 - 5e-4 |

#### Recommended Values by Smartphone Tier

```yaml
# Flagship smartphones (Pixel, iPhone Pro, Galaxy S)
acc_n: 0.08
gyr_n: 0.008
acc_w: 0.001
gyr_w: 5.0e-5

# Mid-range smartphones
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.002
gyr_w: 0.0002

# Budget smartphones (conservative settings)
acc_n: 0.2
gyr_n: 0.02
acc_w: 0.005
gyr_w: 0.0005
```

#### Converting Allan Variance Results

```python
# imu_utils output → VINS-Mono format conversion
# Note: VINS-Mono uses discrete time format

import math

def convert_to_vins(allan_result, imu_rate=200):
    """
    Convert Allan variance results to VINS-Mono format

    Args:
        allan_result: imu_utils output
        imu_rate: IMU sampling frequency (Hz)
    """
    dt = 1.0 / imu_rate

    # Continuous → Discrete conversion
    acc_n = allan_result['acc_noise_density'] / math.sqrt(dt)
    gyr_n = allan_result['gyro_noise_density'] / math.sqrt(dt)
    acc_w = allan_result['acc_random_walk'] * math.sqrt(dt)
    gyr_w = allan_result['gyro_random_walk'] * math.sqrt(dt)

    return {
        'acc_n': acc_n,
        'gyr_n': gyr_n,
        'acc_w': acc_w,
        'gyr_w': gyr_w
    }
```

**Simple method** (without precise calibration):
1. Start with default values
2. Increase noise values if drift is severe
3. Decrease noise values if tracking is unstable

### 3.6 Feature Tracking Parameters

#### Key Parameters

```yaml
# Maximum number of features (performance vs accuracy tradeoff)
max_cnt: 200       # Smartphone recommended: 150-250

# Minimum distance between features (pixels)
min_dist: 25       # 25-35 recommended

# RANSAC threshold (outlier rejection)
F_threshold: 1.0   # 1.0-2.0

# Output frequency (minimum 10Hz recommended)
freq: 10

# Low-light environment handling
equalize: 1        # Enable histogram equalization
```

#### Tuning by Environment

| Environment | max_cnt | min_dist | equalize |
|-------------|---------|----------|----------|
| Bright indoor | 150 | 30 | 0 |
| Dark indoor | 200 | 25 | 1 |
| Outdoor | 150 | 30 | 1 |
| Low texture | 250 | 20 | 1 |

### 3.7 Optimization Parameters

```yaml
# Solver time limit (real-time performance)
max_solver_time: 0.04      # 40ms (enables 25fps real-time)

# Maximum iterations
max_num_iterations: 8      # 8-12 range

# Keyframe selection criterion (parallax, pixels)
keyframe_parallax: 10.0    # 10-15 recommended
```

**Mobile/Low-spec PC**:
```yaml
max_solver_time: 0.06      # More relaxed setting
max_num_iterations: 6      # Reduce iterations
```

---

## 4. Smartphone-Specific Configuration Templates

### 4.1 Android Smartphone (General)

```yaml
%YAML:1.0

#------------------------------------------------------------------------------
# Android Smartphone Configuration for VINS-Mono
# Test environment: Samsung Galaxy S21, Android 12
#------------------------------------------------------------------------------

imu_topic: "/android/imu"
image_topic: "/android/camera/image_raw"
output_path: "/home/user/vins_android_output/"

# Camera (1280x720 @ 30fps)
model_type: PINHOLE
camera_name: "android_rear_camera"
image_width: 1280
image_height: 720

distortion_parameters:
   k1: -0.25
   k2: 0.06
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 920.0
   fy: 920.0
   cx: 640.0
   cy: 360.0

# Extrinsic parameters - Auto estimation
estimate_extrinsic: 2

extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]

extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

# Time synchronization - Required
estimate_td: 1
td: 0.025

# Rolling Shutter - Enabled
rolling_shutter: 1
rolling_shutter_tr: 0.033

# IMU parameters (mid-range smartphone)
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.002
gyr_w: 0.0002
g_norm: 9.81

# Feature tracking
max_cnt: 200
min_dist: 25
freq: 10
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0

# Optimization
max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

# Loop closure
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/user/vins_android_output/pose_graph/"

# Visualization
save_image: 1
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

### 4.2 iPhone (iOS)

```yaml
%YAML:1.0

#------------------------------------------------------------------------------
# iPhone Configuration for VINS-Mono
# Note: iOS recommends using VINS-Mobile
#------------------------------------------------------------------------------

imu_topic: "/iphone/imu"
image_topic: "/iphone/camera/image_raw"
output_path: "/home/user/vins_iphone_output/"

# Camera (1920x1080 @ 30fps)
model_type: PINHOLE
camera_name: "iphone_rear_camera"
image_width: 1920
image_height: 1080

distortion_parameters:
   k1: -0.20
   k2: 0.04
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 1400.0
   fy: 1400.0
   cx: 960.0
   cy: 540.0

# Extrinsic parameters
estimate_extrinsic: 2

extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]

extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

# Time synchronization
estimate_td: 1
td: 0.015  # iPhone has better synchronization

# Rolling Shutter
rolling_shutter: 1
rolling_shutter_tr: 0.033

# IMU parameters (iPhone has good quality)
acc_n: 0.08
gyr_n: 0.008
acc_w: 0.001
gyr_w: 5.0e-5
g_norm: 9.81

# Feature tracking
max_cnt: 180
min_dist: 30
freq: 10
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0

# Optimization
max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

# Loop closure
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/user/vins_iphone_output/pose_graph/"

# Visualization
save_image: 1
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

---

## 5. Troubleshooting

### 5.1 Initialization Failure

**Symptom**: System does not start or stays in "Initializing..." state

**Causes and Solutions**:

| Cause | Solution |
|-------|----------|
| Insufficient motion | **Rotate around various axes for 3-5 seconds** at startup |
| Insufficient features | Increase `max_cnt`, verify `equalize: 1` |
| Data start point issue | Use data starting 1-2 seconds after recording begins |
| IMU noise underestimated | Increase `acc_n`, `gyr_n` values |

```bash
# Check initialization state
rostopic echo /vins_estimator/vins_state
# 0: initializing, 1: initialized
```

### 5.2 Severe Drift

**Symptom**: Trajectory deviates significantly over time

**Checklist**:

```yaml
# 1. Check IMU noise parameters (too small causes drift)
acc_n: 0.1    # Try increasing
gyr_n: 0.01   # Try increasing

# 2. Verify time synchronization
estimate_td: 1  # Must be enabled

# 3. Verify extrinsic parameter estimation
estimate_extrinsic: 2  # or 1

# 4. Check feature count
max_cnt: 200   # Increase if insufficient
```

**Log verification**:
```bash
# Check IMU bias estimates
rostopic echo /vins_estimator/imu_bias

# Abnormal: |acc_bias| > 1.0 m/s² or |gyro_bias| > 0.1 rad/s
```

### 5.3 Tracking Lost

**Symptom**: Tracking suddenly stops or reinitializes

**Causes and Solutions**:

| Cause | Solution |
|-------|----------|
| Fast rotation/movement | Move more slowly |
| Low light | `equalize: 1`, improve lighting |
| Motion blur | Reduce exposure time |
| Low texture | Increase `max_cnt`, decrease `min_dist` |
| Rolling Shutter not set | Verify `rolling_shutter: 1` |

```yaml
# Settings for improved tracking stability
max_cnt: 250          # Increase features
min_dist: 20          # Allow denser distribution
F_threshold: 1.5      # Relax RANSAC
equalize: 1           # Histogram equalization
```

### 5.4 Abnormal Time Offset Estimation

**Symptom**: `td` value is abnormally large or unstable

**Normal range**: -0.05s ~ +0.05s (±50ms)

**Problem resolution**:
```yaml
# 1. Set initial value to reasonable range
td: 0.02  # Start with 20ms

# 2. Verify data quality
# - Camera: minimum 20Hz
# - IMU: minimum 100Hz (200Hz recommended)

# 3. Verify timestamps
rostopic echo /android/imu --noarr | head -20
rostopic echo /android/camera/image_raw --noarr | head -20
# Check that timestamps are monotonically increasing
```

### 5.5 Loop Closure Failure

**Symptom**: Loop does not close even when revisiting the same location

```yaml
# Verify loop closure settings
loop_closure: 1
fast_relocalization: 0  # Prioritize stability

# Feature settings (affects loop detection)
max_cnt: 200  # Sufficient features needed
```

---

## 6. Data Collection Recommendations

### 6.1 Sensor Requirements

| Sensor | Minimum | Recommended | Notes |
|--------|---------|-------------|-------|
| Camera FPS | 20 Hz | 30 Hz | Consistent frame rate |
| IMU sampling | 100 Hz | 200 Hz | Must be higher than camera |
| Resolution | 640x480 | 1280x720 | Too high slows processing |

### 6.2 Data Collection Tips

1. **At startup**: Wait 1-2 seconds in stationary state, then rotate around various axes
2. **While moving**: Avoid sudden movements, maintain constant speed
3. **Environment**: Sufficient texture, appropriate lighting
4. **At end**: Wait 1-2 seconds in stationary state

### 6.3 Android App Settings (Based on OpenCamera Sensors)

```
Camera settings:
- Resolution: 1280x720
- FPS: 30
- Exposure: Auto or fixed
- White balance: Fixed

IMU settings:
- Sampling rate: 200Hz
- Sensor type: UNCALIBRATED recommended
- Timestamp: Use elapsedRealtimeNanos
```

---

## 7. Execution and Verification

### 7.1 Execution Commands

```bash
# 1. Specify configuration file path
roslaunch vins_estimator vins_rviz.launch
roslaunch vins_estimator smartphone.launch \
  config_path:=/path/to/smartphone_config.yaml

# 2. Data playback (when using rosbag)
rosbag play smartphone_data.bag
```

### 7.2 Real-time Monitoring

```bash
# Tracking state
rostopic echo /vins_estimator/vins_state

# Pose output
rostopic echo /vins_estimator/odometry

# Time offset
rostopic echo /vins_estimator/td

# Feature count
rostopic echo /vins_estimator/feature_tracker/feature --noarr | grep -c "point"
```

### 7.3 Result Evaluation

```bash
# Trajectory saving
# Automatically saved to output_path

# Evaluation with EVO tool (if Ground Truth available)
evo_ape tum groundtruth.txt vins_result.txt -va --plot
```

---

## 8. References

### Official Documentation
- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Mono README](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/README.md)
- [VINS-Mobile (iOS)](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile)

### Configuration Examples
- [EuRoC Config](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/euroc/euroc_config.yaml)
- [TUM Config](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/tum/tum_config.yaml)

### Calibration Tools
- [Kalibr](https://github.com/ethz-asl/kalibr)
- [imu_utils](https://github.com/gaowenliang/imu_utils)

### Related Documents
- [05_vislam_frameworks.md](05_vislam_frameworks.md)
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)
- [13_imu_advanced_considerations.md](13_imu_advanced_considerations.md)
