# ORB-SLAM3 Smartphone Configuration Guide

## Overview

ORB-SLAM3 is one of the most accurate open-source SLAM libraries supporting Visual, Visual-Inertial, and Multi-Map SLAM. This document explains how to configure ORB-SLAM3's Mono-Inertial mode using smartphone camera and IMU data.

### ORB-SLAM3 Features

| Item | Description |
|------|-------------|
| **Supported Modes** | Mono, Stereo, RGB-D, Mono-Inertial, Stereo-Inertial |
| **Lens Models** | Pinhole, Fisheye |
| **Advantages** | Highest accuracy, Loop Closure, Multi-Map support |
| **Disadvantages** | No automatic time offset estimation, No Rolling Shutter support |

### Limitations When Using Smartphones

**IMPORTANT**: Unlike VINS-Mono/OpenVINS, ORB-SLAM3 **does not automatically estimate camera-IMU time offset**. Therefore:

1. **Accurate calibration is essential**: Pre-measure accurate time offset using Kalibr or similar tools
2. **Data preprocessing**: Correct timestamps based on measured offset before input
3. **No Rolling Shutter support**: Global Shutter cameras or slow movements recommended

---

## Configuration File Structure

ORB-SLAM3 configuration consists of a single YAML file.

```yaml
# smartphone_mono_inertial.yaml

%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters (Smartphone Camera Parameters)
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

# Camera Intrinsic Parameters (Calibrated with Kalibr or OpenCV)
Camera1.fx: 500.0
Camera1.fy: 500.0
Camera1.cx: 320.0
Camera1.cy: 240.0

# Distortion Coefficients (Radial-Tangential)
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

# Image Size
Camera.width: 640
Camera.height: 480

# New Image Size for Rectification (After Distortion Removal)
Camera.newWidth: 640
Camera.newHeight: 480

# Camera Frame Rate
Camera.fps: 30

# RGB Image Flag (0: BGR, 1: RGB)
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters (Smartphone IMU Parameters)
#--------------------------------------------------------------------------------------------

# IMU-Camera Transformation Matrix (Tbc: Camera frame -> Body/IMU frame)
# 4x4 Homogeneous Transformation Matrix in SE(3)
# Use T_cam_imu output from Kalibr
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0,  0.0,  1.0,  0.0,
          -1.0,  0.0,  0.0,  0.0,
           0.0, -1.0,  0.0,  0.0,
           0.0,  0.0,  0.0,  1.0]

# IMU Noise Parameters (Continuous Time Basis)
# Units: Gyro [rad/s/sqrt(Hz)], Acc [m/s^2/sqrt(Hz)]
IMU.NoiseGyro: 1.7e-4      # Gyroscope white noise
IMU.NoiseAcc: 2.0e-3       # Accelerometer white noise

# IMU Random Walk Parameters
# Units: Gyro [rad/s^2/sqrt(Hz)], Acc [m/s^3/sqrt(Hz)]
IMU.GyroWalk: 1.9e-5       # Gyroscope bias random walk
IMU.AccWalk: 3.0e-3        # Accelerometer bias random walk

# IMU Sampling Frequency [Hz]
IMU.Frequency: 200.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters (Feature Extraction)
#--------------------------------------------------------------------------------------------

# Number of ORB Features per Frame
ORBextractor.nFeatures: 1200

# Image Pyramid Scale Factor
ORBextractor.scaleFactor: 1.2

# Number of Image Pyramid Levels
ORBextractor.nLevels: 8

# FAST Feature Detection Thresholds
ORBextractor.iniThFAST: 20    # Initial threshold
ORBextractor.minThFAST: 7     # Minimum threshold (used when features are insufficient)

#--------------------------------------------------------------------------------------------
# Viewer Parameters (Visualization)
#--------------------------------------------------------------------------------------------

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

---

## IMU Noise Parameters for Different Smartphones

### Recommended Noise Parameters

Smartphone MEMS IMUs have higher noise than industrial IMUs, so it is common to use values **10 times or more larger** than manufacturer specifications.

| Smartphone Grade | NoiseGyro | NoiseAcc | GyroWalk | AccWalk |
|------------------|-----------|----------|----------|---------|
| **Flagship** (iPhone 15, Galaxy S24) | 1.0e-3 | 1.5e-2 | 1.0e-4 | 3.0e-3 |
| **Mid-range** | 2.0e-3 | 3.0e-2 | 2.0e-4 | 5.0e-3 |
| **Budget** | 5.0e-3 | 5.0e-2 | 5.0e-4 | 1.0e-2 |

### Parameter Tuning Guide

```yaml
# Flagship Smartphone Recommended Settings
IMU.NoiseGyro: 1.0e-3
IMU.NoiseAcc: 1.5e-2
IMU.GyroWalk: 1.0e-4
IMU.AccWalk: 3.0e-3
IMU.Frequency: 200.0

# Mid-range Smartphone Recommended Settings
IMU.NoiseGyro: 2.0e-3
IMU.NoiseAcc: 3.0e-2
IMU.GyroWalk: 2.0e-4
IMU.AccWalk: 5.0e-3
IMU.Frequency: 100.0
```

**Tip**: If ORB-SLAM3 initialization is not working well, try increasing `GyroWalk` and `AccWalk` by 2-10 times.

---

## Smartphone Camera Configuration

### Pinhole Model (Standard Smartphones)

```yaml
Camera.type: "PinHole"

# Typical Smartphone Camera Parameters Example (1080p)
Camera1.fx: 1000.0
Camera1.fy: 1000.0
Camera1.cx: 540.0
Camera1.cy: 960.0

# Distortion Coefficients (Use Kalibr Calibration Results)
Camera1.k1: -0.1
Camera1.k2: 0.05
Camera1.p1: 0.001
Camera1.p2: -0.001

Camera.width: 1080
Camera.height: 1920
Camera.fps: 30
```

### Fisheye Model (Wide-Angle Cameras)

Some smartphone wide-angle cameras use Fisheye model:

```yaml
Camera.type: "KannalaBrandt8"

# Fisheye Distortion Coefficients
Camera1.k1: 0.5
Camera1.k2: -0.1
Camera1.k3: 0.05
Camera1.k4: -0.01
```

---

## IMU-Camera Transformation Matrix (Tbc)

### Concept Explanation

`IMU.T_b_c1` is the transformation matrix from Camera coordinate system to Body (IMU) coordinate system.

```
Coordinate System Definitions:
- Camera (C): zC points along optical axis (forward), yC points down, xC points right
- Body/IMU (B): Follows sensor manufacturer definition
```

### Obtaining Tbc from Kalibr

```bash
# Run Kalibr Calibration
kalibr_calibrate_imu_camera \
    --target april_grid.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calibration.bag

# Check T_cam_imu in result file
# Kalibr output: T_cam_imu (Camera <- IMU)
# ORB-SLAM3 requires: T_b_c1 = T_imu_cam = inv(T_cam_imu)
```

### Transformation Matrix Conversion

Convert Kalibr output to ORB-SLAM3 format:

```python
import numpy as np

# Kalibr output (T_cam_imu)
T_cam_imu = np.array([
    [r11, r12, r13, tx],
    [r21, r22, r23, ty],
    [r31, r32, r33, tz],
    [0,   0,   0,   1]
])

# ORB-SLAM3 required (T_b_c1 = T_imu_cam)
T_b_c1 = np.linalg.inv(T_cam_imu)

# Output in YAML format
print("IMU.T_b_c1: !!opencv-matrix")
print("   rows: 4")
print("   cols: 4")
print("   dt: f")
print(f"   data: {T_b_c1.flatten().tolist()}")
```

### Typical Smartphone Tbc Example

```yaml
# Rear Camera + IMU (Typical Configuration)
# When Camera is Mounted in Portrait Mode
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, -1.0,  0.0,  0.0,
           1.0,  0.0,  0.0,  0.0,
           0.0,  0.0,  1.0,  0.0,
           0.0,  0.0,  0.0,  1.0]
```

---

## ORB Feature Extraction Optimization

### Recommended Settings for Smartphone Environment

```yaml
# General Environment (Indoor/Outdoor)
ORBextractor.nFeatures: 1200      # Ensure sufficient features
ORBextractor.scaleFactor: 1.2     # Standard scale factor
ORBextractor.nLevels: 8           # Handle various scales

# FAST Thresholds (Lower in low-texture environments)
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

### Environment-Specific Parameter Adjustment

| Environment | nFeatures | iniThFAST | minThFAST | Description |
|-------------|-----------|-----------|-----------|-------------|
| **General** | 1200 | 20 | 7 | Default settings |
| **Low Texture** | 1500 | 15 | 5 | Increase feature detection sensitivity |
| **Fast Motion** | 1500 | 20 | 7 | More features for tracking stability |
| **Low Light** | 1000 | 12 | 5 | Lower threshold in noisy environment |

---

## Time Synchronization Handling

### Key Considerations

ORB-SLAM3 **does not support automatic time offset estimation**. Therefore, the following procedures are essential:

### 1. Measure Time Offset with Kalibr

```bash
# Set initial time offset value in imu.yaml
# timeshift_cam_imu: 0.0  # [s] initial value

kalibr_calibrate_imu_camera \
    --target april_grid.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calibration.bag \
    --time-calibration  # Also estimate time offset
```

### 2. Pre-correct Timestamps

```python
import numpy as np

class TimestampCorrector:
    def __init__(self, time_offset_cam_imu):
        """
        time_offset_cam_imu: t_imu = t_cam + time_offset
        """
        self.offset = time_offset_cam_imu

    def correct_image_timestamp(self, img_timestamp):
        """Correct image timestamp to IMU time reference"""
        return img_timestamp + self.offset

    def prepare_data_for_orbslam3(self, images, imus, time_offset):
        """Preprocess data before ORB-SLAM3 input"""
        corrector = TimestampCorrector(time_offset)

        corrected_images = []
        for img_ts, img_data in images:
            corrected_ts = corrector.correct_image_timestamp(img_ts)
            corrected_images.append((corrected_ts, img_data))

        return corrected_images, imus  # IMU stays unchanged
```

### 3. Synchronization During Data Input

```cpp
// When passing data to ORB-SLAM3, timestamps must be aligned
// Pass all IMU data before the image timestamp

// Note: Image timestamp must be smaller than IMU timestamp
// ERROR: "Frame with a timestamp older than previous frame detected!"
// If this error occurs, check timestamp order
```

---

## Calibration Workflow

### Complete Process

```
1. Camera Intrinsic Calibration
   └── Use OpenCV or Kalibr

2. IMU Intrinsic Calibration
   └── Allan Variance Analysis (imu_utils)

3. Camera-IMU Extrinsic Calibration
   └── Kalibr kalibr_calibrate_imu_camera

4. Time Offset Measurement
   └── Kalibr --time-calibration

5. Configuration File Generation and Verification
   └── Create config files and test
```

### Kalibr Calibration Files

**april_grid.yaml** (Calibration Target):
```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088      # Tag size [m]
tagSpacing: 0.3     # Tag spacing ratio
```

**cam.yaml** (Camera Initial Settings):
```yaml
cam0:
  camera_model: pinhole
  intrinsics: [500, 500, 320, 240]  # fx, fy, cx, cy
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [640, 480]
  rostopic: /camera/image_raw
```

**imu.yaml** (IMU Settings):
```yaml
imu0:
  accelerometer_noise_density: 0.015      # m/s^2/sqrt(Hz)
  accelerometer_random_walk: 0.003        # m/s^3/sqrt(Hz)
  gyroscope_noise_density: 0.001          # rad/s/sqrt(Hz)
  gyroscope_random_walk: 0.0001           # rad/s^2/sqrt(Hz)
  rostopic: /imu/data
  update_rate: 200.0
```

---

## Execution Method

### ROS Environment

```bash
# 1. Build ORB-SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# 2. Build ROS Workspace
cd Examples/ROS/ORB_SLAM3
mkdir build && cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4

# 3. Run
rosrun ORB_SLAM3 Mono_Inertial \
    /path/to/ORBvoc.txt \
    /path/to/smartphone_mono_inertial.yaml \
    true  # Visualization
```

### Data Input Topics

```
/camera/image_raw     - Image (sensor_msgs/Image)
/imu/data            - IMU (sensor_msgs/Imu)
```

---

## Android App Integration

### ORB-SLAM3 AR Android Project

Refer to [ORB_SLAM3_AR-for-Android](https://github.com/Abonaventure/ORB_SLAM3_AR-for-Android):

```
Required Files:
/sdcard/SLAM/
├── ORBvoc.bin           # Vocabulary (Binary format)
└── PARAconfig.yaml      # Camera/IMU configuration file
```

### Important Notes

- Camera calibration is essential for each smartphone
- Initialize in texture-rich environments
- Initialization may take some time

---

## Troubleshooting

### 1. IMU Initialization Failure

**Symptoms**: "IMU initialization failed" or slow convergence

**Cause and Solution**:
```yaml
# 1. Increase Random Walk values (10x)
IMU.GyroWalk: 1.0e-3    # 10x from original
IMU.AccWalk: 3.0e-2     # 10x from original

# 2. Verify IMU frequency
IMU.Frequency: 200.0    # Must match actual collection frequency

# 3. Provide sufficient movement during initialization
# - Rotate in various directions
# - Move for 2-3 seconds then pause briefly
```

### 2. "Frame with timestamp older than previous frame"

**Cause**: Timestamp order mismatch

**Solution**:
```python
# Verify timestamp ordering
def check_timestamp_order(images, imus):
    prev_img_ts = 0
    for img_ts, _ in images:
        if img_ts < prev_img_ts:
            print(f"ERROR: Image timestamp disorder at {img_ts}")
        prev_img_ts = img_ts

    prev_imu_ts = 0
    for imu_ts, _ in imus:
        if imu_ts < prev_imu_ts:
            print(f"ERROR: IMU timestamp disorder at {imu_ts}")
        prev_imu_ts = imu_ts
```

### 3. Severe Drift

**Cause**: IMU parameter mismatch or calibration error

**Solution**:
```yaml
# 1. Re-measure IMU noise
# Use Allan Variance analysis to identify actual noise characteristics

# 2. Verify Tbc matrix
# Re-calibrate with Kalibr or manual adjustment

# 3. Re-verify time offset
# Accurate time synchronization is critical
```

### 4. Feature Tracking Failure

**Symptoms**: Frequent tracking loss

**Solution**:
```yaml
# Increase feature count
ORBextractor.nFeatures: 1500

# Decrease FAST thresholds (detect more features)
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

# Check image resolution
# Too low resolution degrades feature quality
```

### 5. Segmentation Fault

**Cause**: Incorrect configuration file format

**Solution**:
```yaml
# Verify YAML format
# Must follow opencv-matrix format exactly

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f      # float type
   data: [...]  # 16 elements
```

---

## VINS-Mono vs ORB-SLAM3 Comparison

| Item | VINS-Mono | ORB-SLAM3 |
|------|-----------|-----------|
| **Automatic Time Offset Estimation** | Supported | Not Supported |
| **Rolling Shutter** | Supported | Not Supported |
| **Loop Closure** | Supported | Supported (Stronger) |
| **Multi-Map** | Not Supported | Supported |
| **Accuracy** | High | Highest Level |
| **Smartphone Compatibility** | Excellent | Moderate (Calibration Important) |

### Recommended Selection for Smartphone Use

| Situation | Recommended Framework |
|-----------|----------------------|
| **Accurate calibration possible** | ORB-SLAM3 |
| **Rapid prototyping** | VINS-Mono |
| **Rolling Shutter camera** | VINS-Mono |
| **Highest accuracy required** | ORB-SLAM3 |
| **Offline processing** | ORB-SLAM3 |

---

## Complete Configuration File Templates

### For Flagship Smartphones

<details>
<summary>smartphone_flagship.yaml (Click to expand)</summary>

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters - Flagship Smartphone (e.g., iPhone 15, Galaxy S24)
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

Camera1.fx: 1500.0
Camera1.fy: 1500.0
Camera1.cx: 540.0
Camera1.cy: 960.0

Camera1.k1: -0.1
Camera1.k2: 0.05
Camera1.p1: 0.001
Camera1.p2: -0.001

Camera.width: 1080
Camera.height: 1920
Camera.newWidth: 1080
Camera.newHeight: 1920
Camera.fps: 30
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters - Flagship Grade MEMS IMU
#--------------------------------------------------------------------------------------------

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, -1.0,  0.0,  0.0,
           1.0,  0.0,  0.0,  0.0,
           0.0,  0.0,  1.0,  0.0,
           0.0,  0.0,  0.0,  1.0]

IMU.NoiseGyro: 1.0e-3
IMU.NoiseAcc: 1.5e-2
IMU.GyroWalk: 1.0e-4
IMU.AccWalk: 3.0e-3
IMU.Frequency: 200.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters
#--------------------------------------------------------------------------------------------

ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

</details>

### For Mid-range Smartphones

<details>
<summary>smartphone_midrange.yaml (Click to expand)</summary>

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters - Mid-range Smartphone
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

Camera1.fx: 800.0
Camera1.fy: 800.0
Camera1.cx: 320.0
Camera1.cy: 240.0

Camera1.k1: -0.15
Camera1.k2: 0.08
Camera1.p1: 0.002
Camera1.p2: -0.001

Camera.width: 640
Camera.height: 480
Camera.newWidth: 640
Camera.newHeight: 480
Camera.fps: 30
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters - Mid-range MEMS IMU (Higher noise)
#--------------------------------------------------------------------------------------------

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, -1.0,  0.0,  0.0,
           1.0,  0.0,  0.0,  0.0,
           0.0,  0.0,  1.0,  0.0,
           0.0,  0.0,  0.0,  1.0]

IMU.NoiseGyro: 2.0e-3
IMU.NoiseAcc: 3.0e-2
IMU.GyroWalk: 2.0e-4
IMU.AccWalk: 5.0e-3
IMU.Frequency: 100.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters (More features for lower quality images)
#--------------------------------------------------------------------------------------------

ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

</details>

---

## References

### Official Documentation and Code
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [Calibration Tutorial PDF](http://git.autolabor.com.cn/12345qiupeng/orb_slam3_details/raw/commit/e6e28a86a5e35de35fb3022c881328b88f2bacd2/Calibration_Tutorial.pdf)

### Calibration Tools
- [Kalibr](https://github.com/ethz-asl/kalibr) - Camera-IMU Calibration
- [imu_utils](https://github.com/gaowenliang/imu_utils) - IMU Allan Variance Analysis

### Related Projects
- [ORB_SLAM3_AR-for-Android](https://github.com/Abonaventure/ORB_SLAM3_AR-for-Android) - Android AR Implementation
- [Camera IMU time offset Issue #78](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/78) - Time Offset Discussion

---

*Document Created: 2026-01-19*
*Version: 1.0.0*
