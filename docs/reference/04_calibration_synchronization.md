# Camera-IMU Calibration and Synchronization

## 1. Overview

In VI-SLAM systems, accurate calibration and time synchronization between camera and IMU have a decisive impact on system performance.

### Calibration Types

| Type | Description | Importance |
|------|-------------|------------|
| Spatial | Relative position/rotation between camera and IMU | Very High |
| Temporal | Time offset between camera and IMU | Very High |
| Intrinsic | Individual sensor parameters | High |

## 2. Camera Calibration

### 2.1 Camera Intrinsic Parameters

```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]

fx, fy: Focal length (in pixels)
cx, cy: Principal point
```

### 2.2 Lens Distortion Models

#### Radial Distortion
```
x_distorted = x(1 + k1*r² + k2*r⁴ + k3*r⁶)
y_distorted = y(1 + k1*r² + k2*r⁴ + k3*r⁶)
```

#### Tangential Distortion
```
x_distorted = x + [2*p1*xy + p2*(r² + 2x²)]
y_distorted = y + [p1*(r² + 2y²) + 2*p2*xy]
```

### 2.3 Calibration Tools

#### OpenCV
```python
import cv2
import numpy as np

# Chessboard corner detection
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

# Calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
```

#### Kalibr
```bash
# Camera calibration
rosrun kalibr kalibr_calibrate_cameras \
  --bag /path/to/bag \
  --topics /cam0/image_raw \
  --models pinhole-radtan \
  --target /path/to/target.yaml
```

## 3. IMU Calibration

### 3.1 IMU Intrinsic Parameters

#### Bias
```
Accelerometer bias: b_a = [bax, bay, baz]
Gyroscope bias: b_g = [bgx, bgy, bgz]
```

#### Noise Characteristics
```
Accelerometer noise density: σ_a (m/s²/√Hz)
Gyroscope noise density: σ_g (rad/s/√Hz)
Accelerometer random walk: σ_ba (m/s³/√Hz)
Gyroscope random walk: σ_bg (rad/s²/√Hz)
```

### 3.2 IMU Noise Parameter Estimation

#### Allan Variance Analysis
```python
# Using imu_utils
# https://github.com/gaowenliang/imu_utils

roslaunch imu_utils imu.launch
# Requires 2+ hours of stationary IMU data collection
```

#### Typical MEMS IMU Noise Parameters
```yaml
# Low-cost smartphone IMU example
accelerometer_noise_density: 0.01     # m/s²/√Hz
accelerometer_random_walk: 0.0002    # m/s³/√Hz
gyroscope_noise_density: 0.005       # rad/s/√Hz
gyroscope_random_walk: 4.0e-06       # rad/s²/√Hz
```

## 4. Camera-IMU Extrinsic Calibration

### 4.1 Transformation Matrix

```
T_cam_imu = [R_cam_imu | t_cam_imu]
            [    0     |     1    ]

R_cam_imu: Rotation from IMU to camera (3x3)
t_cam_imu: Translation from IMU to camera (3x1)
```

### 4.2 Calibration with Kalibr

#### Requirements
- Aprilgrid or chessboard target
- Calibrated camera
- IMU data

#### Calibration Process
```bash
# Camera-IMU calibration
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /path/to/bag \
  --cam /path/to/cam_chain.yaml \
  --imu /path/to/imu.yaml \
  --target /path/to/target.yaml
```

#### Recommended Settings
- Camera: 20Hz
- IMU: 200Hz
- Motion: Excitation in all 3 axes
- Duration: 60-120 seconds

### 4.3 Calibration Quality Verification

```
Good calibration indicators:
- Reprojection error: < 0.5 pixels
- Gyro error: < 0.01 rad/s
- Accel error: < 0.1 m/s²
```

## 5. Time Synchronization (Temporal Calibration)

### 5.1 Impact of Time Offset

```
Impact per 1ms offset (at typical motion speed):
- Rotation 1 rad/s → 1 mrad error
- Translation 1 m/s → 1 mm error

Larger errors occur during fast motion
```

### 5.2 Synchronization Methods

#### Hardware Synchronization (Best)
```
Method: Use external trigger to synchronize camera shutter and IMU sampling
Accuracy: < 1ms
Requirements: Hardware modification required

[Trigger Signal] ──┬──→ [Camera]
                   └──→ [IMU]
```

#### Software Synchronization
```
Method: Use system clock timestamps
Accuracy: 1-10ms (OS/platform dependent)
Advantages: No hardware modification required
```

#### Online Temporal Calibration
```
Method: Algorithm estimates time offset during runtime
Supported algorithms:
- VINS-Mono: Supported (automatic estimation)
- ORB-SLAM3: Not supported (pre-synchronization required)
- OpenVINS: Supported
```

### 5.3 Time Offset Estimation Methods

#### Angular Velocity Correlation
```python
# Correlation analysis between camera-estimated angular velocity and IMU angular velocity
def estimate_time_offset(camera_angular_vel, imu_angular_vel, timestamps):
    # Initial offset estimation via cross-correlation
    correlation = np.correlate(camera_angular_vel, imu_angular_vel, mode='full')
    offset_samples = np.argmax(correlation) - len(camera_angular_vel) + 1

    # Sub-sample accuracy improvement via quadratic fitting
    # ...
    return time_offset
```

#### Kalibr Temporal Calibration
```bash
# Estimate time offset together
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /path/to/bag \
  --cam cam.yaml \
  --imu imu.yaml \
  --target target.yaml \
  --time-calibration
```

### 5.4 Typical Time Offset Values

| Platform | Typical Offset | Variability |
|----------|----------------|-------------|
| High-end sensors (RealSense) | < 1ms | Low |
| Smartphone (high-end) | 5-20ms | Medium |
| Smartphone (budget) | 10-50ms | High |
| Custom assembly | Variable | Very High |

## 6. Smartphone-Specific Considerations

### 6.1 Rolling Shutter Correction

Most smartphone cameras use Rolling Shutter:

```
Global Shutter: All pixels exposed simultaneously
Rolling Shutter: Row-by-row sequential exposure

Rolling Shutter Skew: Time difference between first and last row
Typical value: 10-30ms (sensor dependent)
```

#### Correction Methods
- VINS-Mono: Rolling Shutter model supported
- ORB-SLAM3: Rolling Shutter not supported
- OpenVINS: Rolling Shutter supported

### 6.2 OIS (Optical Image Stabilization)

```
Issues:
- Optical parameters change per frame
- Cannot be disabled on most smartphones

Solutions:
- Use devices that provide OIS data (some Android)
- Use camera without OIS (separate external camera)
- Reflect OIS changes in calibration
```

## 7. Calibration Tools Summary

| Tool | Platform | Features | Link |
|------|----------|----------|------|
| Kalibr | ROS | Camera + IMU + Time | [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) |
| OpenCV | Cross | Camera intrinsics | [opencv.org](https://opencv.org/) |
| imu_utils | ROS | IMU noise | [gaowenliang/imu_utils](https://github.com/gaowenliang/imu_utils) |
| OpenVINS Calibration | ROS | Full pipeline | [docs.openvins.com](https://docs.openvins.com/gs-calibration.html) |

## 8. Calibration Workflow

```
1. Camera Intrinsic Calibration
   └─→ OpenCV or Kalibr

2. IMU Intrinsic Calibration (optional)
   └─→ imu_utils (Allan Variance)

3. Camera-IMU Extrinsic Calibration
   └─→ Kalibr

4. Temporal Calibration
   └─→ Kalibr (offline) or VINS-Mono (online)

5. Verification
   └─→ Check reprojection error, trajectory accuracy
```

## References

- [Kalibr Camera-IMU Calibration Wiki](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [Online Temporal Calibration Paper](https://arxiv.org/pdf/1808.00692)
- [Camera-IMU Time Offset Modeling (ECCV 2018)](https://openaccess.thecvf.com/content_ECCV_2018/papers/Yonggen_Ling_Modeling_Varying_Camera-IMU_ECCV_2018_paper.pdf)
- [Hardware Synchronization Project](https://github.com/TurtleZhong/Visual-Inertial-Synchronization-Hardware)
