# Basalt Smartphone Configuration Guide

## Overview

Basalt is a high-performance Visual-Inertial Odometry system developed by TUM (Technical University of Munich), featuring excellent performance particularly in **real-time processing speed** and **accuracy**. This document explains how to configure Basalt using smartphone camera and IMU data.

### Basalt Features

| Item | Description |
|------|-------------|
| **Processing Speed** | Highest level (suitable for real-time AR/VR) |
| **Camera Model** | Double Sphere (optimized for wide-angle lenses) |
| **Backend** | Square Root Marginalization |
| **Time Offset** | Can be estimated during calibration |
| **Frontend** | Optical Flow based |

### Considerations for Smartphone Use

| Item | Description |
|------|-------------|
| **Advantages** | Fast processing speed, low latency |
| **Disadvantages** | Requires Kalibr calibration conversion, no Rolling Shutter support |
| **Recommended Scenarios** | Real-time AR/VR, low-latency environments |

---

## Configuration File Structure

Basalt uses **two JSON files**:

1. **Calibration file** (`calibration.json`) - Camera intrinsics, IMU-Camera extrinsics
2. **Configuration file** (`config.json`) - VIO parameters, Optical Flow settings

---

## Calibration File (calibration.json)

### Basic Structure

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.0,
        "py": 0.0,
        "pz": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0
      }
    ],
    "intrinsics": [
      {
        "camera_type": "ds",
        "intrinsics": {
          "fx": 500.0,
          "fy": 500.0,
          "cx": 320.0,
          "cy": 240.0,
          "xi": -0.2,
          "alpha": 0.5
        }
      }
    ],
    "resolution": [[640, 480]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": 200.0,
    "accel_noise_std": [0.016, 0.016, 0.016],
    "gyro_noise_std": [0.000282, 0.000282, 0.000282],
    "accel_bias_std": [0.001, 0.001, 0.001],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "cam_time_offset_ns": 0
  }
}
```

### Camera Model Explanation

#### Double Sphere Model (ds)

Basalt's default camera model, optimized for wide-angle lenses.

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `fx`, `fy` | Focal length (pixels) | 400-1500 |
| `cx`, `cy` | Principal point coordinates (pixels) | Image center |
| `xi` | First sphere parameter | -0.5 ~ 0.5 |
| `alpha` | Second sphere parameter | 0.4 ~ 0.6 |

#### Pinhole Model (pinhole)

Used for standard smartphone cameras:

```json
{
  "camera_type": "pinhole",
  "intrinsics": {
    "fx": 500.0,
    "fy": 500.0,
    "cx": 320.0,
    "cy": 240.0
  }
}
```

#### Unified Camera Model (ucm)

Alternative for wide-angle cameras:

```json
{
  "camera_type": "ucm",
  "intrinsics": {
    "fx": 500.0,
    "fy": 500.0,
    "cx": 320.0,
    "cy": 240.0,
    "alpha": 0.5
  }
}
```

### IMU-Camera Transform (T_imu_cam)

`T_imu_cam` represents the transformation from IMU coordinate system to camera coordinate system.

```json
"T_imu_cam": [
  {
    "px": 0.005,      // x displacement [m]
    "py": 0.01,       // y displacement [m]
    "pz": -0.02,      // z displacement [m]
    "qx": 0.0,        // quaternion x
    "qy": 0.707,      // quaternion y
    "qz": 0.0,        // quaternion z
    "qw": 0.707       // quaternion w
  }
]
```

### Time Offset

```json
"cam_time_offset_ns": -5000000
```

- Unit: nanoseconds (ns)
- Meaning: `IMU time = Camera time + cam_time_offset_ns`
- Negative value: Image is ahead of IMU

---

## IMU Noise Parameters

### Recommended Values by Smartphone Grade

| Smartphone Grade | gyro_noise_std | accel_noise_std | gyro_bias_std | accel_bias_std |
|------------------|----------------|-----------------|---------------|----------------|
| **Flagship** | 0.001 | 0.02 | 0.0001 | 0.002 |
| **Mid-range** | 0.002 | 0.04 | 0.0002 | 0.004 |
| **Budget** | 0.005 | 0.08 | 0.0005 | 0.008 |

### Flagship Smartphone Example

```json
{
  "imu_update_rate": 200.0,
  "accel_noise_std": [0.02, 0.02, 0.02],
  "gyro_noise_std": [0.001, 0.001, 0.001],
  "accel_bias_std": [0.002, 0.002, 0.002],
  "gyro_bias_std": [0.0001, 0.0001, 0.0001]
}
```

### Mid-range Smartphone Example

```json
{
  "imu_update_rate": 100.0,
  "accel_noise_std": [0.04, 0.04, 0.04],
  "gyro_noise_std": [0.002, 0.002, 0.002],
  "accel_bias_std": [0.004, 0.004, 0.004],
  "gyro_bias_std": [0.0002, 0.0002, 0.0002]
}
```

---

## VIO Configuration File (config.json)

### Complete Configuration Structure

```json
{
  "config.optical_flow_type": "frame_to_frame",
  "config.optical_flow_detection_grid_size": 50,
  "config.optical_flow_max_recovered_dist2": 0.04,
  "config.optical_flow_pattern": 51,
  "config.optical_flow_max_iterations": 5,
  "config.optical_flow_epipolar_error": 0.005,
  "config.optical_flow_levels": 3,
  "config.optical_flow_skip_frames": 1,

  "config.vio_linearization_type": "ABS_QR",
  "config.vio_sqrt_marg": true,
  "config.vio_max_states": 3,
  "config.vio_max_kfs": 7,
  "config.vio_min_frames_after_kf": 5,
  "config.vio_new_kf_keypoints_thresh": 0.7,
  "config.vio_debug": false,
  "config.vio_extended_logging": false,

  "config.vio_obs_std_dev": 0.5,
  "config.vio_obs_huber_thresh": 1.0,
  "config.vio_min_triangulation_dist": 0.05,
  "config.vio_outlier_threshold": 3.0,
  "config.vio_filter_iteration": 4,
  "config.vio_max_iterations": 7,
  "config.vio_enforce_realtime": false,

  "config.vio_use_lm": true,
  "config.vio_lm_lambda_initial": 1e-4,
  "config.vio_lm_lambda_min": 1e-6,
  "config.vio_lm_lambda_max": 100.0,

  "config.vio_init_pose_weight": 1e8,
  "config.vio_init_ba_weight": 10.0,
  "config.vio_init_bg_weight": 100.0,

  "config.mapper_obs_std_dev": 0.25,
  "config.mapper_obs_huber_thresh": 1.5,
  "config.mapper_detection_num_points": 800,
  "config.mapper_num_frames_to_match": 30,
  "config.mapper_frames_to_match_threshold": 0.04,
  "config.mapper_min_triangulation_dist": 0.07,
  "config.mapper_no_factor_weights": false,
  "config.mapper_use_factors": true,
  "config.mapper_use_lm": true,
  "config.mapper_lm_lambda_min": 1e-32,
  "config.mapper_lm_lambda_max": 1e2
}
```

---

## Detailed Parameter Explanation

### Optical Flow Parameters

| Parameter | Default | Description | Tuning Guide |
|-----------|---------|-------------|--------------|
| `optical_flow_type` | frame_to_frame | Flow type | No change needed |
| `optical_flow_detection_grid_size` | 50 | Feature detection grid size | Smaller = more features |
| `optical_flow_max_recovered_dist2` | 0.04 | Bidirectional consistency check threshold | Lower if calibration is good |
| `optical_flow_pattern` | 51 | Patch pattern (24, 51, 52) | 51 recommended |
| `optical_flow_max_iterations` | 5 | Maximum iterations | 5-10 |
| `optical_flow_epipolar_error` | 0.005 | Epipolar error threshold | 0.01-0.05 |
| `optical_flow_levels` | 3 | Pyramid levels | 3-4 |
| `optical_flow_skip_frames` | 1 | Frame skip count | 1=all, 2=half |

### VIO Backend Parameters

| Parameter | Default | Description | Tuning Guide |
|-----------|---------|-------------|--------------|
| `vio_max_states` | 3 | Number of IMU-Pose states to maintain | 3-5 |
| `vio_max_kfs` | 7 | Maximum keyframes | 5-10 |
| `vio_min_frames_after_kf` | 5 | Minimum frames between keyframes | 3-10 |
| `vio_new_kf_keypoints_thresh` | 0.7 | New keyframe creation threshold | Higher = more frequent creation |
| `vio_obs_std_dev` | 0.5 | Observation standard deviation | 0.1-0.5 (lower = trust Flow more) |
| `vio_obs_huber_thresh` | 1.0 | Huber loss threshold | 1.0-5.0 |
| `vio_min_triangulation_dist` | 0.05 | Minimum triangulation distance [m] | Slightly smaller than baseline |
| `vio_outlier_threshold` | 3.0 | Outlier threshold | 2.0-4.0 |

### Initialization Weights

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vio_init_pose_weight` | 1e8 | Initial pose weight |
| `vio_init_ba_weight` | 10.0 | Accelerometer bias initial weight |
| `vio_init_bg_weight` | 100.0 | Gyroscope bias initial weight |

---

## Environment-Specific Configurations

### General Environment (Indoor/Outdoor)

```json
{
  "config.optical_flow_detection_grid_size": 50,
  "config.optical_flow_epipolar_error": 0.01,
  "config.optical_flow_skip_frames": 1,

  "config.vio_max_states": 3,
  "config.vio_max_kfs": 7,
  "config.vio_obs_std_dev": 0.5,
  "config.vio_new_kf_keypoints_thresh": 0.7
}
```

### Fast Motion Environment

```json
{
  "config.optical_flow_detection_grid_size": 40,
  "config.optical_flow_max_iterations": 7,
  "config.optical_flow_skip_frames": 1,

  "config.vio_max_states": 5,
  "config.vio_max_kfs": 10,
  "config.vio_obs_std_dev": 0.3,
  "config.vio_new_kf_keypoints_thresh": 0.8
}
```

### Low Light Environment

```json
{
  "config.optical_flow_detection_grid_size": 60,
  "config.optical_flow_epipolar_error": 0.02,
  "config.optical_flow_max_iterations": 7,

  "config.vio_obs_std_dev": 0.7,
  "config.vio_obs_huber_thresh": 2.0,
  "config.vio_outlier_threshold": 4.0
}
```

### Low Texture Environment

```json
{
  "config.optical_flow_detection_grid_size": 35,
  "config.optical_flow_epipolar_error": 0.02,

  "config.vio_min_triangulation_dist": 0.03,
  "config.vio_new_kf_keypoints_thresh": 0.6,
  "config.mapper_detection_num_points": 1200
}
```

---

## Converting from Kalibr to Basalt

### Conversion Script

Python script to convert Kalibr YAML output to Basalt JSON format:

```python
import yaml
import json
import numpy as np
from scipy.spatial.transform import Rotation

def kalibr_to_basalt(kalibr_cam_yaml, kalibr_imu_yaml, output_json):
    """Convert Kalibr calibration results to Basalt format"""

    # Load Kalibr calibration
    with open(kalibr_cam_yaml, 'r') as f:
        cam_calib = yaml.safe_load(f)

    with open(kalibr_imu_yaml, 'r') as f:
        imu_calib = yaml.safe_load(f)

    # Extract camera parameters
    cam0 = cam_calib['cam0']
    intrinsics = cam0['intrinsics']
    resolution = cam0['resolution']

    # Calculate T_imu_cam from T_cam_imu
    T_cam_imu = np.array(cam0['T_cam_imu']).reshape(4, 4)
    T_imu_cam = np.linalg.inv(T_cam_imu)

    # Convert rotation matrix to quaternion
    R = T_imu_cam[:3, :3]
    t = T_imu_cam[:3, 3]
    quat = Rotation.from_matrix(R).as_quat()  # [qx, qy, qz, qw]

    # Time offset (seconds -> nanoseconds)
    time_offset_s = cam_calib.get('cam0', {}).get('timeshift_cam_imu', 0.0)
    time_offset_ns = int(time_offset_s * 1e9)

    # IMU noise parameters
    imu0 = imu_calib['imu0']
    gyro_noise = imu0['gyroscope_noise_density']
    accel_noise = imu0['accelerometer_noise_density']
    gyro_bias = imu0['gyroscope_random_walk']
    accel_bias = imu0['accelerometer_random_walk']
    imu_rate = imu0['update_rate']

    # Construct Basalt format
    basalt_calib = {
        "value0": {
            "T_imu_cam": [{
                "px": float(t[0]),
                "py": float(t[1]),
                "pz": float(t[2]),
                "qx": float(quat[0]),
                "qy": float(quat[1]),
                "qz": float(quat[2]),
                "qw": float(quat[3])
            }],
            "intrinsics": [{
                "camera_type": "pinhole",  # or "ds"
                "intrinsics": {
                    "fx": intrinsics[0],
                    "fy": intrinsics[1],
                    "cx": intrinsics[2],
                    "cy": intrinsics[3]
                }
            }],
            "resolution": [[resolution[0], resolution[1]]],
            "calib_accel_bias": [0.0] * 9,
            "calib_gyro_bias": [0.0] * 12,
            "imu_update_rate": imu_rate,
            "accel_noise_std": [accel_noise] * 3,
            "gyro_noise_std": [gyro_noise] * 3,
            "accel_bias_std": [accel_bias] * 3,
            "gyro_bias_std": [gyro_bias] * 3,
            "cam_time_offset_ns": time_offset_ns
        }
    }

    with open(output_json, 'w') as f:
        json.dump(basalt_calib, f, indent=2)

    print(f"Basalt calibration saved to {output_json}")
    return basalt_calib


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--cam-yaml', required=True, help='Kalibr camera YAML')
    parser.add_argument('--imu-yaml', required=True, help='Kalibr IMU YAML')
    parser.add_argument('--output', required=True, help='Output Basalt JSON')
    args = parser.parse_args()

    kalibr_to_basalt(args.cam_yaml, args.imu_yaml, args.output)
```

### Usage

```bash
python kalibr2basalt.py \
    --cam-yaml camchain-imucam.yaml \
    --imu-yaml imu.yaml \
    --output smartphone_calib.json
```

---

## Calibration Workflow

### 1. Basalt Native Calibration

```bash
# Camera Calibration
basalt_calibrate \
    --dataset-path ~/calib_data/camera.bag \
    --dataset-type bag \
    --aprilgrid /usr/etc/basalt/aprilgrid_6x6.json \
    --result-path ~/calib_result/ \
    --cam-types ds

# IMU + Camera Calibration
basalt_calibrate_imu \
    --dataset-path ~/calib_data/imu_camera.bag \
    --dataset-type bag \
    --aprilgrid /usr/etc/basalt/aprilgrid_6x6.json \
    --result-path ~/calib_result/ \
    --gyro-noise-std 0.001 \
    --accel-noise-std 0.02 \
    --gyro-bias-std 0.0001 \
    --accel-bias-std 0.002
```

### 2. Using Kalibr Then Converting

```bash
# Kalibr Calibration
kalibr_calibrate_imu_camera \
    --target april_grid.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calibration.bag

# Convert to Basalt Format
python kalibr2basalt.py \
    --cam-yaml camchain-imucam.yaml \
    --imu-yaml imu.yaml \
    --output smartphone_calib.json
```

### AprilGrid Configuration

```json
{
  "tagCols": 6,
  "tagRows": 6,
  "tagSize": 0.088,
  "tagSpacing": 0.3
}
```

---

## Execution Method

### Basic Execution

```bash
# Run VIO
basalt_vio \
    --dataset-path /path/to/data/ \
    --cam-calib smartphone_calib.json \
    --dataset-type bag \
    --config-path smartphone_config.json \
    --show-gui 1

# Run Optical Flow Only (for debugging)
basalt_opt_flow \
    --dataset-path /path/to/data/ \
    --cam-calib smartphone_calib.json \
    --dataset-type bag \
    --config-path smartphone_config.json \
    --show-gui 1
```

### ROS Integration

```bash
# Real-time execution with ROS topics
roslaunch basalt basalt_vio.launch \
    cam_calib:=smartphone_calib.json \
    config:=smartphone_config.json
```

### Data Topics

```
/camera/image_raw  - Image (sensor_msgs/Image)
/imu/data          - IMU (sensor_msgs/Imu)
```

---

## Troubleshooting

### 1. "Not enough optical flow patches"

**Symptoms**: Tracking failure, insufficient features

**Solution**:
```json
{
  "config.optical_flow_detection_grid_size": 35,
  "config.optical_flow_epipolar_error": 0.02,
  "config.mapper_detection_num_points": 1200
}
```

### 2. Initialization Failure

**Symptoms**: VIO does not start

**Solution**:
```json
{
  "config.vio_init_pose_weight": 1e6,
  "config.vio_init_ba_weight": 1.0,
  "config.vio_init_bg_weight": 10.0
}
```

Also, provide sufficient movement (2-3 seconds in various directions).

### 3. Severe Drift

**Cause**: IMU parameter mismatch or calibration error

**Solution**:
```json
{
  "config.vio_obs_std_dev": 0.3,
  "config.vio_obs_huber_thresh": 2.0
}
```

Re-measure IMU noise parameters or increase the values.

### 4. Processing Speed Degradation

**Solution**:
```json
{
  "config.optical_flow_skip_frames": 2,
  "config.vio_max_states": 2,
  "config.vio_max_kfs": 5,
  "config.vio_max_iterations": 5
}
```

### 5. Time Synchronization Issues

**Symptoms**: Unstable pose estimation

**Solution**:
- Verify `cam_time_offset_ns` value
- Re-calibrate with `--time-calibration` option in Kalibr
- Negative value means image is ahead of IMU

```json
{
  "cam_time_offset_ns": -5000000
}
```

---

## Complete Configuration File Templates

### Flagship Smartphone Calibration

<details>
<summary>smartphone_flagship_calib.json (Click to expand)</summary>

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.005,
        "py": 0.01,
        "pz": -0.02,
        "qx": 0.0,
        "qy": 0.707,
        "qz": 0.0,
        "qw": 0.707
      }
    ],
    "intrinsics": [
      {
        "camera_type": "pinhole",
        "intrinsics": {
          "fx": 1000.0,
          "fy": 1000.0,
          "cx": 540.0,
          "cy": 960.0
        }
      }
    ],
    "resolution": [[1080, 1920]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": 200.0,
    "accel_noise_std": [0.02, 0.02, 0.02],
    "gyro_noise_std": [0.001, 0.001, 0.001],
    "accel_bias_std": [0.002, 0.002, 0.002],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "cam_time_offset_ns": 0
  }
}
```

</details>

### Flagship Smartphone VIO Configuration

<details>
<summary>smartphone_flagship_config.json (Click to expand)</summary>

```json
{
  "config.optical_flow_type": "frame_to_frame",
  "config.optical_flow_detection_grid_size": 50,
  "config.optical_flow_max_recovered_dist2": 0.04,
  "config.optical_flow_pattern": 51,
  "config.optical_flow_max_iterations": 5,
  "config.optical_flow_epipolar_error": 0.01,
  "config.optical_flow_levels": 3,
  "config.optical_flow_skip_frames": 1,

  "config.vio_linearization_type": "ABS_QR",
  "config.vio_sqrt_marg": true,
  "config.vio_max_states": 3,
  "config.vio_max_kfs": 7,
  "config.vio_min_frames_after_kf": 5,
  "config.vio_new_kf_keypoints_thresh": 0.7,
  "config.vio_debug": false,
  "config.vio_extended_logging": false,

  "config.vio_obs_std_dev": 0.5,
  "config.vio_obs_huber_thresh": 1.0,
  "config.vio_min_triangulation_dist": 0.05,
  "config.vio_outlier_threshold": 3.0,
  "config.vio_filter_iteration": 4,
  "config.vio_max_iterations": 7,
  "config.vio_enforce_realtime": false,

  "config.vio_use_lm": true,
  "config.vio_lm_lambda_initial": 1e-4,
  "config.vio_lm_lambda_min": 1e-6,
  "config.vio_lm_lambda_max": 100.0,

  "config.vio_init_pose_weight": 1e8,
  "config.vio_init_ba_weight": 10.0,
  "config.vio_init_bg_weight": 100.0,

  "config.mapper_obs_std_dev": 0.25,
  "config.mapper_obs_huber_thresh": 1.5,
  "config.mapper_detection_num_points": 800,
  "config.mapper_num_frames_to_match": 30,
  "config.mapper_frames_to_match_threshold": 0.04,
  "config.mapper_min_triangulation_dist": 0.07,
  "config.mapper_no_factor_weights": false,
  "config.mapper_use_factors": true,
  "config.mapper_use_lm": true,
  "config.mapper_lm_lambda_min": 1e-32,
  "config.mapper_lm_lambda_max": 1e2
}
```

</details>

### Mid-range Smartphone Calibration

<details>
<summary>smartphone_midrange_calib.json (Click to expand)</summary>

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.005,
        "py": 0.01,
        "pz": -0.02,
        "qx": 0.0,
        "qy": 0.707,
        "qz": 0.0,
        "qw": 0.707
      }
    ],
    "intrinsics": [
      {
        "camera_type": "pinhole",
        "intrinsics": {
          "fx": 600.0,
          "fy": 600.0,
          "cx": 320.0,
          "cy": 240.0
        }
      }
    ],
    "resolution": [[640, 480]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": 100.0,
    "accel_noise_std": [0.04, 0.04, 0.04],
    "gyro_noise_std": [0.002, 0.002, 0.002],
    "accel_bias_std": [0.004, 0.004, 0.004],
    "gyro_bias_std": [0.0002, 0.0002, 0.0002],
    "cam_time_offset_ns": 0
  }
}
```

</details>

---

## Framework Comparison

| Item | Basalt | VINS-Mono | OpenVINS | ORB-SLAM3 |
|------|--------|-----------|----------|-----------|
| **Processing Speed** | 5/5 | 3/5 | 3/5 | 3/5 |
| **Accuracy** | 4/5 | 4/5 | 4/5 | 5/5 |
| **Automatic Time Offset Estimation** | During calibration | Runtime | Runtime | No |
| **Rolling Shutter** | No | Yes | Yes | No |
| **Loop Closure** | No | Yes | No | Yes |
| **Smartphone Compatibility** | Good | Excellent | Excellent | Moderate |

### Recommended Scenarios for Basalt

| Scenario | Suitability | Reason |
|----------|-------------|--------|
| **Real-time AR/VR** | 5/5 | Highest processing speed |
| **Drones/Robots** | 5/5 | Low latency |
| **General Smartphone Use** | 3/5 | Calibration conversion required |
| **Offline Processing** | 3/5 | Other frameworks more suitable |

---

## References

### Official Documentation and Code
- [Basalt GitLab](https://gitlab.com/VladyslavUsenko/basalt)
- [Basalt GitHub Mirror](https://github.com/VladyslavUsenko/basalt)
- [Basalt Headers Documentation](https://vladyslavusenko.gitlab.io/basalt-headers/)
- [TUM CVG - Basalt](https://cvg.cit.tum.de/research/vslam/basalt)

### Papers
- [Basalt: Visual-Inertial Mapping with Non-Linear Factor Recovery](https://arxiv.org/abs/1904.06504)
- [The Double Sphere Camera Model](https://arxiv.org/abs/1807.08957)

### Tuning Guide
- [Basalt VIO Tuning & Configs](https://tlab-uav.github.io/tech-details/docs/research/vio/basalt-tuning)

### Calibration Tools
- [Kalibr](https://github.com/ethz-asl/kalibr) - Camera-IMU Calibration
- [OpenImuCameraCalibrator](https://github.com/urbste/OpenImuCameraCalibrator)

---

*Document Created: 2026-01-19*
*Version: 1.0.0*
