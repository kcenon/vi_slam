# Calibration Practical Guide

## Overview

This document is a practical guide for calibrating smartphone cameras and IMUs for use in VI-SLAM. It provides step-by-step instructions for the complete workflow including camera intrinsic calibration, IMU noise parameter measurement, and Camera-IMU extrinsic calibration.

### Complete Calibration Workflow

```
+----------------------------------------------------------------+
|                    Calibration Workflow                         |
+----------------------------------------------------------------+
|                                                                |
|  Step 1: Prepare Calibration Target                            |
|          +-- Print AprilGrid or Checkerboard                   |
|                    |                                           |
|  Step 2: Camera Intrinsic Calibration                          |
|          +-- Use OpenCV or Kalibr                              |
|                    |                                           |
|  Step 3: Measure IMU Noise Parameters                          |
|          +-- Allan Variance Analysis (imu_utils)               |
|                    |                                           |
|  Step 4: Camera-IMU Extrinsic Calibration                      |
|          +-- Kalibr kalibr_calibrate_imu_camera                |
|                    |                                           |
|  Step 5: Validate Results and Apply to Frameworks              |
|          +-- Generate config files and test                    |
|                                                                |
+----------------------------------------------------------------+
```

---

## Step 1: Prepare Calibration Target

### Target Type Comparison

| Target Type | Pros | Cons | Recommended Use |
|-------------|------|------|-----------------|
| **AprilGrid** | Partial visibility OK, no pose flip | Complex detection | Kalibr recommended |
| **Checkerboard** | Simple, OpenCV native support | Flip possible due to symmetry | OpenCV standalone use |
| **ChArUco** | AprilTag + Checkerboard benefits | Complex setup | High precision needed |

### AprilGrid Generation and Printing

#### Generate AprilGrid with Kalibr

```bash
# Generate 6x6 AprilGrid (A0 size recommended)
kalibr_create_target_pdf \
    --type apriltag \
    --nx 6 \
    --ny 6 \
    --tsize 0.088 \
    --tspace 0.3 \
    --output aprilgrid_6x6.pdf
```

**Parameter Description:**
- `--nx`, `--ny`: Number of tag columns/rows
- `--tsize`: Tag size (meters)
- `--tspace`: Tag spacing ratio (relative to tsize)

#### AprilGrid Configuration File (aprilgrid.yaml)

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088        # Tag size [m] - Update with actual measured value after printing
tagSpacing: 0.3       # Spacing ratio (space = tagSize * tagSpacing)
```

### Checkerboard Generation

#### Generate Checkerboard with Kalibr

```bash
kalibr_create_target_pdf \
    --type checkerboard \
    --nx 9 \
    --ny 6 \
    --csx 0.03 \
    --csy 0.03 \
    --output checkerboard_9x6.pdf
```

#### Checkerboard Configuration File (checkerboard.yaml)

```yaml
target_type: 'checkerboard'
targetCols: 9          # Number of internal corner columns
targetRows: 6          # Number of internal corner rows
rowSpacingMeters: 0.03 # Row spacing [m]
colSpacingMeters: 0.03 # Column spacing [m]
```

### Printing and Installation Tips

1. **Accurate Printing**
   - A0 or A1 size recommended
   - Select "Print at actual size" option
   - **Measure actual size after printing** - essential

2. **Sturdy Installation**
   - Attach to flat, rigid board
   - Prevent wrinkles or warping
   - Foam board, acrylic, or aluminum board recommended

3. **Size Measurement**
   ```python
   # Update config file with actual measured values
   measured_tag_size = 0.0875  # Measured value [m]
   ```

---

## Step 2: Camera Intrinsic Calibration

### Method 1: Using OpenCV (Simple, Fast)

#### Python Calibration Script

```python
#!/usr/bin/env python3
"""
camera_calibration_opencv.py
Smartphone Camera Intrinsic Calibration
"""

import cv2
import numpy as np
import glob
import yaml
import os

class CameraCalibrator:
    def __init__(self, checkerboard_size=(9, 6), square_size=0.03):
        """
        Args:
            checkerboard_size: (cols, rows) Number of internal corners
            square_size: Size of one square side [m]
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size

        # Prepare 3D world coordinates
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3),
                             np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0],
                                     0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size

        self.obj_points = []  # 3D points
        self.img_points = []  # 2D points
        self.image_size = None

    def add_images(self, image_paths, show_corners=False):
        """Detect checkerboard corners from images"""
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001)

        successful = 0
        for path in image_paths:
            img = cv2.imread(path)
            if img is None:
                print(f"Cannot read: {path}")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if self.image_size is None:
                self.image_size = gray.shape[::-1]

            # Find corners
            ret, corners = cv2.findChessboardCorners(
                gray, self.checkerboard_size, None)

            if ret:
                # Refine corners to subpixel accuracy
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)

                self.obj_points.append(self.objp)
                self.img_points.append(corners2)
                successful += 1

                if show_corners:
                    cv2.drawChessboardCorners(
                        img, self.checkerboard_size, corners2, ret)
                    cv2.imshow('Corners', cv2.resize(img, (640, 480)))
                    cv2.waitKey(500)

        print(f"Detected corners in {successful}/{len(image_paths)} images")
        return successful

    def calibrate(self):
        """Run calibration"""
        if len(self.obj_points) < 10:
            raise ValueError("Need at least 10 images with detected corners")

        print("Calibrating...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            self.image_size,
            None,
            None
        )

        # Calculate reprojection error
        total_error = 0
        for i in range(len(self.obj_points)):
            img_points2, _ = cv2.projectPoints(
                self.obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.img_points[i], img_points2, cv2.NORM_L2)
            total_error += error ** 2

        mean_error = np.sqrt(total_error / len(self.obj_points))

        result = {
            'camera_matrix': mtx,
            'dist_coeffs': dist,
            'image_size': self.image_size,
            'reprojection_error': mean_error,
            'num_images': len(self.obj_points)
        }

        print(f"\nCalibration Results:")
        print(f"  fx: {mtx[0, 0]:.2f}")
        print(f"  fy: {mtx[1, 1]:.2f}")
        print(f"  cx: {mtx[0, 2]:.2f}")
        print(f"  cy: {mtx[1, 2]:.2f}")
        print(f"  k1: {dist[0, 0]:.6f}")
        print(f"  k2: {dist[0, 1]:.6f}")
        print(f"  p1: {dist[0, 2]:.6f}")
        print(f"  p2: {dist[0, 3]:.6f}")
        print(f"  Reprojection Error: {mean_error:.4f} pixels")

        return result

    def save_kalibr_format(self, result, output_path, topic_name="/cam0/image_raw"):
        """Save in Kalibr YAML format"""
        mtx = result['camera_matrix']
        dist = result['dist_coeffs'].flatten()

        data = {
            'cam0': {
                'camera_model': 'pinhole',
                'intrinsics': [
                    float(mtx[0, 0]),  # fx
                    float(mtx[1, 1]),  # fy
                    float(mtx[0, 2]),  # cx
                    float(mtx[1, 2])   # cy
                ],
                'distortion_model': 'radtan',
                'distortion_coeffs': [
                    float(dist[0]),  # k1
                    float(dist[1]),  # k2
                    float(dist[2]),  # p1
                    float(dist[3])   # p2
                ],
                'resolution': list(result['image_size']),
                'rostopic': topic_name
            }
        }

        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"\nSaved to {output_path}")

    def save_vins_format(self, result, output_path):
        """Save in VINS-Mono format"""
        mtx = result['camera_matrix']
        dist = result['dist_coeffs'].flatten()

        content = f"""# Camera Intrinsic Parameters (VINS-Mono format)
# Calibrated with OpenCV

model_type: PINHOLE
camera_name: smartphone_camera
image_width: {result['image_size'][0]}
image_height: {result['image_size'][1]}

distortion_parameters:
   k1: {dist[0]:.10f}
   k2: {dist[1]:.10f}
   p1: {dist[2]:.10f}
   p2: {dist[3]:.10f}

projection_parameters:
   fx: {mtx[0, 0]:.10f}
   fy: {mtx[1, 1]:.10f}
   cx: {mtx[0, 2]:.10f}
   cy: {mtx[1, 2]:.10f}
"""

        with open(output_path, 'w') as f:
            f.write(content)

        print(f"Saved VINS format to {output_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Camera Calibration')
    parser.add_argument('--images', required=True, help='Image directory or glob pattern')
    parser.add_argument('--cols', type=int, default=9, help='Checkerboard columns')
    parser.add_argument('--rows', type=int, default=6, help='Checkerboard rows')
    parser.add_argument('--size', type=float, default=0.03, help='Square size in meters')
    parser.add_argument('--output', default='camera_calib.yaml', help='Output file')
    parser.add_argument('--show', action='store_true', help='Show detected corners')
    args = parser.parse_args()

    # Find image files
    if os.path.isdir(args.images):
        patterns = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
        image_paths = []
        for pattern in patterns:
            image_paths.extend(glob.glob(os.path.join(args.images, pattern)))
    else:
        image_paths = glob.glob(args.images)

    if not image_paths:
        print("No images found!")
        return

    print(f"Found {len(image_paths)} images")

    # Run calibration
    calibrator = CameraCalibrator(
        checkerboard_size=(args.cols, args.rows),
        square_size=args.size
    )

    calibrator.add_images(image_paths, show_corners=args.show)
    result = calibrator.calibrate()

    # Save
    calibrator.save_kalibr_format(result, args.output)
    calibrator.save_vins_format(result, args.output.replace('.yaml', '_vins.yaml'))

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
```

#### Usage

```bash
# 1. Capture checkerboard images (at least 15-20)
#    - Capture from various angles and distances
#    - Cover the entire image area

# 2. Run calibration
python camera_calibration_opencv.py \
    --images ./calib_images/ \
    --cols 9 \
    --rows 6 \
    --size 0.03 \
    --output camera_calib.yaml \
    --show
```

### Method 2: Using Kalibr (Precise, Recommended)

#### Record ROS bag

```bash
# Record smartphone camera topic to ROS bag
rosbag record -O camera_calib.bag /camera/image_raw

# Recording tips:
# - 20Hz recommended
# - Move target slowly at various angles
# - Record for 1-2 minutes
```

#### Kalibr Camera Calibration

```bash
# Single camera calibration
kalibr_calibrate_cameras \
    --target aprilgrid.yaml \
    --bag camera_calib.bag \
    --models pinhole-radtan \
    --topics /camera/image_raw \
    --show-extraction

# Result: camchain-camera_calib.yaml
```

### Image Capture Guidelines

| Item | Recommendation |
|------|----------------|
| **Number of images** | 15-30 |
| **Angle variation** | Various tilt (+-45 degrees) |
| **Distance variation** | Near to far |
| **Position** | Cover entire image area |
| **Movement** | Slow, no shake |
| **Lighting** | Uniform and sufficient brightness |
| **Focus** | Sharp (no blur) |

---

## Step 3: Measure IMU Noise Parameters

### IMU Noise Model

```
Measurement = True Value + White Noise + Bias
            = true + n(t) + b(t)

White Noise:
  - Gyroscope: sigma_g [rad/s/sqrt(Hz)]
  - Accelerometer: sigma_a [m/s^2/sqrt(Hz)]

Random Walk:
  - Gyroscope: sigma_bg [rad/s^2/sqrt(Hz)]
  - Accelerometer: sigma_ba [m/s^3/sqrt(Hz)]
```

### Method 1: Allan Variance Analysis (Precise)

#### Install and Use imu_utils

```bash
# 1. Install imu_utils
cd ~/catkin_ws/src
git clone https://github.com/gaowenliang/imu_utils.git
git clone https://github.com/gaowenliang/code_utils.git
cd ..
catkin_make

# 2. Collect IMU data (2+ hours in stationary state)
# Place smartphone on flat, vibration-free surface and record
rosbag record -O imu_static.bag /imu/data --duration=7200

# 3. Create launch file (imu_analysis.launch)
```

**imu_analysis.launch:**
```xml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" value="/imu/data"/>
        <param name="imu_name" value="smartphone_imu"/>
        <param name="data_save_path" value="$(find imu_utils)/data/"/>
        <param name="max_time_min" value="120"/>  <!-- in minutes -->
        <param name="max_cluster" value="100"/>
    </node>
</launch>
```

```bash
# 4. Run analysis
roslaunch imu_utils imu_analysis.launch
rosbag play imu_static.bag

# 5. Check results
# Results saved to data/smartphone_imu_imu_param.yaml
```

#### Using kalibr_allan (Alternative)

```bash
# 1. Install
cd ~/catkin_ws/src
git clone https://github.com/rpng/kalibr_allan.git
cd ..
catkin_make

# 2. Run analysis
rosrun kalibr_allan allan_variance /imu/data imu_static.bag

# 3. Plot results with MATLAB/Python
```

#### Interpreting Allan Variance Results

```
Allan Deviation Plot:
                    |
    sigma(tau)      |    /
    [unit]          |   / <-- Random Walk (slope +1/2)
                    |  /
                    | o <-- Bias Instability (minimum point)
                    |  \
                    |   \ <-- White Noise (slope -1/2)
                    |    \
                    +---------------------> tau [s]
                         (integration time)

White Noise (N): Value at tau=1s
Random Walk (K): Fit to +1/2 slope on right side
Bias Instability (B): Minimum point
```

### Method 2: Datasheet-based Estimation (Fast)

Smartphone IMUs often lack accurate datasheets, so refer to typical MEMS IMU values.

#### Recommended Values by Smartphone Tier

```yaml
# Flagship smartphones (iPhone 15, Galaxy S24, etc.)
imu0:
  accelerometer_noise_density: 0.015      # [m/s^2/sqrt(Hz)]
  accelerometer_random_walk: 0.003        # [m/s^3/sqrt(Hz)]
  gyroscope_noise_density: 0.001          # [rad/s/sqrt(Hz)]
  gyroscope_random_walk: 0.0001           # [rad/s^2/sqrt(Hz)]
  update_rate: 200.0

# Mid-range smartphones
imu0:
  accelerometer_noise_density: 0.03
  accelerometer_random_walk: 0.005
  gyroscope_noise_density: 0.002
  gyroscope_random_walk: 0.0002
  update_rate: 100.0

# Budget smartphones
imu0:
  accelerometer_noise_density: 0.05
  accelerometer_random_walk: 0.01
  gyroscope_noise_density: 0.005
  gyroscope_random_walk: 0.0005
  update_rate: 100.0
```

### IMU Configuration File (Kalibr Format)

```yaml
# imu.yaml
imu0:
  accelerometer_noise_density: 0.015      # sigma_a [m/s^2/sqrt(Hz)]
  accelerometer_random_walk: 0.003        # sigma_ba [m/s^3/sqrt(Hz)]
  gyroscope_noise_density: 0.001          # sigma_g [rad/s/sqrt(Hz)]
  gyroscope_random_walk: 0.0001           # sigma_bg [rad/s^2/sqrt(Hz)]
  rostopic: /imu/data
  update_rate: 200.0                      # [Hz]
```

### Practical Tips

> **Important**: Allan Variance analysis is performed in a stationary state at constant temperature, which differs from actual operating conditions.
> Therefore, it is common practice to **multiply measured values by a safety factor**:
> - Noise Density: **x 10**
> - Random Walk: **x 5-10**

---

## Step 4: Camera-IMU Extrinsic Calibration

### Data Collection

#### Data Collection Requirements

| Item | Recommended Value |
|------|-------------------|
| **Camera frame rate** | 20 Hz |
| **IMU sampling rate** | 200 Hz |
| **Recording duration** | 60-120 seconds |
| **Motion** | Rotation + translation on all axes |

#### Motion Pattern

```
Recommended motion sequence:

1. X-axis rotation (Roll): 3 times left-right
2. Y-axis rotation (Pitch): 3 times up-down
3. Z-axis rotation (Yaw): 3 times clockwise/counter-clockwise
4. Figure-8 motion: 2-3 times
5. Combined motion: Freely for 10-15 seconds

Notes:
- Minimize motion blur (move slowly)
- Keep target always in view
- Sufficiently excite all IMU axes
```

#### Record ROS bag

```bash
# Record camera and IMU simultaneously
rosbag record -O calib_imu_camera.bag \
    /camera/image_raw \
    /imu/data

# Note: Stream from smartphone app to ROS topics or
# convert recorded data to ROS bag
```

### Run Kalibr Camera-IMU Calibration

```bash
kalibr_calibrate_imu_camera \
    --target aprilgrid.yaml \
    --cam camchain.yaml \
    --imu imu.yaml \
    --bag calib_imu_camera.bag \
    --show-extraction \
    --time-calibration

# Option description:
# --target: Calibration target configuration
# --cam: Camera intrinsics (Step 2 result)
# --imu: IMU noise parameters (Step 3 result)
# --bag: Recorded data
# --show-extraction: Visualize corner detection
# --time-calibration: Also estimate time offset
```

### Analyzing Result Files

#### camchain-imucam-*.yaml Structure

```yaml
cam0:
  T_cam_imu:
  - [r11, r12, r13, tx]
  - [r21, r22, r23, ty]
  - [r31, r32, r33, tz]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [k1, k2, p1, p2]
  distortion_model: radtan
  intrinsics: [fx, fy, cx, cy]
  resolution: [width, height]
  rostopic: /camera/image_raw
  timeshift_cam_imu: -0.00523    # Time offset [s]
```

### Result Validation

#### Check Reprojection Error

```
Good result criteria:
- Reprojection error: < 0.5 pixels
- Time offset: Typically -10ms to +10ms
- T_cam_imu: Physically reasonable transformation
```

#### PDF Report Analysis

Kalibr automatically generates a PDF report:
- Corner detection results
- Reprojection error distribution
- IMU prediction vs actual comparison
- Calibration convergence graph

---

## Step 5: Generate Configuration Files for Each Framework

### VINS-Mono Configuration

```yaml
# config/smartphone/smartphone_config.yaml

%YAML:1.0

# Camera Intrinsics
model_type: PINHOLE
camera_name: smartphone
image_width: 640
image_height: 480

distortion_parameters:
   k1: -0.28340811
   k2: 0.07395907
   p1: 0.00019359
   p2: 1.76187114e-05

projection_parameters:
   fx: 458.654
   fy: 457.296
   cx: 367.215
   cy: 248.375

# Camera-IMU Extrinsic (from Kalibr T_cam_imu)
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [0.0148655, -0.9998809, 0.00414029, -0.0216401,
          0.9995572, 0.0149672, 0.02571552, -0.0646769,
          -0.0257744, 0.00375618, 0.9996607, 0.00981073,
          0, 0, 0, 1]

# IMU Parameters
acc_n: 0.015          # accelerometer noise density
gyr_n: 0.001          # gyroscope noise density
acc_w: 0.003          # accelerometer random walk
gyr_w: 0.0001         # gyroscope random walk

# Time offset
estimate_td: 1        # Enable online estimation
td: -0.00523          # Initial value (Kalibr result)

# Extrinsic estimation
estimate_extrinsic: 1 # Online fine-tuning

# Rolling Shutter
rolling_shutter: 1
rolling_shutter_tr: 0.033  # Line readout time [s]
```

### OpenVINS Configuration

```yaml
# config/smartphone/estimator_config.yaml

# Camera intrinsics
cam0_is_fisheye: false
cam0_k: [458.654, 457.296, 367.215, 248.375]
cam0_d: [-0.2834, 0.0739, 0.00019, 1.76e-05]
cam0_wh: [640, 480]

# Camera-IMU transform (T_imu_cam = inv(T_cam_imu))
T_imu_cam:
  - [0.0148655, 0.9995572, -0.0257744, 0.0652229]
  - [-0.9998809, 0.0149672, 0.00375618, 0.0207089]
  - [0.00414029, 0.02571552, 0.9996607, -0.0094768]
  - [0, 0, 0, 1]

# IMU intrinsics
gyroscope_noise_density: 0.001
accelerometer_noise_density: 0.015
gyroscope_random_walk: 0.0001
accelerometer_random_walk: 0.003

# Time offset calibration
calib_cam_timeoffset: true
calib_cam_extrinsics: true
```

### ORB-SLAM3 Configuration

```yaml
# smartphone.yaml

%YAML:1.0

Camera.type: "PinHole"

Camera1.fx: 458.654
Camera1.fy: 457.296
Camera1.cx: 367.215
Camera1.cy: 248.375

Camera1.k1: -0.28340811
Camera1.k2: 0.07395907
Camera1.p1: 0.00019359
Camera1.p2: 1.76187114e-05

Camera.width: 640
Camera.height: 480
Camera.fps: 30

# IMU-Camera Transform (Tbc = T_imu_cam)
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [0.0148655, 0.9995572, -0.0257744, 0.0652229,
          -0.9998809, 0.0149672, 0.00375618, 0.0207089,
          0.00414029, 0.02571552, 0.9996607, -0.0094768,
          0, 0, 0, 1]

# IMU Parameters (10x increase recommended)
IMU.NoiseGyro: 0.01       # 0.001 * 10
IMU.NoiseAcc: 0.15        # 0.015 * 10
IMU.GyroWalk: 0.001       # 0.0001 * 10
IMU.AccWalk: 0.03         # 0.003 * 10
IMU.Frequency: 200.0

# ORB Extractor
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

### Basalt Configuration

#### calibration.json

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.0652229,
        "py": 0.0207089,
        "pz": -0.0094768,
        "qx": 0.0128,
        "qy": -0.0074,
        "qz": 0.7071,
        "qw": 0.7071
      }
    ],
    "intrinsics": [
      {
        "camera_type": "pinhole",
        "intrinsics": {
          "fx": 458.654,
          "fy": 457.296,
          "cx": 367.215,
          "cy": 248.375
        }
      }
    ],
    "resolution": [[640, 480]],
    "imu_update_rate": 200.0,
    "accel_noise_std": [0.015, 0.015, 0.015],
    "gyro_noise_std": [0.001, 0.001, 0.001],
    "accel_bias_std": [0.003, 0.003, 0.003],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "cam_time_offset_ns": -5230000
  }
}
```

---

## Conversion Utility Scripts

### Kalibr to Each Framework Conversion

```python
#!/usr/bin/env python3
"""
kalibr_converter.py
Convert Kalibr calibration results to each framework format
"""

import yaml
import json
import numpy as np
from scipy.spatial.transform import Rotation

def load_kalibr_result(camchain_path):
    """Load Kalibr camchain-imucam YAML"""
    with open(camchain_path, 'r') as f:
        data = yaml.safe_load(f)
    return data

def get_T_imu_cam(T_cam_imu):
    """Convert T_cam_imu to T_imu_cam"""
    T = np.array(T_cam_imu)
    return np.linalg.inv(T)

def matrix_to_quaternion(R):
    """Convert rotation matrix to quaternion"""
    rot = Rotation.from_matrix(R)
    return rot.as_quat()  # [qx, qy, qz, qw]

def to_vins_mono(kalibr_data, imu_params, output_path):
    """Convert to VINS-Mono format"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    intrinsics = cam['intrinsics']
    dist = cam['distortion_coeffs']
    resolution = cam['resolution']
    time_offset = cam.get('timeshift_cam_imu', 0.0)

    content = f"""%YAML:1.0

#common parameters
imu_topic: "/imu/data"
image_topic: "/camera/image_raw"
output_path: "./output/"

#camera calibration
model_type: PINHOLE
camera_name: smartphone
image_width: {resolution[0]}
image_height: {resolution[1]}

distortion_parameters:
   k1: {dist[0]:.10f}
   k2: {dist[1]:.10f}
   p1: {dist[2]:.10f}
   p2: {dist[3]:.10f}

projection_parameters:
   fx: {intrinsics[0]:.10f}
   fy: {intrinsics[1]:.10f}
   cx: {intrinsics[2]:.10f}
   cy: {intrinsics[3]:.10f}

# Extrinsic parameter between IMU and Camera
estimate_extrinsic: 1

body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [{T_cam_imu[0][0]}, {T_cam_imu[0][1]}, {T_cam_imu[0][2]}, {T_cam_imu[0][3]},
          {T_cam_imu[1][0]}, {T_cam_imu[1][1]}, {T_cam_imu[1][2]}, {T_cam_imu[1][3]},
          {T_cam_imu[2][0]}, {T_cam_imu[2][1]}, {T_cam_imu[2][2]}, {T_cam_imu[2][3]},
          0, 0, 0, 1]

#imu parameters
acc_n: {imu_params['accelerometer_noise_density']}
gyr_n: {imu_params['gyroscope_noise_density']}
acc_w: {imu_params['accelerometer_random_walk']}
gyr_w: {imu_params['gyroscope_random_walk']}
g_norm: 9.81007

#unsynchronized parameters
estimate_td: 1
td: {time_offset}

#rolling shutter parameters
rolling_shutter: 1
rolling_shutter_tr: 0.033

#visualization parameters
show_track: 1
"""

    with open(output_path, 'w') as f:
        f.write(content)
    print(f"Saved VINS-Mono config to {output_path}")

def to_openvins(kalibr_data, imu_params, output_path):
    """Convert to OpenVINS format"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    T_imu_cam = np.linalg.inv(T_cam_imu)
    intrinsics = cam['intrinsics']
    dist = cam['distortion_coeffs']
    resolution = cam['resolution']

    content = f"""# OpenVINS Configuration

# Camera parameters
cam0_is_fisheye: false
cam0_k: [{intrinsics[0]}, {intrinsics[1]}, {intrinsics[2]}, {intrinsics[3]}]
cam0_d: [{dist[0]}, {dist[1]}, {dist[2]}, {dist[3]}]
cam0_wh: [{resolution[0]}, {resolution[1]}]

# IMU to Camera transform
T_imu_cam:
  - [{T_imu_cam[0][0]}, {T_imu_cam[0][1]}, {T_imu_cam[0][2]}, {T_imu_cam[0][3]}]
  - [{T_imu_cam[1][0]}, {T_imu_cam[1][1]}, {T_imu_cam[1][2]}, {T_imu_cam[1][3]}]
  - [{T_imu_cam[2][0]}, {T_imu_cam[2][1]}, {T_imu_cam[2][2]}, {T_imu_cam[2][3]}]
  - [0, 0, 0, 1]

# IMU noise parameters
gyroscope_noise_density: {imu_params['gyroscope_noise_density']}
accelerometer_noise_density: {imu_params['accelerometer_noise_density']}
gyroscope_random_walk: {imu_params['gyroscope_random_walk']}
accelerometer_random_walk: {imu_params['accelerometer_random_walk']}

# Online calibration
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
"""

    with open(output_path, 'w') as f:
        f.write(content)
    print(f"Saved OpenVINS config to {output_path}")

def to_orbslam3(kalibr_data, imu_params, output_path):
    """Convert to ORB-SLAM3 format"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    T_imu_cam = np.linalg.inv(T_cam_imu)
    intrinsics = cam['intrinsics']
    dist = cam['distortion_coeffs']
    resolution = cam['resolution']

    # ORB-SLAM3 recommends 10x increase for noise parameters
    noise_factor = 10

    # Convert 4x4 matrix to 1D list
    tbc_data = T_imu_cam.flatten().tolist()

    content = f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

Camera1.fx: {intrinsics[0]}
Camera1.fy: {intrinsics[1]}
Camera1.cx: {intrinsics[2]}
Camera1.cy: {intrinsics[3]}

Camera1.k1: {dist[0]}
Camera1.k2: {dist[1]}
Camera1.p1: {dist[2]}
Camera1.p2: {dist[3]}

Camera.width: {resolution[0]}
Camera.height: {resolution[1]}
Camera.fps: 30
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters
#--------------------------------------------------------------------------------------------

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{', '.join([str(x) for x in tbc_data])}]

IMU.NoiseGyro: {imu_params['gyroscope_noise_density'] * noise_factor}
IMU.NoiseAcc: {imu_params['accelerometer_noise_density'] * noise_factor}
IMU.GyroWalk: {imu_params['gyroscope_random_walk'] * noise_factor}
IMU.AccWalk: {imu_params['accelerometer_random_walk'] * noise_factor}
IMU.Frequency: {imu_params.get('update_rate', 200.0)}

#--------------------------------------------------------------------------------------------
# ORB Extractor
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
"""

    with open(output_path, 'w') as f:
        f.write(content)
    print(f"Saved ORB-SLAM3 config to {output_path}")

def to_basalt(kalibr_data, imu_params, output_path):
    """Convert to Basalt format"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    T_imu_cam = np.linalg.inv(T_cam_imu)

    R = T_imu_cam[:3, :3]
    t = T_imu_cam[:3, 3]
    quat = matrix_to_quaternion(R)

    intrinsics = cam['intrinsics']
    resolution = cam['resolution']
    time_offset = cam.get('timeshift_cam_imu', 0.0)
    time_offset_ns = int(time_offset * 1e9)

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
                "camera_type": "pinhole",
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
            "imu_update_rate": imu_params.get('update_rate', 200.0),
            "accel_noise_std": [imu_params['accelerometer_noise_density']] * 3,
            "gyro_noise_std": [imu_params['gyroscope_noise_density']] * 3,
            "accel_bias_std": [imu_params['accelerometer_random_walk']] * 3,
            "gyro_bias_std": [imu_params['gyroscope_random_walk']] * 3,
            "cam_time_offset_ns": time_offset_ns
        }
    }

    with open(output_path, 'w') as f:
        json.dump(basalt_calib, f, indent=2)
    print(f"Saved Basalt config to {output_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Convert Kalibr calibration')
    parser.add_argument('--kalibr', required=True, help='Kalibr camchain-imucam YAML')
    parser.add_argument('--imu', required=True, help='IMU parameters YAML')
    parser.add_argument('--output-dir', default='.', help='Output directory')
    parser.add_argument('--format', nargs='+',
                        choices=['vins', 'openvins', 'orbslam3', 'basalt', 'all'],
                        default=['all'], help='Output formats')
    args = parser.parse_args()

    # Load
    kalibr_data = load_kalibr_result(args.kalibr)
    with open(args.imu, 'r') as f:
        imu_data = yaml.safe_load(f)
    imu_params = imu_data['imu0']

    formats = args.format
    if 'all' in formats:
        formats = ['vins', 'openvins', 'orbslam3', 'basalt']

    import os
    os.makedirs(args.output_dir, exist_ok=True)

    if 'vins' in formats:
        to_vins_mono(kalibr_data, imu_params,
                     os.path.join(args.output_dir, 'vins_config.yaml'))

    if 'openvins' in formats:
        to_openvins(kalibr_data, imu_params,
                    os.path.join(args.output_dir, 'openvins_config.yaml'))

    if 'orbslam3' in formats:
        to_orbslam3(kalibr_data, imu_params,
                    os.path.join(args.output_dir, 'orbslam3_config.yaml'))

    if 'basalt' in formats:
        to_basalt(kalibr_data, imu_params,
                  os.path.join(args.output_dir, 'basalt_calib.json'))


if __name__ == '__main__':
    main()
```

#### Usage

```bash
python kalibr_converter.py \
    --kalibr camchain-imucam-calib.yaml \
    --imu imu.yaml \
    --output-dir ./configs \
    --format all
```

---

## Troubleshooting

### Camera Calibration Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Corner detection failure | Poor lighting, blurry image | Improve lighting, capture sharp images |
| Large reprojection error (>1.0) | Insufficient images, limited angles | More images, various angles |
| Abnormal distortion coefficients | Incorrect checkerboard size | Use actual measured values |

### IMU Calibration Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Noise values too large | Vibration, short recording time | Stable environment, 2+ hour recording |
| Random walk not converging | Insufficient recording time | Longer recording (6+ hours) |
| Allan plot looks abnormal | Data issues | Check IMU data, re-record |

### Camera-IMU Calibration Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Not converging | Insufficient motion | Sufficiently excite all axes |
| Time offset too large (>50ms) | Synchronization issue | Check hardware synchronization |
| Large reprojection error | Camera calibration error | Redo Step 2 |
| Kalibr crash | Memory shortage, data issue | Check bag file, increase memory |

---

## Checklist

### Before Calibration

- [ ] Calibration target printed and measured
- [ ] Target installed flat
- [ ] Camera settings fixed (exposure, focus, zoom)
- [ ] IMU sampling rate confirmed (200Hz recommended)
- [ ] Data synchronization method confirmed

### After Calibration

- [ ] Reprojection error < 0.5 pixels
- [ ] Time offset reasonable (-50ms to +50ms)
- [ ] T_cam_imu physically reasonable
- [ ] IMU noise parameters in reasonable range
- [ ] Configuration file conversion complete
- [ ] Testing with actual data complete

---

## References

### Tools
- [Kalibr GitHub](https://github.com/ethz-asl/kalibr)
- [imu_utils GitHub](https://github.com/gaowenliang/imu_utils)
- [kalibr_allan GitHub](https://github.com/rpng/kalibr_allan)
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

### Documentation
- [Kalibr Wiki - Camera IMU Calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [Kalibr Wiki - IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [iPhone Calibration with Kalibr](https://tomas789.medium.com/iphone-calibration-camera-imu-and-kalibr-33b8645fb0aa)

### Tutorials
- [Robotics Knowledgebase - IMU-Camera Calibration](https://roboticsknowledgebase.com/wiki/sensing/camera-imu-calibration/)
- [LearnOpenCV - Camera Calibration](https://learnopencv.com/camera-calibration-using-opencv/)

---

*Document created: 2026-01-19*
*Version: 1.0.0*
