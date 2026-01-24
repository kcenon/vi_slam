# VI-SLAM Framework Selection Guide

## Overview

This document is a comprehensive guide for selecting the appropriate framework when implementing Visual-Inertial SLAM using smartphone data. It provides comparative analysis of four major open-source frameworks—VINS-Mono, OpenVINS, ORB-SLAM3, and Basalt—from various perspectives.

---

## Framework Overview

### At-a-Glance Comparison

| Framework | Developer | Algorithm | Best Use Case |
|-----------|-----------|-----------|---------------|
| **VINS-Mono** | HKUST | Sliding Window Optimization | Smartphones, Drones |
| **OpenVINS** | Univ. of Delaware | MSCKF (EKF-based) | Research, Learning, Customization |
| **ORB-SLAM3** | Univ. of Zaragoza | Graph Optimization + BoW | High-precision Mapping, Loop Closure |
| **Basalt** | TUM | Square Root VIO | Real-time AR/VR, Low Latency |

---

## Detailed Comparison Tables

### 1. Feature Comparison

| Feature | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|---------|:---------:|:--------:|:---------:|:------:|
| **Mono-Inertial** | Yes | Yes | Yes | Yes |
| **Stereo-Inertial** | Yes (Fusion) | Yes | Yes | Yes |
| **Automatic Time Offset Estimation** | Yes | Yes | No | Partial (During calibration) |
| **Rolling Shutter Support** | Yes | Yes | No | No |
| **Loop Closure** | Yes | No | Yes | No |
| **Relocalization** | Yes | No | Yes | No |
| **Multi-Map** | No | No | Yes | No |
| **Global BA** | Yes | No | Yes | No |
| **Online Calibration** | Yes | Yes | No | No |
| **Fisheye Support** | Yes | Yes | Yes | Yes |

### 2. Performance Comparison (Based on EuRoC/TUM-VI Benchmarks)

| Metric | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|--------|:---------:|:--------:|:---------:|:------:|
| **Accuracy (ATE)** | 4/5 | 4/5 | 5/5 | 4/5 |
| **Processing Speed** | 3/5 | 4/5 | 3/5 | 5/5 |
| **CPU Usage** | 3/5 | 4/5 | 3/5 | 5/5 |
| **Memory Usage** | 3/5 | 4/5 | 3/5 | 5/5 |
| **Initialization Speed** | 4/5 | 4/5 | 3/5 | 4/5 |
| **Robustness** | 4/5 | 4/5 | 5/5 | 4/5 |

### 3. Benchmark Numbers (RMSE ATE)

#### EuRoC Dataset

| Sequence | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|----------|-----------|----------|-----------|--------|
| MH_01_easy | 0.15m | 0.12m | **0.035m** | 0.08m |
| MH_03_medium | 0.19m | 0.15m | **0.037m** | 0.11m |
| MH_05_difficult | 0.30m | 0.22m | **0.062m** | 0.15m |
| V1_01_easy | 0.08m | 0.07m | **0.035m** | **0.04m** |
| V2_03_difficult | 0.28m | 0.20m | **0.10m** | 0.18m |

#### TUM-VI Dataset

| Sequence | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|----------|-----------|----------|-----------|--------|
| room1 | 0.05m | 0.04m | **0.009m** | 0.03m |
| room4 | 0.08m | 0.06m | **0.012m** | 0.05m |
| corridor1 | 0.15m | 0.12m | **0.08m** | 0.10m |

> **Note**: The above numbers are representative benchmark results. Actual performance may vary depending on parameter tuning and environment.

### 4. Smartphone Compatibility

| Item | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **Smartphone Data Suitability** | 5/5 | 5/5 | 3/5 | 4/5 |
| **Rolling Shutter Handling** | Yes (Auto correction) | Yes (Auto correction) | No (Unsupported) | No (Unsupported) |
| **Time Sync Flexibility** | Yes (Auto estimation) | Yes (Auto estimation) | No (Pre-calibration required) | Partial (Estimated during calibration) |
| **Low-quality IMU Handling** | Yes (Robust) | Yes (Robust) | Partial (Accurate params needed) | Partial (Accurate params needed) |
| **Configuration Difficulty** | Medium | Medium | High | Medium |

---

## Recommended Frameworks by Scenario

### Decision Flowchart

```
Start
  |
  +-- Is real-time AR/VR the goal?
  |   +-- Yes --> Processing speed is most important
  |   |           --> Basalt recommended
  |   |
  |   +-- No --+
  |            |
  +-- Is Loop Closure needed? (Large-scale environment, revisiting)
  |   +-- Yes --> Global consistency is important
  |   |           --> ORB-SLAM3 or VINS-Mono recommended
  |   |
  |   +-- No --+
  |            |
  +-- Using smartphone data?
  |   +-- Yes --> Need Rolling Shutter + time offset handling
  |   |       |
  |   |       +-- Can perform accurate calibration?
  |   |       |   +-- Yes --> All frameworks are viable
  |   |       |   +-- No --> VINS-Mono or OpenVINS recommended
  |   |       |
  |   |       +-- For rapid prototyping?
  |   |           --> VINS-Mono recommended
  |   |
  |   +-- No --+
  |            |
  +-- Is this for research/learning?
  |   +-- Yes --> Documentation + modularity important
  |   |           --> OpenVINS recommended
  |   |
  |   +-- No --+
  |            |
  +-- Need highest accuracy?
      +-- Yes --> ORB-SLAM3 recommended
      +-- No --> Choose based on situation
```

### Detailed Guide by Scenario

#### 1. Real-time SLAM on Smartphone

| Priority | Framework | Reason |
|----------|-----------|--------|
| **1st** | VINS-Mono | Rolling Shutter support, automatic time offset estimation, robust initialization |
| **2nd** | OpenVINS | Similar advantages to VINS, better documentation |
| **3rd** | Basalt | Excellent processing speed, but no RS/time offset support |
| **4th** | ORB-SLAM3 | Best accuracy, but requires pre-calibration |

#### 2. Drone/Robot Autonomous Navigation

| Priority | Framework | Reason |
|----------|-----------|--------|
| **1st** | Basalt | Lowest latency, high-speed processing |
| **2nd** | VINS-Fusion | Stereo + GPS fusion capable |
| **3rd** | ORB-SLAM3 | Loop Closure for large-scale environments |

#### 3. AR/VR Applications

| Priority | Framework | Reason |
|----------|-----------|--------|
| **1st** | Basalt | Fastest processing speed, low latency |
| **2nd** | VINS-Mono | Excellent smartphone compatibility |

#### 4. High-precision 3D Mapping

| Priority | Framework | Reason |
|----------|-----------|--------|
| **1st** | ORB-SLAM3 | Best accuracy, Loop Closure, Multi-Map |
| **2nd** | VINS-Fusion | Global BA support |

#### 5. Research/Educational Purposes

| Priority | Framework | Reason |
|----------|-----------|--------|
| **1st** | OpenVINS | Excellent documentation, modularity, ROS integration |
| **2nd** | VINS-Mono | Active community, abundant references |

#### 6. Offline Data Processing

| Priority | Framework | Reason |
|----------|-----------|--------|
| **1st** | ORB-SLAM3 | Full optimization possible, best accuracy |
| **2nd** | OpenVINS | Easy reprocessing and analysis |

---

## Detailed Pros and Cons by Framework

### VINS-Mono / VINS-Fusion

#### Pros
- **Automatic time offset estimation** (`estimate_td: 1`)
- **Rolling Shutter correction** (`rolling_shutter: 1`)
- **Online Camera-IMU extrinsic estimation** (`estimate_extrinsic: 2`)
- Robust initialization (within 2-3 seconds)
- Loop Closure and Global BA support
- VINS-Fusion: GPS and Stereo fusion support
- Active community and abundant references

#### Cons
- Slower processing speed than Basalt
- Relatively high CPU/memory usage
- Less modular code than OpenVINS

#### Recommended Settings (Smartphone)
```yaml
estimate_extrinsic: 2    # Online estimation
estimate_td: 1           # Time offset estimation
rolling_shutter: 1       # RS correction enabled
```

---

### OpenVINS

#### Pros
- **Excellent documentation** (official docs, tutorials)
- **Modular code structure** (suitable for research/learning)
- Online time offset and extrinsic estimation
- Rolling Shutter support
- Computationally efficient (MSCKF-based)
- Seamless Kalibr integration
- ROS1/ROS2 support

#### Cons
- No Loop Closure support
- No Global optimization support
- Drift accumulation in large-scale environments

#### Recommended Settings (Smartphone)
```yaml
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
use_mask: false
```

---

### ORB-SLAM3

#### Pros
- **Best accuracy** (1st place in EuRoC/TUM-VI benchmarks)
- **Powerful Loop Closure** (BoW-based)
- **Multi-Map support** (map reuse across sessions)
- Relocalization support
- Supports Visual-only, Visual-Inertial, and RGB-D
- Robust feature tracking (ORB)

#### Cons
- **No automatic time offset estimation** (pre-calibration required)
- **No Rolling Shutter support**
- Challenging initialization (requires accurate parameters)
- Sensitive to smartphone data

#### Recommended Settings (Smartphone)
```yaml
# Increase IMU noise by 10x to improve initialization stability
IMU.NoiseGyro: 1.0e-3
IMU.NoiseAcc: 1.5e-2
IMU.GyroWalk: 1.0e-4
IMU.AccWalk: 3.0e-3
```

---

### Basalt

#### Pros
- **Fastest processing speed** (optimal for real-time AR/VR)
- **Lowest CPU/memory usage**
- Square Root Marginalization (numerical stability)
- Optical Flow-based frontend (efficient)
- Double Sphere camera model (wide-angle lens support)
- Time offset estimation during calibration

#### Cons
- **No Rolling Shutter support**
- **No Loop Closure support**
- Requires Kalibr calibration conversion
- Relatively limited community/documentation

#### Recommended Settings (Smartphone)
```json
{
  "config.optical_flow_epipolar_error": 0.01,
  "config.vio_obs_std_dev": 0.5,
  "config.vio_max_kfs": 7
}
```

---

## Resource Requirements Comparison

### Minimum Hardware Specifications

| Item | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|-----------|----------|-----------|--------|
| **CPU** | i5 or higher | i5 or higher | i5 or higher | i3 or higher |
| **RAM** | 8GB | 4GB | 8GB | 4GB |
| **GPU** | Not required | Not required | Not required | Not required |

### Recommended Hardware Specifications

| Item | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|-----------|----------|-----------|--------|
| **CPU** | i7/Ryzen 7 | i5/Ryzen 5 | i7/Ryzen 7 | i5/Ryzen 5 |
| **RAM** | 16GB | 8GB | 16GB | 8GB |
| **Storage** | SSD | SSD | SSD | SSD |

### Per-frame Processing Time (on i7)

| Framework | Average Processing Time | Real-time at 30fps |
|-----------|------------------------|-------------------|
| **Basalt** | ~15ms | Yes (with margin) |
| **OpenVINS** | ~25ms | Yes |
| **VINS-Mono** | ~35ms | Borderline |
| **ORB-SLAM3** | ~40ms | Borderline |

---

## Installation and Dependencies

### ROS Version Support

| Framework | ROS1 | ROS2 | Standalone |
|-----------|:----:|:----:|:---------:|
| VINS-Mono | Yes | Partial (porting needed) | No |
| OpenVINS | Yes | Yes | Yes |
| ORB-SLAM3 | Yes | Partial (wrapper needed) | Yes |
| Basalt | Yes | Partial | Yes |

### Key Dependencies

| Framework | OpenCV | Eigen | Ceres | g2o | Pangolin |
|-----------|:------:|:-----:|:-----:|:---:|:--------:|
| VINS-Mono | Yes | Yes | Yes | No | No |
| OpenVINS | Yes | Yes | No | No | No |
| ORB-SLAM3 | Yes | Yes | No | Yes | Yes |
| Basalt | Yes | Yes | No | No | No |

### Build Difficulty

| Framework | Difficulty | Estimated Time | Major Issues |
|-----------|:----------:|----------------|--------------|
| VINS-Mono | 2/5 | 30 min | Ceres version compatibility |
| OpenVINS | 1/5 | 15 min | Almost none |
| ORB-SLAM3 | 3/5 | 1 hour | Pangolin, DBoW2 build |
| Basalt | 2/5 | 30 min | Submodule initialization |

---

## Calibration Requirements

### Required Calibration Items

| Item | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **Camera Intrinsic** | Required | Required | Required | Required |
| **Camera-IMU Extrinsic** | Initial value only | Initial value only | Accurate value required | Accurate value required |
| **Time Offset** | Auto-estimated | Auto-estimated | Pre-correction required | Estimated during calibration |
| **IMU Noise** | Required | Required | Required | Required |

### Calibration Tool Compatibility

| Tool | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **Kalibr** | Direct use | Direct use | Conversion needed | Conversion needed |
| **OpenCV** | Yes | Yes | Yes | Yes |
| **Built-in Tool** | No | No | No | Yes (basalt_calibrate) |

---

## Quick Selection Guide

### Question-based Selection

| Question | Recommended Framework |
|----------|----------------------|
| I want to quickly test with a smartphone | **VINS-Mono** |
| I want to study VI-SLAM | **OpenVINS** |
| I need the highest accuracy | **ORB-SLAM3** |
| I want to build real-time AR/VR | **Basalt** |
| I want to map a large-scale environment | **ORB-SLAM3** |
| I want to mount it on a drone | **Basalt** or **VINS-Fusion** |
| Accurate calibration is difficult | **VINS-Mono** or **OpenVINS** |
| I'm using a Global Shutter camera | All frameworks are viable |
| I'm using a Rolling Shutter camera | **VINS-Mono** or **OpenVINS** |

### One-page Summary

```
+----------------------------------------------------------------+
|              VI-SLAM Framework Selection Summary                |
+----------------------------------------------------------------+
|                                                                |
|  Best Accuracy     -->  ORB-SLAM3                              |
|  Best Speed        -->  Basalt                                 |
|  Best Smartphone   -->  VINS-Mono                              |
|  Best Learning     -->  OpenVINS                               |
|                                                                |
+----------------------------------------------------------------+
|                                                                |
|  Rolling Shutter Required  -->  VINS-Mono, OpenVINS            |
|  Loop Closure Required     -->  ORB-SLAM3, VINS-Mono           |
|  Real-time Low Latency     -->  Basalt                         |
|  Difficult Calibration     -->  VINS-Mono, OpenVINS            |
|                                                                |
+----------------------------------------------------------------+
```

---

## Migration Guide

### Migrating from VINS-Mono to Other Frameworks

#### VINS-Mono to OpenVINS

```yaml
# VINS-Mono settings
estimate_extrinsic: 2
estimate_td: 1

# Equivalent OpenVINS settings
calib_cam_extrinsics: true
calib_cam_timeoffset: true
```

#### VINS-Mono to ORB-SLAM3

**Note**: Time offset must be measured in advance and data preprocessing is required

```python
# Timestamp correction code
time_offset = 0.005  # Value measured from Kalibr [s]
corrected_img_time = img_time + time_offset
```

### Using Kalibr Results

| Framework | How to Use Kalibr Results |
|-----------|---------------------------|
| VINS-Mono | Direct YAML copy |
| OpenVINS | Direct YAML use or minor modifications |
| ORB-SLAM3 | Invert T_cam_imu to get Tbc |
| Basalt | Convert to JSON using Python script |

---

## Conclusion and Recommendations

### When Starting a Smartphone VI-SLAM Project

1. **Beginners/Prototyping**: Start with VINS-Mono
   - Automatic time offset and Rolling Shutter handling
   - Relatively forgiving configuration

2. **Research/Learning Purposes**: OpenVINS recommended
   - Excellent documentation
   - Easy code analysis and modification

3. **Final Product/High Precision**: ORB-SLAM3
   - Use after accurate calibration
   - Achieves best performance

4. **Real-time AR/VR**: Basalt
   - When processing speed is the priority
   - Must ensure calibration accuracy

### Recommended Step-by-step Approach

```
Step 1: Verify system with VINS-Mono
        |
Step 2: Refine calibration (Kalibr)
        |
Step 3: Transition to framework matching your goal
        - Accuracy --> ORB-SLAM3
        - Speed --> Basalt
        - Research --> OpenVINS
```

---

## References

### Papers
- [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator](https://arxiv.org/abs/1708.03852)
- [OpenVINS: A Research Platform for Visual-Inertial Estimation](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)
- [ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM](https://arxiv.org/abs/2007.11898)
- [Basalt: Visual-Inertial Mapping with Non-Linear Factor Recovery](https://arxiv.org/abs/1904.06504)

### Benchmark Comparisons
- [Comparison of modern open-source Visual SLAM approaches](https://arxiv.org/abs/2108.01654)
- [Visual-Inertial SLAM Comparison (Bharat Joshi)](https://joshi-bharat.github.io/projects/visual_slam_comparison/)
- [The TUM VI Benchmark for Evaluating Visual-Inertial Odometry](https://arxiv.org/abs/1804.06120)

### Official Repositories
- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Basalt GitLab](https://gitlab.com/VladyslavUsenko/basalt)

### Datasets
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM VI Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)

---

*Document created: 2026-01-19*
*Version: 1.0.0*
