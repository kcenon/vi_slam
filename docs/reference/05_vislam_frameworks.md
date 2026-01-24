# Open-source VI-SLAM Frameworks Comparison

## 1. Major Framework Overview

| Framework | Developer | Method | License | Latest Version |
|-----------|-----------|--------|---------|----------------|
| ORB-SLAM3 | Univ. Zaragoza | Optimization | GPLv3 | 1.0 (2021) |
| VINS-Mono | HKUST | Optimization | GPLv3 | - |
| VINS-Fusion | HKUST | Optimization | GPLv3 | - |
| OpenVINS | Univ. Delaware | Filter (MSCKF) | GPLv3 | 2.7+ |
| Basalt | TUM | Optimization | BSD-3 | - |
| Kimera | MIT | Hybrid | BSD-2 | - |
| ROVIO | ETH Zurich | Filter (EKF) | BSD | - |

## 2. Detailed Comparison

### 2.1 ORB-SLAM3

**GitHub**: [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

#### Features
- Most complete feature-based SLAM implementation
- Multi-map system support
- Loop closing and relocalization

#### Supported Sensors
- Monocular / Stereo / RGB-D
- Monocular-Inertial / Stereo-Inertial

#### Advantages
- Top-level accuracy
- Rich features (Loop closing, Map saving/loading)
- Various sensor combination support

#### Disadvantages
- **Sensor synchronization required** (no automatic time offset estimation)
- Complex setup
- Heavy computation
- GPLv3 license

#### Performance (EuRoC Dataset)
```
ATE RMSE: 0.02-0.05m (top tier)
Processing speed: ~20-30 FPS
```

#### Installation
```bash
# Dependencies
sudo apt install libopencv-dev libeigen3-dev

# Build
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

---

### 2.2 VINS-Mono

**GitHub**: [HKUST-Aerial-Robotics/VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)

#### Features
- Tightly-coupled monocular VIO
- Online temporal calibration support
- Pose graph optimization

#### Supported Sensors
- Monocular camera + IMU

#### Advantages
- **Automatic time offset estimation** (allows asynchronous sensors)
- Robust initialization
- Real-time performance

#### Disadvantages
- Monocular only (Stereo requires VINS-Fusion)
- Limited loop closing

#### Performance (EuRoC Dataset)
```
ATE RMSE: 0.05-0.15m
Processing speed: ~30 FPS
```

#### Installation (ROS)
```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ../
catkin_make
```

---

### 2.3 VINS-Fusion

**GitHub**: [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

#### Features
- Extended version of VINS-Mono
- Stereo and GPS fusion support
- Enhanced loop closing

#### Supported Sensors
- Mono/Stereo + IMU
- GPS fusion capable

#### Advantages
- Various sensor combinations
- GPS fusion for drift correction
- Online temporal calibration

#### Disadvantages
- More complex setup than VINS-Mono
- Increased resource usage

#### Performance
```
ATE RMSE: 0.04-0.12m (Stereo-Inertial)
Long-range drift minimized with GPS fusion
```

---

### 2.4 OpenVINS

**GitHub**: [rpng/open_vins](https://github.com/rpng/open_vins)
**Documentation**: [docs.openvins.com](https://docs.openvins.com/)

#### Features
- MSCKF-based filter approach
- Modular design optimized for research
- Excellent documentation

#### Supported Sensors
- Mono/Stereo + IMU

#### Advantages
- **Best-in-class documentation**
- Modular code structure
- Online calibration support
- CPU efficient

#### Disadvantages
- No loop closing (VIO only)
- Lower accuracy compared to optimization-based methods

#### Performance
```
ATE RMSE: 0.06-0.20m
CPU usage: Lowest level
```

#### Installation
```bash
# ROS 1
cd ~/catkin_ws/src
git clone https://github.com/rpng/open_vins.git
cd ../
catkin_make

# ROS 2
colcon build --packages-select ov_core ov_msckf
```

---

### 2.5 Basalt

**GitHub**: [VladyslavUsenko/basalt-mirror](https://gitlab.com/VladyslavUsenko/basalt)

#### Features
- Graph-based VIO
- Very fast processing speed
- Suitable for XR applications

#### Supported Sensors
- Stereo + IMU
- Fisheye camera support

#### Advantages
- **Fastest processing speed**
- Good software engineering
- BSD license (commercial use allowed)
- Excellent CI/documentation

#### Disadvantages
- No monocular support
- No loop closing

#### Performance
```
ATE RMSE: 0.05-0.15m
Processing speed: ~100+ FPS (fastest)
```

#### Installation
```bash
# Dependencies
sudo apt install libopencv-dev libfmt-dev

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

### 2.6 Kimera

**GitHub**: [MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera)

#### Features
- Metric-Semantic SLAM
- Modular architecture
- 3D mesh reconstruction

#### Component Modules
1. **Kimera-VIO**: Visual-Inertial Odometry
2. **Kimera-RPGO**: Robust Pose Graph Optimization
3. **Kimera-Mesher**: 3D Mesh generation
4. **Kimera-Semantics**: Semantic Reconstruction

#### Advantages
- 3D mesh and semantic map generation
- Individual module usage possible
- Research extensibility

#### Disadvantages
- Complex setup
- High resource usage
- Variable real-world performance

#### Performance
```
ATE RMSE: 0.10-0.30m (environment dependent)
3D mesh quality: Excellent
```

---

### 2.7 ROVIO

**GitHub**: [ethz-asl/rovio](https://github.com/ethz-asl/rovio)

#### Features
- EKF-based direct VIO
- No feature extraction required
- Lightweight implementation

#### Advantages
- Fast processing speed
- Low resource usage
- Works in low-texture environments

#### Disadvantages
- Scale drift possible
- No loop closing
- Limited accuracy

---

## 3. Performance Benchmark Comparison

### EuRoC MAV Dataset Results

| Algorithm | MH_01 | MH_03 | MH_05 | V1_02 | V2_03 | Average |
|-----------|-------|-------|-------|-------|-------|---------|
| ORB-SLAM3 VI | 0.033 | 0.027 | 0.061 | 0.016 | 0.020 | **0.031** |
| VINS-Fusion | 0.078 | 0.091 | 0.128 | 0.066 | 0.158 | 0.104 |
| OpenVINS | 0.086 | 0.103 | 0.220 | 0.064 | 0.148 | 0.124 |
| Basalt | 0.065 | 0.047 | 0.095 | 0.037 | 0.102 | 0.069 |

*(ATE RMSE, unit: m)*

### Resource Usage Comparison

| Algorithm | CPU Usage | Memory Usage | Real-time Capable |
|-----------|-----------|--------------|-------------------|
| ORB-SLAM3 | High | High | Conditional |
| VINS-Fusion | Medium | Medium | Yes |
| OpenVINS | **Low** | Low | Yes |
| Basalt | Low | **Low** | **Yes** |
| Kimera | High | High | Conditional |

## 4. Selection Guide

### Recommendations by Purpose

| Purpose | Recommended Framework | Reason |
|---------|----------------------|--------|
| Highest accuracy | ORB-SLAM3 | Top benchmark performance |
| Real-time AR/VR | Basalt | Fastest speed, BSD license |
| Research/learning | OpenVINS | Best documentation, modular |
| Asynchronous sensors | VINS-Mono/Fusion | Online temporal calibration |
| 3D reconstruction | Kimera | Mesh and semantic support |
| Commercial use | Basalt | BSD license |

### Smartphone Data Suitability

| Framework | Smartphone Suitability | Reason |
|-----------|------------------------|--------|
| VINS-Mono | **High** | Automatic time offset estimation |
| VINS-Fusion | **High** | VINS-Mono + Stereo/GPS |
| OpenVINS | Medium | Calibration required |
| ORB-SLAM3 | Low | Precise synchronization required |
| Basalt | Medium | Stereo required |

## 5. Installation Requirements

### Common Dependencies

```bash
# Ubuntu 20.04/22.04 based
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libopencv-dev libeigen3-dev \
  libboost-all-dev libgoogle-glog-dev \
  libceres-dev
```

### ROS Dependencies (ROS-based Frameworks)

```bash
# ROS 1 (Noetic)
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport

# ROS 2 (Humble)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

## 6. Datasets

### Benchmark Datasets

| Dataset | Environment | Sensors | Link |
|---------|-------------|---------|------|
| EuRoC MAV | Indoor drone | Stereo + IMU | [projects.asl.ethz.ch](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) |
| TUM VI | Indoor/outdoor | Stereo + IMU | [vision.in.tum.de](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) |
| KITTI | Autonomous driving | Stereo + GPS/IMU | [cvlibs.net](https://www.cvlibs.net/datasets/kitti/) |
| OpenLORIS | Indoor robot | RGB-D + IMU | [lifelong-robotic-vision.github.io](https://lifelong-robotic-vision.github.io/) |

## References

- [Comparison of modern open-source Visual SLAM approaches](https://arxiv.org/pdf/2108.01654)
- [Visual-Inertial SLAM Comparison (Bharat Joshi)](https://joshi-bharat.github.io/projects/visual_slam_comparison/)
- [VI-SLAM Benchmarking (Journal of Field Robotics 2025)](https://onlinelibrary.wiley.com/doi/10.1002/rob.22581)
- [HybVIO Paper (WACV 2022)](https://openaccess.thecvf.com/content/WACV2022/papers/Seiskari_HybVIO_Pushing_the_Limits_of_Real-Time_Visual-Inertial_Odometry_WACV_2022_paper.pdf)
- [Kimera Paper (arXiv 2019)](https://arxiv.org/abs/1910.02490)
