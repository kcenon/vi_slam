# VI-SLAM (Visual-Inertial SLAM) Overview

## 1. What is VI-SLAM?

Visual-Inertial SLAM (VI-SLAM) is a technology that performs Simultaneous Localization and Mapping (SLAM) by combining cameras (visual sensors) and IMUs (Inertial Measurement Units).

### Key Concepts

- **Visual SLAM**: Recognizes the environment and estimates position using camera images only
- **IMU**: Composed of accelerometer and gyroscope, provides inertial data
- **VI-SLAM**: A fusion system that leverages the complementary characteristics of both sensors

### Advantages of VI-SLAM

| Characteristic | Visual Only | IMU Only | VI-SLAM |
|----------------|-------------|----------|---------|
| Scale estimation | Impossible (monocular) | Possible | Possible |
| Fast motion | Vulnerable | Robust | Robust |
| Motion blur | Vulnerable | Unaffected | Can compensate |
| Drift accumulation | Present | Severe | Minimized |
| Textureless environment | Vulnerable | Unaffected | Can compensate |

## 2. VI-SLAM Classification

### 2.1 Classification by Sensor Fusion Method

#### Loosely Coupled
- Visual and Inertial systems operate independently
- Combines outputs of each system through post-processing
- Simple implementation but limited accuracy

#### Tightly Coupled
- Processes Visual and Inertial data in a single optimization framework
- Higher accuracy and robustness
- Mainstream approach in modern VI-SLAM

### 2.2 Classification by Estimation Method

#### Filter-based
- **Method**: Uses EKF (Extended Kalman Filter), MSCKF, etc.
- **Representative algorithms**: ROVIO, MSCKF, OpenVINS
- **Features**: Favorable for real-time processing, computationally efficient
- **Drawbacks**: Linearization error can accumulate

#### Optimization-based
- **Method**: Uses Bundle Adjustment, Graph Optimization
- **Representative algorithms**: ORB-SLAM3, VINS-Mono, VINS-Fusion, Basalt
- **Features**: Higher accuracy, excellent nonlinearity handling
- **Drawbacks**: High computational cost

## 3. Major VI-SLAM Algorithms

### 3.1 ORB-SLAM3
- **Developer**: University of Zaragoza
- **Features**: Most complete feature-based SLAM implementation
- **Support**: Monocular, Stereo, RGB-D + IMU
- **Advantages**: High accuracy, loop closing, map reuse
- **Disadvantages**: Sensor synchronization required, complex setup

### 3.2 VINS-Mono / VINS-Fusion
- **Developer**: HKUST (Hong Kong University of Science and Technology)
- **Features**: Tightly-coupled optimization-based
- **Advantages**: Online temporal calibration support, allows asynchronous sensors
- **VINS-Fusion additional features**: Stereo support, GPS fusion capable

### 3.3 OpenVINS
- **Developer**: University of Delaware
- **Features**: MSCKF-based filter approach
- **Advantages**: Excellent modularity, suitable for research, well documented
- **Support**: Monocular/Stereo + IMU

### 3.4 Basalt
- **Developer**: TUM (Technical University of Munich)
- **Features**: Graph-based VIO
- **Advantages**: Very fast processing speed, suitable for XR applications
- **Features**: Good software engineering practices, uses CI

### 3.5 Kimera
- **Developer**: MIT
- **Features**: Metric-Semantic SLAM
- **Components**: VIO module + Pose Graph Optimizer + 3D Mesher + Semantic Reconstruction
- **Advantages**: Modular design, 3D mesh reconstruction capable

### 3.6 ROVIO
- **Developer**: ETH Zurich
- **Features**: EKF-based direct method
- **Advantages**: No feature extraction needed, fast processing
- **Disadvantages**: Scale drift may occur

## 4. Performance Comparison Summary

| Algorithm | Method | Accuracy | Speed | Robustness | Implementation Difficulty |
|-----------|--------|----------|-------|------------|---------------------------|
| ORB-SLAM3 | Optimization | Best | Medium | High | High |
| VINS-Fusion | Optimization | High | Medium | High | Medium |
| OpenVINS | Filter | High | Fast | Medium | Low |
| Basalt | Optimization | High | Best | High | Medium |
| Kimera | Hybrid | Medium | Medium | Medium | High |

## 5. Application Areas

- **Autonomous navigation**: Drones, robots, autonomous vehicles
- **AR/VR**: Augmented reality, virtual reality headsets
- **Mobile mapping**: Indoor/outdoor 3D map generation
- **Industrial robots**: Logistics robots, AGV

## 6. Key Challenges

1. **Dynamic environments**: Robustness in environments with many moving objects
2. **Lighting changes**: Response to rapid lighting changes
3. **Scale Drift**: Accumulated error during long-term operation
4. **Initialization**: Accurate IMU bias estimation and system initialization
5. **Calibration**: Accurate spatiotemporal calibration between camera and IMU

## References

- [A review of visual SLAM for robotics (Frontiers 2024)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1347985/full)
- [Visual and Visual-Inertial SLAM Benchmarking (Wiley 2021)](https://onlinelibrary.wiley.com/doi/10.1155/2021/2054828)
- [RPG - Visual-Inertial Odometry and SLAM](https://rpg.ifi.uzh.ch/research_vo.html)
- [A Comprehensive Survey of Visual SLAM Algorithms (MDPI)](https://www.mdpi.com/2218-6581/11/1/24)
- [Comparison of modern open-source Visual SLAM approaches](https://arxiv.org/pdf/2108.01654)
