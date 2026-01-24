# VI-SLAM Project: Smartphone to PC Streaming

## Project Overview

A project to collect camera and IMU data from a smartphone and stream it to a PC in real-time (or with minor delay tolerance) to implement VI-SLAM.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Smartphone (Android/iOS)                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │    Camera    │    │     IMU      │    │   Timestamp  │       │
│  │   (30Hz)     │    │   (200Hz)    │    │     Sync     │       │
│  └──────┬───────┘    └──────┬───────┘    └──────────────┘       │
│         │                   │                                    │
│         ▼                   ▼                                    │
│  ┌──────────────────────────────────────┐                       │
│  │         Data Collection App           │                       │
│  │  (OpenCamera Sensors / MARS Logger)   │                       │
│  └──────────────────┬───────────────────┘                       │
│                     │                                            │
└─────────────────────┼────────────────────────────────────────────┘
                      │
          ┌───────────┴───────────┐
          │    Network Layer      │
          │  WebRTC / RTSP / UDP  │
          └───────────┬───────────┘
                      │
┌─────────────────────┼────────────────────────────────────────────┐
│                     ▼                        PC (Ubuntu/Windows)  │
│  ┌──────────────────────────────────────┐                       │
│  │           Data Receiver              │                       │
│  │    ┌─────────┐    ┌─────────┐       │                       │
│  │    │  Video  │    │   IMU   │       │                       │
│  │    │ Buffer  │    │ Buffer  │       │                       │
│  │    └────┬────┘    └────┬────┘       │                       │
│  │         └──────┬───────┘            │                       │
│  │                ▼                     │                       │
│  │    ┌─────────────────────┐          │                       │
│  │    │ Time Synchronization │          │                       │
│  │    └──────────┬──────────┘          │                       │
│  └───────────────┼──────────────────────┘                       │
│                  ▼                                               │
│  ┌──────────────────────────────────────┐                       │
│  │           VI-SLAM Engine             │                       │
│  │   (VINS-Mono / OpenVINS / Basalt)    │                       │
│  └──────────────────────────────────────┘                       │
│                  │                                               │
│                  ▼                                               │
│  ┌──────────────────────────────────────┐                       │
│  │             Output                    │                       │
│  │  • 6DoF Pose (position + rotation)   │                       │
│  │  • 3D Point Cloud / Map              │                       │
│  │  • Trajectory Visualization          │                       │
│  └──────────────────────────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
```

## Reference Documents

### Foundation Materials (VI-SLAM Concepts and Systems)

| # | Document | Content |
|---|----------|---------|
| 01 | [VI-SLAM Overview](01_vi_slam_overview.md) | VI-SLAM basic concepts, classification, major algorithms |
| 02 | [Smartphone Data Collection](02_smartphone_data_collection.md) | Smartphone sensor data collection apps and methods |
| 03 | [Streaming Protocols](03_streaming_protocols.md) | Real-time streaming protocol comparison (WebRTC, RTSP, etc.) |
| 04 | [Calibration & Synchronization](04_calibration_synchronization.md) | Camera-IMU calibration and time synchronization |
| 05 | [VI-SLAM Frameworks](05_vislam_frameworks.md) | Open-source VI-SLAM framework comparison |

### App Development Guide (Platform-specific APIs and Implementation)

| # | Document | Content |
|---|----------|---------|
| 06 | [Android Camera2 API](06_android_camera2_api.md) | Camera2 API detailed guide, timestamps, settings |
| 07 | [Android Sensor API (IMU)](07_android_sensor_api.md) | SensorManager, high-frequency sampling, data storage |
| 08 | [iOS AVFoundation & CoreMotion](08_ios_avfoundation_coremotion.md) | iOS Camera/IMU API, Swift implementation examples |
| 09 | [Timestamp Synchronization](09_timestamp_synchronization.md) | Camera-IMU time synchronization implementation methods |
| 10 | [Data Format & Streaming](10_data_format_streaming.md) | Data formats (CSV, Binary, Protobuf), streaming implementation |
| 11 | [App Development Guide](11_app_development_guide.md) | App architecture, core components, calibration workflow |

### Advanced Sensors (Smartphone IMU Characteristics and Optimization)

| # | Document | Content |
|---|----------|---------|
| 12 | [Smartphone IMU Drift](12_smartphone_imu_drift.md) | MEMS IMU error types, Allan Variance, drift correction techniques |
| 13 | [IMU Advanced Considerations](13_imu_advanced_considerations.md) | Sampling jitter, magnetometer fusion, ZUPT, power management, failure recovery |

### VI-SLAM Framework Configuration Guide

| # | Document | Content |
|---|----------|---------|
| 14 | [VINS-Mono Smartphone Config](14_vins_mono_smartphone_config.md) | VINS-Mono smartphone setup, parameter tuning, troubleshooting |
| 15 | [OpenVINS Smartphone Config](15_openvins_smartphone_config.md) | OpenVINS MSCKF setup, Kalibr integration, calibration workflow |
| 16 | [ORB-SLAM3 Smartphone Config](16_orbslam3_smartphone_config.md) | ORB-SLAM3 Mono-Inertial setup, Tbc transformation, time synchronization |
| 17 | [Basalt Smartphone Config](17_basalt_smartphone_config.md) | Basalt VIO setup, Optical Flow tuning, real-time processing optimization |
| 18 | [Framework Selection Guide](18_framework_selection_guide.md) | Framework comparison, scenario-based selection guide, benchmarks |

### Practical Guide

| # | Document | Content |
|---|----------|---------|
| 19 | [Calibration Practical Guide](19_calibration_practical_guide.md) | Step-by-step calibration tutorial, Kalibr/OpenCV usage, conversion scripts |

## Recommended Technology Stack

### Smartphone App (Data Collection)

| Option | Platform | Advantages | Disadvantages |
|--------|----------|------------|---------------|
| **OpenCamera Sensors** | Android | Open source, remote control, multi-device | No iOS support |
| **MARS Logger** | Android/iOS | Both platforms, AR optimized | Limited customization |
| **Custom App** | Choice | Full control | Development time required |

### Streaming Protocols

| Scenario | Recommended Protocol | Latency |
|----------|---------------------|---------|
| Real-time (< 100ms) | WebRTC | ~300ms |
| Low latency tolerance | RTSP + UDP(IMU) | ~500ms-1s |
| Offline processing | Save to file then transfer | N/A |

### VI-SLAM Framework

| Scenario | Recommended Framework | Reason |
|----------|----------------------|--------|
| **Smartphone data (recommended)** | VINS-Mono/Fusion | Auto time offset estimation |
| Highest accuracy required | ORB-SLAM3 | Top benchmark performance |
| Research/learning | OpenVINS | Excellent documentation |
| Real-time AR/VR | Basalt | Fastest processing speed |

## Implementation Roadmap (Proposed)

### Phase 1: Environment Setup and Offline Testing
1. Install VI-SLAM framework on PC (VINS-Mono recommended)
2. Verify operation with public dataset (EuRoC)
3. Install and test smartphone data collection app
4. Test VI-SLAM with offline data

### Phase 2: Calibration
1. Camera intrinsic calibration
2. Camera-IMU extrinsic calibration
3. Time synchronization verification

### Phase 3: Real-time Streaming Implementation
1. Select and implement streaming protocol
2. Develop PC receiver module
3. Implement data synchronization module

### Phase 4: Integration and Optimization
1. Integrate full pipeline
2. Optimize latency
3. Stability testing

## Key Challenges and Solutions

| Challenge | Impact | Solution |
|-----------|--------|----------|
| Camera-IMU time offset | Accuracy degradation | Use VINS family (auto estimation) |
| Network delay/instability | Real-time processing impossible | Buffering + timestamp synchronization |
| Rolling Shutter | Error during fast motion | Use VINS-Mono (RS model support) |
| OIS lens movement | Calibration error | Utilize OIS data or external camera |
| Lighting changes | Tracking failure | Fixed exposure, histogram normalization |

## Required Hardware

### Minimum Requirements

**Smartphone:**
- Android 8.0+ (Camera2 API support)
- Or iOS 11+
- Built-in gyroscope and accelerometer

**PC:**
- Ubuntu 18.04/20.04/22.04 (ROS support)
- CPU: Intel i5 or higher
- RAM: 8GB or more
- GPU: Optional (for acceleration)

**Network:**
- 5GHz WiFi (recommended)
- Or USB tethering

### Recommended Specifications

**Smartphone:**
- Device with Camera2 API timestamp synchronization support
- Recent flagship device (higher chance of OIS data availability)

**PC:**
- Ubuntu 20.04 + ROS Noetic
- CPU: Intel i7 or AMD Ryzen 7 or higher
- RAM: 16GB or more
- SSD storage

## Useful Links

### VI-SLAM Frameworks
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)
- [Basalt GitLab](https://gitlab.com/VladyslavUsenko/basalt)

### Data Collection
- [OpenCamera Sensors GitHub](https://github.com/prime-slam/OpenCamera-Sensors)
- [MARS Logger GitHub](https://github.com/OSUPCVLab/mobile-ar-sensor-logger)

### Calibration
- [Kalibr GitHub](https://github.com/ethz-asl/kalibr)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)

### Datasets
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM VI Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)

---

*Document created: 2026-01-19*
*Version: 0.1.8.0 (Added calibration practical guide)*

<!-- Version policy: x.major.minor.hotfix -->
<!-- 0.1.0.0: Initial document structure (01-11) -->
<!-- 0.1.1.0: IMU drift document (12) -->
<!-- 0.1.2.0: IMU advanced considerations (13) -->
<!-- 0.1.3.0: VINS-Mono config (14) -->
<!-- 0.1.4.0: OpenVINS config (15) -->
<!-- 0.1.5.0: ORB-SLAM3 config (16) -->
<!-- 0.1.6.0: Basalt config (17) -->
<!-- 0.1.7.0: Framework selection guide (18) -->
<!-- 0.1.8.0: Calibration practical guide (19) -->
