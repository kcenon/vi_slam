# Product Requirements Document (PRD)
# VI-SLAM: Smartphone Visual-Inertial SLAM System

---

**Document Information**

| Item | Details |
|------|---------|
| Document Version | 1.0.0 |
| Created Date | 2026-01-20 |
| Status | Draft |
| Author | VI-SLAM Project Team |
| Approver | - |

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Product Vision](#2-product-vision)
3. [Problem Definition](#3-problem-definition)
4. [Goals and Success Metrics](#4-goals-and-success-metrics)
5. [User Analysis](#5-user-analysis)
6. [Use Cases](#6-use-cases)
7. [Functional Requirements](#7-functional-requirements)
8. [Non-Functional Requirements](#8-non-functional-requirements)
9. [Technical Requirements](#9-technical-requirements)
10. [System Architecture](#10-system-architecture)
11. [Constraints and Assumptions](#11-constraints-and-assumptions)
12. [Roadmap and Milestones](#12-roadmap-and-milestones)
13. [Risk Analysis](#13-risk-analysis)
14. [Appendix](#14-appendix)

---

## 1. Executive Summary

### 1.1 Product Overview

**VI-SLAM** is a system that performs Visual-Inertial SLAM by streaming camera and IMU sensor data from a smartphone to a PC in real-time. It enables 6DoF pose estimation and 3D environment mapping using only a regular smartphone, without the need for expensive dedicated equipment.

### 1.2 Core Value Proposition

| Value | Description |
|-------|-------------|
| **Accessibility** | Implement VI-SLAM with a smartphone without expensive equipment |
| **Real-time** | Real-time pose estimation with less than 100ms latency |
| **Accuracy** | Achieve position estimation error of less than 0.1m |
| **Extensibility** | Support integration with various VI-SLAM frameworks |

### 1.3 Target Market

- AR/VR application developers
- Robotics researchers and developers
- Indoor positioning solution developers
- 3D scanning/mapping service providers

---

## 2. Product Vision

### 2.1 Vision Statement

> "Enable anyone to leverage precise spatial awareness technology with just the smartphone in their hand."

### 2.2 Long-term Goals

```
Year 1: Complete smartphone → PC real-time VI-SLAM system
Year 2: Implement on-device lightweight VI-SLAM
Year 3: Cloud-based multi-user collaborative mapping platform
```

### 2.3 Product Positioning

```
                    High
                     ↑
           Accuracy  │    ┌─────────────┐
                     │    │  Dedicated  │
                     │    │  Equipment  │
                     │    │  (LiDAR etc)│
                     │    └─────────────┘
                     │              ┌─────────────┐
                     │              │  VI-SLAM    │ ← Target
                     │              │(This Product)│
                     │              └─────────────┘
                     │    ┌─────────────┐
                     │    │  GPS-based  │
                     │    │(Outdoor Only)│
                     │    └─────────────┘
           Low  ─────┼────────────────────────────→ High
                     │                          Accessibility
                   High Cost                    Low Cost
```

---

## 3. Problem Definition

### 3.1 Current State (As-Is)

| Problem Area | Current State | Impact |
|--------------|---------------|--------|
| **Equipment Cost** | Dedicated VI-SLAM equipment $500-$5,000 | High entry barrier |
| **Technical Complexity** | Difficult sensor synchronization and calibration | Development time 6+ months |
| **Lack of Integration** | Scattered open source, insufficient documentation | High trial-and-error costs |
| **Real-time Processing** | Smartphone computational limitations | Latency issues |

### 3.2 Core Problems to Solve

#### P1: Sensor Data Synchronization Problem

```
Problem: Smartphone camera (30Hz) and IMU (200Hz) use different clocks
         → Even a few milliseconds of time offset is critical for SLAM accuracy

Impact:
- Position estimation error increases by 10x or more
- Tracking failure during rapid movements
- Unstable initialization
```

#### P2: Real-time Data Transmission Problem

```
Problem: Simultaneous transmission of high-bandwidth video + low-latency IMU data
         → Network delays cause SLAM performance degradation

Impact:
- Real-time applications impossible with delays over 100ms
- Data continuity destroyed during packet loss
- Additional delays due to buffering
```

#### P3: Smartphone Sensor Quality Problem

```
Problem: High noise and drift in smartphone MEMS IMU
         → Low-quality inertial data degrades SLAM accuracy

Impact:
- Integration error accumulation due to bias drift
- Image distortion due to Rolling Shutter
- Calibration errors due to OIS lens movement
```

### 3.3 Target State (To-Be)

| Problem Area | Target State | Improvement Effect |
|--------------|--------------|-------------------|
| **Equipment Cost** | Utilize existing smartphone ($0 additional cost) | Entry barrier eliminated |
| **Technical Complexity** | Integrated guide and automated setup | Development time within 1 month |
| **Lack of Integration** | End-to-end solution provided | Minimize trial-and-error |
| **Real-time Processing** | Real-time processing via PC offloading | < 100ms latency |

---

## 4. Goals and Success Metrics

### 4.1 Product Goals

#### Primary Goals (MVP)

| Goal | Description | Priority |
|------|-------------|----------|
| **G1** | Develop smartphone data collection app | P0 |
| **G2** | Develop PC real-time receiver module | P0 |
| **G3** | VINS-Mono integration | P0 |
| **G4** | Offline data processing pipeline | P1 |

#### Secondary Goals (v1.0)

| Goal | Description | Priority |
|------|-------------|----------|
| **G5** | Implement real-time streaming (< 100ms) | P1 |
| **G6** | Multi-framework support (OpenVINS, ORB-SLAM3) | P1 |
| **G7** | Automatic calibration tool | P2 |
| **G8** | iOS app development | P2 |

### 4.2 Success Metrics (KPIs)

#### Technical Metrics

| Metric | Target Value | Measurement Method |
|--------|--------------|-------------------|
| **Position Accuracy (ATE RMSE)** | ≤ 0.1m | Compared to EuRoC benchmark |
| **End-to-end Latency** | ≤ 100ms | Network RTT + processing time |
| **IMU Sampling Rate** | ≥ 200Hz | In-app measurement |
| **Frame Drop Rate** | ≤ 1% | During continuous recording |
| **System Uptime** | ≥ 99% | 30-minute continuous operation |

#### Usability Metrics

| Metric | Target Value | Measurement Method |
|--------|--------------|-------------------|
| **Installation to First Run Time** | ≤ 30 min | User testing |
| **Calibration Time** | ≤ 10 min | Guide-based |
| **Documentation Completeness** | 100% | All features documented |

#### Business Metrics

| Metric | Target Value | Measurement Method |
|--------|--------------|-------------------|
| **GitHub Stars** | ≥ 500 (within 1 year) | GitHub metrics |
| **Community Contributions** | ≥ 10 PRs/year | GitHub metrics |
| **Supported Devices** | ≥ 20 models | Compatibility testing |

---

## 5. User Analysis

### 5.1 User Personas

#### Persona 1: AR Developer Alex

```
┌─────────────────────────────────────────────────────────┐
│ Alex (32, AR Startup Developer)                          │
├─────────────────────────────────────────────────────────┤
│ Background:                                              │
│ • 3 years experience developing AR apps with Unity/Unreal│
│ • Has experience with ARCore/ARKit                       │
│ • Knows SLAM theory but no implementation experience     │
├─────────────────────────────────────────────────────────┤
│ Goals:                                                   │
│ • Overcome ARCore accuracy limitations                   │
│ • Develop own pose estimation system                     │
│ • Develop AR app capable of offline operation            │
├─────────────────────────────────────────────────────────┤
│ Pain Points:                                             │
│ • Difficulty understanding SLAM papers/code              │
│ • Doesn't know sensor calibration methods                │
│ • Complex test environment setup                         │
├─────────────────────────────────────────────────────────┤
│ Expected Solution:                                       │
│ • Step-by-step guide documentation                       │
│ • Working sample code                                    │
│ • Fast prototyping environment                           │
└─────────────────────────────────────────────────────────┘
```

#### Persona 2: Robotics Researcher Sarah

```
┌─────────────────────────────────────────────────────────┐
│ Sarah (28, Graduate Robotics Researcher)                 │
├─────────────────────────────────────────────────────────┤
│ Background:                                              │
│ • Master's program in Computer Vision/SLAM               │
│ • Extensive ROS experience                               │
│ • Needs research data collection                         │
├─────────────────────────────────────────────────────────┤
│ Goals:                                                   │
│ • Collect comparative experiment data for papers         │
│ • Test new algorithms                                    │
│ • Evaluate performance in various environments           │
├─────────────────────────────────────────────────────────┤
│ Pain Points:                                             │
│ • Lab budget constraints prevent expensive equipment     │
│ • Existing datasets lack diversity                       │
│ • Difficulty verifying sensor synchronization            │
├─────────────────────────────────────────────────────────┤
│ Expected Solution:                                       │
│ • Low-cost data collection environment                   │
│ • Accurate timestamp synchronization                     │
│ • Integration with various SLAM frameworks               │
└─────────────────────────────────────────────────────────┘
```

#### Persona 3: Indoor Positioning Solution CTO Mike

```
┌─────────────────────────────────────────────────────────┐
│ Mike (40, Indoor Positioning Startup CTO)                │
├─────────────────────────────────────────────────────────┤
│ Background:                                              │
│ • 10 years experience in location-based services         │
│ • Currently operating BLE beacon-based indoor positioning│
│ • Exploring accuracy improvement methods                 │
├─────────────────────────────────────────────────────────┤
│ Goals:                                                   │
│ • Reduce beacon installation costs                       │
│ • Achieve positioning accuracy within 1m                 │
│ • Fast PoC development                                   │
├─────────────────────────────────────────────────────────┤
│ Pain Points:                                             │
│ • Existing beacon solution accuracy limitations (2-5m)   │
│ • High initial infrastructure installation costs         │
│ • New technology verification risks                      │
├─────────────────────────────────────────────────────────┤
│ Expected Solution:                                       │
│ • Infrastructure-free positioning solution               │
│ • Commercial license (BSD)                               │
│ • Technical support and documentation                    │
└─────────────────────────────────────────────────────────┘
```

### 5.2 User Journey Map

```
┌──────────────────────────────────────────────────────────────────────────┐
│                            User Journey Map                               │
├──────────────────────────────────────────────────────────────────────────┤
│ Stage   │ Awareness   │ Exploration │ Installation│ Usage       │Extension│
├──────────────────────────────────────────────────────────────────────────┤
│ Actions │ Search SLAM │ Check docs  │ Setup env   │ Collect data│ Custom- │
│         │ Find GitHub │ Requirements│ Install app │ Run SLAM    │ ization │
│         │             │ review      │ PC setup    │             │         │
├──────────────────────────────────────────────────────────────────────────┤
│ Touch   │ GitHub      │ README      │ Install     │ App UI      │ API     │
│ Points  │ search      │ Doc site    │ guide       │ Terminal    │ docs    │
│         │ results     │             │ scripts     │ output      │         │
├──────────────────────────────────────────────────────────────────────────┤
│ Emotion │ Expectation │ Interest ↗  │ Anxiety ↘   │ Satisfied ↗ │Challenge│
│         │             │ "This is it!"│"Will it work?"│"It works!"│Achievement│
├──────────────────────────────────────────────────────────────────────────┤
│ Pain    │ Scattered   │ Unclear     │ Dependency  │ Calibration │ Lacking │
│ Points  │ information │ requirements│ conflicts   │ difficulty  │ APIs    │
├──────────────────────────────────────────────────────────────────────────┤
│ Opport- │ SEO         │ Quick Start │ One-click   │ Auto        │ Plugin  │
│ unities │ optimization│ guide       │ install     │ calibration │ system  │
│         │             │             │ script      │             │         │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Use Cases

### 6.1 Use Case Diagram

```
                          ┌─────────────────────────────────┐
                          │         VI-SLAM System          │
                          │                                 │
    ┌─────────┐           │  ┌─────────────────────────┐   │
    │         │           │  │                         │   │
    │ AR/VR   │──────────────│UC1: Real-time Pose      │   │
    │Developer│           │  │     Estimation          │   │
    │         │──────┐    │  └─────────────────────────┘   │
    └─────────┘      │    │                                 │
                     │    │  ┌─────────────────────────┐   │
    ┌─────────┐      │    │  │                         │   │
    │         │      └───────│UC2: Data Collection/    │   │
    │Researcher│─────────────│     Storage             │   │
    │         │──────┐    │  └─────────────────────────┘   │
    └─────────┘      │    │                                 │
                     │    │  ┌─────────────────────────┐   │
    ┌─────────┐      │    │  │                         │   │
    │         │      └───────│UC3: Offline SLAM        │   │
    │ Mapping │──────────────│     Processing          │   │
    │ Engineer│           │  └─────────────────────────┘   │
    │         │──────┐    │                                 │
    └─────────┘      │    │  ┌─────────────────────────┐   │
                     │    │  │                         │   │
                     └───────│UC4: Sensor Calibration  │   │
                          │  │                         │   │
                          │  └─────────────────────────┘   │
                          │                                 │
                          │  ┌─────────────────────────┐   │
                          │  │                         │   │
                          │  │UC5: 3D Map Generation/  │   │
                          │  │     Storage             │   │
                          │  └─────────────────────────┘   │
                          │                                 │
                          └─────────────────────────────────┘
```

### 6.2 Detailed Use Cases

#### UC1: Real-time Pose Estimation

```
┌─────────────────────────────────────────────────────────────┐
│ UC1: Real-time Pose Estimation                               │
├─────────────────────────────────────────────────────────────┤
│ Actors: AR/VR Developer, Robot Operator                      │
│ Preconditions: Calibration completed, Network connected      │
│ Postconditions: Continuous 6DoF pose output                  │
├─────────────────────────────────────────────────────────────┤
│ Basic Flow:                                                  │
│ 1. User launches smartphone app                              │
│ 2. App verifies connection with PC server                    │
│ 3. User taps "Start" button                                  │
│ 4. App starts camera/IMU data capture                        │
│ 5. App streams data to PC in real-time                       │
│ 6. PC receives and synchronizes data                         │
│ 7. VI-SLAM engine estimates pose                             │
│ 8. System outputs 6DoF pose (30Hz)                           │
│ 9. User taps "Stop" button                                   │
│ 10. System ends session                                      │
├─────────────────────────────────────────────────────────────┤
│ Alternative Flows:                                           │
│ 4a. On tracking loss:                                        │
│     → System attempts re-initialization                      │
│     → Notifies user after 3 failures                         │
│ 5a. On network disconnection:                                │
│     → App stores data in local buffer                        │
│     → Synchronizes after connection recovery                 │
├─────────────────────────────────────────────────────────────┤
│ Success Metrics:                                             │
│ • End-to-end latency < 100ms                                 │
│ • Position accuracy < 0.1m                                   │
│ • Tracking retention rate > 95%                              │
└─────────────────────────────────────────────────────────────┘
```

#### UC2: Data Collection and Storage

```
┌─────────────────────────────────────────────────────────────┐
│ UC2: Data Collection and Storage                             │
├─────────────────────────────────────────────────────────────┤
│ Actors: Researcher, Data Engineer                            │
│ Preconditions: App installed, Storage space available        │
│ Postconditions: Synchronized dataset generated               │
├─────────────────────────────────────────────────────────────┤
│ Basic Flow:                                                  │
│ 1. User configures recording settings (resolution, IMU freq) │
│ 2. User taps "Record" button                                 │
│ 3. App captures camera/IMU data synchronously                │
│ 4. App stores data to local storage                          │
│ 5. App displays real-time statistics (frame count, IMU count)│
│ 6. User taps "Stop" button                                   │
│ 7. App generates metadata file                               │
│ 8. App verifies data integrity                               │
│ 9. User exports data                                         │
├─────────────────────────────────────────────────────────────┤
│ Output Files:                                                │
│ • video.mp4 (H.264 encoded video)                            │
│ • imu.csv (timestamp, ax, ay, az, gx, gy, gz)                │
│ • metadata.json (calibration, device info)                   │
│ • frames/ (optional: individual frame images)                │
├─────────────────────────────────────────────────────────────┤
│ Success Metrics:                                             │
│ • Frame drop rate < 1%                                       │
│ • IMU sampling accuracy ±5%                                  │
│ • Timestamp accuracy < 1ms                                   │
└─────────────────────────────────────────────────────────────┘
```

#### UC3: Offline SLAM Processing

```
┌─────────────────────────────────────────────────────────────┐
│ UC3: Offline SLAM Processing                                 │
├─────────────────────────────────────────────────────────────┤
│ Actors: Researcher, Mapping Engineer                         │
│ Preconditions: Collected dataset, SLAM framework installed   │
│ Postconditions: Trajectory file, point cloud generated       │
├─────────────────────────────────────────────────────────────┤
│ Basic Flow:                                                  │
│ 1. User selects dataset directory                            │
│ 2. System validates data format                              │
│ 3. System loads selected SLAM framework configuration        │
│ 4. User clicks "Start Processing"                            │
│ 5. System feeds data to SLAM framework                       │
│ 6. System displays progress                                  │
│ 7. SLAM engine estimates trajectory                          │
│ 8. System saves result files                                 │
│ 9. System provides result visualization                      │
├─────────────────────────────────────────────────────────────┤
│ Output:                                                      │
│ • trajectory.txt (TUM format trajectory)                     │
│ • pointcloud.ply (3D point cloud)                            │
│ • keyframes/ (keyframe images)                               │
│ • report.html (processing statistics, visualization)         │
└─────────────────────────────────────────────────────────────┘
```

#### UC4: Sensor Calibration

```
┌─────────────────────────────────────────────────────────────┐
│ UC4: Sensor Calibration                                      │
├─────────────────────────────────────────────────────────────┤
│ Actors: All Users                                            │
│ Preconditions: Calibration target prepared (checkerboard/    │
│                Aprilgrid)                                    │
│ Postconditions: Calibration parameter file generated         │
├─────────────────────────────────────────────────────────────┤
│ Basic Flow:                                                  │
│                                                              │
│ [Camera Intrinsic Calibration]                               │
│ 1. Select calibration mode in app                            │
│ 2. User captures checkerboard from various angles (20+ shots)│
│ 3. App detects corners and provides feedback                 │
│ 4. App calculates camera parameters                          │
│ 5. App displays reprojection error                           │
│                                                              │
│ [Camera-IMU Extrinsic Calibration]                           │
│ 6. User moves device in various directions (60 seconds)      │
│ 7. System calculates extrinsic with Kalibr                   │
│ 8. System estimates time offset                              │
│                                                              │
│ [Verification]                                               │
│ 9. User records verification sequence                        │
│ 10. System verifies calibration accuracy                     │
│ 11. System saves calibration.yaml                            │
├─────────────────────────────────────────────────────────────┤
│ Success Metrics:                                             │
│ • Reprojection error < 0.5 pixel                             │
│ • Time offset estimation accuracy < 1ms                      │
└─────────────────────────────────────────────────────────────┘
```

### 6.3 User Stories

| ID | Role | Story | Priority | Acceptance Criteria |
|----|------|-------|----------|---------------------|
| US01 | AR Developer | I want to install the smartphone app and connect to PC to receive real-time pose | P0 | Connect within 30 sec, 30Hz pose output |
| US02 | Researcher | I want to save camera/IMU data with synchronization | P0 | Timestamp error < 1ms |
| US03 | Developer | I want to test various SLAM algorithms with saved data | P0 | 3+ framework support |
| US04 | All Users | I want to perform camera calibration within the app | P1 | Complete within 10 min |
| US05 | AR Developer | I want to use SLAM results in Unity/Unreal | P1 | SDK/plugin provided |
| US06 | Researcher | I want to compare performance across multiple smartphone models | P2 | Benchmark tool provided |
| US07 | iOS Developer | I want to collect data on iPhone as well | P2 | iOS app provided |
| US08 | Mapping Engineer | I want to export generated 3D maps | P2 | PLY/OBJ format support |

---

## 7. Functional Requirements

### 7.1 Functional Requirements Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     Functional Requirements Hierarchy                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                    Application Layer                             │    │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐       │    │
│  │  │ Android App   │  │   iOS App     │  │   PC Client   │       │    │
│  │  │  (FR1.x)      │  │  (FR2.x)      │  │   (FR3.x)     │       │    │
│  │  └───────────────┘  └───────────────┘  └───────────────┘       │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    │                                     │
│  ┌─────────────────────────────────▼───────────────────────────────┐    │
│  │                    Communication Layer                           │    │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐       │    │
│  │  │  Streaming    │  │    Sync       │  │   Protocol    │       │    │
│  │  │  (FR4.x)      │  │   (FR5.x)     │  │   (FR6.x)     │       │    │
│  │  └───────────────┘  └───────────────┘  └───────────────┘       │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    │                                     │
│  ┌─────────────────────────────────▼───────────────────────────────┐    │
│  │                    Processing Layer                              │    │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐       │    │
│  │  │  VI-SLAM      │  │ Calibration   │  │   Output      │       │    │
│  │  │  (FR7.x)      │  │   (FR8.x)     │  │   (FR9.x)     │       │    │
│  │  └───────────────┘  └───────────────┘  └───────────────┘       │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 7.2 FR1: Android App Features

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR1.1 | Camera Capture | Frame capture via Camera2 API (30Hz, 720p/1080p) | P0 |
| FR1.2 | IMU Capture | Accelerometer/gyroscope data capture (200Hz) | P0 |
| FR1.3 | Timestamp Synchronization | Integrated camera/IMU timestamp management | P0 |
| FR1.4 | Local Storage | Local storage of video/IMU data | P0 |
| FR1.5 | Real-time Streaming | Real-time data transmission to PC | P1 |
| FR1.6 | Settings UI | Resolution, FPS, IMU frequency settings | P1 |
| FR1.7 | Status Display | Connection status, FPS, buffer status display | P1 |
| FR1.8 | Calibration UI | In-app calibration guide | P2 |
| FR1.9 | OIS Disable | Option to disable optical image stabilization | P2 |

#### FR1.1 Detail: Camera Capture

```yaml
Feature Name: Camera Capture
ID: FR1.1
Priority: P0

Description: |
  Capture frames from rear camera using Camera2 API.
  Include accurate sensor timestamp with each frame.

Input:
  - Resolution setting (720p/1080p)
  - FPS setting (15/30/60)
  - Exposure mode (auto/manual)

Output:
  - YUV420 or JPEG frame
  - Sensor timestamp (nanoseconds)
  - Frame metadata (exposure time, ISO, etc.)

Requirements:
  - Device supporting Camera2 API HARDWARE_LEVEL_FULL or higher
  - Use SENSOR_TIMESTAMP
  - Stable capture at minimum 30Hz

Acceptance Criteria:
  - [ ] Frame drop rate < 1% at 30Hz
  - [ ] Timestamp accuracy < 1ms
  - [ ] No memory leaks (1 hour continuous operation)
```

#### FR1.2 Detail: IMU Capture

```yaml
Feature Name: IMU Capture
ID: FR1.2
Priority: P0

Description: |
  Capture accelerometer and gyroscope data at high frequency
  using SensorManager.

Input:
  - Target sampling frequency (100-500Hz)
  - Sensor type (Uncalibrated/Calibrated)

Output:
  - Acceleration (ax, ay, az) m/s²
  - Angular velocity (gx, gy, gz) rad/s
  - Sensor timestamp (nanoseconds)

Requirements:
  - Use SENSOR_DELAY_FASTEST
  - Provide Uncalibrated sensor option
  - Maintain recent data in circular buffer

Acceptance Criteria:
  - [ ] Actual sampling rate ≥ 95% of target
  - [ ] Sample loss rate < 0.1%
  - [ ] Guarantee monotonically increasing timestamps
```

### 7.3 FR2: iOS App Features (Phase 2)

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR2.1 | Camera Capture | Frame capture via AVFoundation | P2 |
| FR2.2 | IMU Capture | IMU data capture via CoreMotion | P2 |
| FR2.3 | Timestamp Synchronization | Camera/IMU timestamp integration | P2 |
| FR2.4 | Local Storage | Local data storage | P2 |
| FR2.5 | Streaming | Real-time transmission to PC | P2 |

### 7.4 FR3: PC Client Features

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR3.1 | Data Reception | Receive smartphone data | P0 |
| FR3.2 | Video Decoding | H.264/H.265 decoding | P0 |
| FR3.3 | Data Synchronization | Video/IMU timestamp alignment | P0 |
| FR3.4 | SLAM Integration | VI-SLAM framework integration | P0 |
| FR3.5 | Visualization | Real-time trajectory/point cloud display | P1 |
| FR3.6 | Data Storage | Local storage of received data | P1 |
| FR3.7 | ROS Integration | ROS topic publishing | P1 |

#### FR3.4 Detail: SLAM Framework Integration

```yaml
Feature Name: VI-SLAM Framework Integration
ID: FR3.4
Priority: P0

Supported Frameworks:
  Priority 1 (P0):
    - VINS-Mono: Optimal smartphone data support

  Priority 2 (P1):
    - OpenVINS: MSCKF-based, for research
    - ORB-SLAM3: Highest accuracy
    - Basalt: Fastest speed, BSD license

Interface:
  Input:
    - Image: cv::Mat (grayscale/color)
    - IMU: timestamp, acc[3], gyro[3]
    - Calibration: YAML file

  Output:
    - 6DoF pose: position (x,y,z), orientation (qw,qx,qy,qz)
    - Status: INITIALIZING, TRACKING, LOST
    - Optional: 3D point cloud

Acceptance Criteria:
  - [ ] VINS-Mono integration complete and EuRoC test passed
  - [ ] Position accuracy ATE RMSE < 0.15m
  - [ ] Real-time processing (input 30Hz, output 30Hz)
```

### 7.5 FR4: Streaming Features

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR4.1 | WebRTC Video | Video streaming via WebRTC | P1 |
| FR4.2 | UDP IMU | IMU data transmission via UDP | P1 |
| FR4.3 | Connection Management | Connection status monitoring and reconnection | P1 |
| FR4.4 | Buffering | Buffer to absorb network jitter | P1 |
| FR4.5 | Bandwidth Adaptation | Quality adjustment based on network conditions | P2 |

### 7.6 FR5: Synchronization Features

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR5.1 | Timestamp Alignment | Video/IMU timestamp alignment | P0 |
| FR5.2 | IMU Interpolation | IMU interpolation to match frame time | P0 |
| FR5.3 | Dropped Frame Handling | Maintain data continuity on frame drop | P1 |
| FR5.4 | Latency Compensation | Network latency measurement and compensation | P1 |

### 7.7 FR6: Data Formats

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR6.1 | EuRoC Format | EuRoC dataset format support | P0 |
| FR6.2 | TUM Format | TUM VI dataset format support | P1 |
| FR6.3 | ROS bag | rosbag format export | P1 |
| FR6.4 | CSV/Binary | Raw data CSV/Binary storage | P0 |

### 7.8 FR7: VI-SLAM Processing

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR7.1 | Initialization | VI-SLAM automatic initialization | P0 |
| FR7.2 | Tracking | Continuous pose estimation | P0 |
| FR7.3 | Map Management | Point cloud management | P1 |
| FR7.4 | Loop Closing | Loop detection and optimization | P2 |
| FR7.5 | Relocalization | Recovery after tracking loss | P1 |

### 7.9 FR8: Calibration

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR8.1 | Camera Intrinsic | Focal length, principal point, distortion coefficients | P0 |
| FR8.2 | Camera-IMU Extrinsic | Relative position/orientation | P1 |
| FR8.3 | Time Offset | Camera-IMU time offset | P1 |
| FR8.4 | IMU Noise Parameters | Allan Variance-based estimation | P2 |
| FR8.5 | Auto Calibration | Guide-based automation | P2 |

### 7.10 FR9: Output and Export

| ID | Feature | Description | Priority |
|----|---------|-------------|----------|
| FR9.1 | Trajectory Export | TUM/KITTI format trajectory | P0 |
| FR9.2 | Point Cloud | PLY/PCD format map | P1 |
| FR9.3 | Mesh | OBJ/PLY 3D mesh | P2 |
| FR9.4 | Real-time API | Pose streaming API | P1 |

---

## 8. Non-Functional Requirements

### 8.1 NFR1: Performance Requirements

| ID | Requirement | Target Value | Measurement Method |
|----|-------------|--------------|-------------------|
| NFR1.1 | End-to-end Latency | < 100ms | Timestamp difference |
| NFR1.2 | Position Accuracy | ATE RMSE < 0.1m | EuRoC benchmark |
| NFR1.3 | Processing Speed | ≥ 30 FPS | Framework output |
| NFR1.4 | IMU Sampling Rate | ≥ 200 Hz | In-app measurement |
| NFR1.5 | Frame Drop Rate | < 1% | During continuous recording |
| NFR1.6 | CPU Usage (App) | < 50% | Profiler |
| NFR1.7 | Memory Usage (App) | < 500 MB | Profiler |

### 8.2 NFR2: Reliability Requirements

| ID | Requirement | Target Value | Measurement Method |
|----|-------------|--------------|-------------------|
| NFR2.1 | System Uptime | ≥ 99% | 30-minute continuous operation |
| NFR2.2 | Tracking Retention Rate | ≥ 95% | Under normal conditions |
| NFR2.3 | Data Integrity | 100% | Checksum verification |
| NFR2.4 | Error Recovery | Auto-reconnect | On network disconnection |
| NFR2.5 | Crash Rate | < 0.1% | Per session |

### 8.3 NFR3: Usability Requirements

| ID | Requirement | Target Value | Measurement Method |
|----|-------------|--------------|-------------------|
| NFR3.1 | Installation Time | < 30 min | User testing |
| NFR3.2 | Calibration Time | < 10 min | Guide-based |
| NFR3.3 | Learning Curve | Basic use within 1 day | User feedback |
| NFR3.4 | Documentation Coverage | 100% | All features documented |
| NFR3.5 | Error Message Clarity | Include resolution method | User feedback |

### 8.4 NFR4: Compatibility Requirements

| ID | Requirement | Target Value |
|----|-------------|--------------|
| NFR4.1 | Android Version | 8.0 (API 26) or higher |
| NFR4.2 | iOS Version | 11.0 or higher (Phase 2) |
| NFR4.3 | PC OS | Ubuntu 20.04/22.04, Windows 10/11 (WSL) |
| NFR4.4 | ROS Version | Noetic, Humble |
| NFR4.5 | Supported Devices | ≥ 20 smartphone models |

### 8.5 NFR5: Security Requirements

| ID | Requirement | Description |
|----|-------------|-------------|
| NFR5.1 | Data Transmission Encryption | Use DTLS (WebRTC) |
| NFR5.2 | Local Data Protection | Use app-exclusive storage |
| NFR5.3 | Minimal Permissions | Request only necessary permissions |
| NFR5.4 | Network Authentication | Optional connection authentication |

### 8.6 NFR6: Maintainability Requirements

| ID | Requirement | Target Value |
|----|-------------|--------------|
| NFR6.1 | Code Coverage | ≥ 70% |
| NFR6.2 | Documentation Level | All public APIs |
| NFR6.3 | Module Coupling | Loose coupling |
| NFR6.4 | CI/CD | Automated build/test |

---

## 9. Technical Requirements

### 9.1 Hardware Requirements

#### Smartphone (Minimum/Recommended)

| Item | Minimum | Recommended |
|------|---------|-------------|
| **Android Version** | 8.0 | 11.0+ |
| **Camera2 API** | HARDWARE_LEVEL_LIMITED | HARDWARE_LEVEL_FULL |
| **RAM** | 2 GB | 4 GB+ |
| **Storage** | 100 MB (app) + 1 GB (data) | 10 GB+ |
| **Sensors** | Accelerometer, Gyroscope | + Magnetometer, Barometer |

#### PC (Minimum/Recommended)

| Item | Minimum | Recommended |
|------|---------|-------------|
| **OS** | Ubuntu 18.04 | Ubuntu 20.04/22.04 |
| **CPU** | Intel i5 / AMD Ryzen 5 | Intel i7 / AMD Ryzen 7 |
| **RAM** | 8 GB | 16 GB+ |
| **GPU** | Integrated graphics | NVIDIA GTX 1060+ |
| **Storage** | 50 GB | 200 GB+ SSD |

#### Network

| Item | Minimum | Recommended |
|------|---------|-------------|
| **Connection** | 2.4 GHz WiFi | 5 GHz WiFi / USB Tethering |
| **Bandwidth** | 5 Mbps | 20 Mbps+ |
| **Latency** | < 50 ms | < 20 ms |

### 9.2 Software Requirements

#### Development Environment

| Component | Tool | Version |
|-----------|------|---------|
| **Android App** | Android Studio | 2023.1+ |
| **Android SDK** | - | API 26-34 |
| **Kotlin** | - | 1.9+ |
| **PC Client** | Python | 3.8+ |
| **C++ Compiler** | GCC / Clang | 9+ / 10+ |
| **Build System** | CMake | 3.16+ |
| **ROS** | ROS Noetic / Humble | - |

#### Dependencies

```yaml
Android App:
  - Camera2 API
  - SensorManager
  - WebRTC (Google libwebrtc)
  - Gson (JSON processing)
  - Kotlin Coroutines

PC Client:
  - OpenCV: 4.5+
  - Eigen3: 3.3+
  - Ceres Solver: 2.0+
  - aiortc (WebRTC)
  - numpy, scipy

VI-SLAM Frameworks:
  - VINS-Mono: Own dependencies
  - OpenVINS: ROS dependency
  - ORB-SLAM3: Pangolin, DBoW2
  - Basalt: TBB, fmt
```

### 9.3 Data Format Specifications

#### IMU Data Format

```
# CSV format
# timestamp (ns), ax (m/s²), ay, az, gx (rad/s), gy, gz
1000000000,0.1,-0.2,9.8,0.001,-0.002,0.001
1005000000,0.1,-0.2,9.8,0.001,-0.002,0.001
...

# Binary format (56 bytes per sample)
struct ImuSample {
    int64_t timestamp;    // 8 bytes
    double acc[3];        // 24 bytes
    double gyro[3];       // 24 bytes
};
```

#### Video Metadata Format

```json
{
  "format_version": "1.0",
  "device": {
    "model": "Pixel 6",
    "manufacturer": "Google",
    "android_version": "12"
  },
  "camera": {
    "resolution": [1920, 1080],
    "fps": 30,
    "codec": "H.264",
    "timestamp_source": "SENSOR_TIMESTAMP"
  },
  "calibration": {
    "fx": 1500.0,
    "fy": 1500.0,
    "cx": 960.0,
    "cy": 540.0,
    "distortion": [-0.28, 0.07, 0.0, 0.0]
  },
  "recording": {
    "start_time": "2026-01-20T10:00:00Z",
    "duration_ms": 60000,
    "frame_count": 1800,
    "imu_sample_count": 12000
  }
}
```

---

## 10. System Architecture

### 10.1 Overall System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              VI-SLAM SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    SMARTPHONE (Android/iOS)                       │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐     │   │
│  │  │  Camera Module │  │   IMU Module   │  │  Sync Module   │     │   │
│  │  │  (Camera2 API) │  │ (SensorManager)│  │  (Timestamp)   │     │   │
│  │  └───────┬────────┘  └───────┬────────┘  └───────┬────────┘     │   │
│  │          │                   │                   │               │   │
│  │          └───────────────────┼───────────────────┘               │   │
│  │                              ▼                                    │   │
│  │  ┌──────────────────────────────────────────────────────────┐   │   │
│  │  │                   Data Manager                            │   │   │
│  │  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │   │   │
│  │  │  │   Buffer   │  │   Encoder  │  │  Streamer  │          │   │   │
│  │  │  │  Manager   │  │  (H.264)   │  │  (WebRTC)  │          │   │   │
│  │  │  └────────────┘  └────────────┘  └────────────┘          │   │   │
│  │  └──────────────────────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                    │                                     │
│                      ┌─────────────┴─────────────┐                      │
│                      │      Network Layer        │                      │
│                      │  ┌─────────┐ ┌─────────┐  │                      │
│                      │  │ WebRTC  │ │   UDP   │  │                      │
│                      │  │ (Video) │ │  (IMU)  │  │                      │
│                      │  └─────────┘ └─────────┘  │                      │
│                      └─────────────┬─────────────┘                      │
│                                    │                                     │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                         PC (Ubuntu/Windows)                       │   │
│  │  ┌──────────────────────────────────────────────────────────┐   │   │
│  │  │                   Data Receiver                           │   │   │
│  │  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │   │   │
│  │  │  │   Video    │  │    IMU     │  │    Time    │          │   │   │
│  │  │  │  Decoder   │  │   Buffer   │  │  Aligner   │          │   │   │
│  │  │  └──────┬─────┘  └──────┬─────┘  └──────┬─────┘          │   │   │
│  │  │         └───────────────┼───────────────┘                 │   │   │
│  │  └─────────────────────────┼─────────────────────────────────┘   │   │
│  │                            ▼                                      │   │
│  │  ┌──────────────────────────────────────────────────────────┐   │   │
│  │  │                   VI-SLAM Engine                          │   │   │
│  │  │  ┌──────────────────────────────────────────────────┐    │   │   │
│  │  │  │              Framework Adapter                    │    │   │   │
│  │  │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐         │    │   │   │
│  │  │  │  │VINS-Mono │ │ OpenVINS │ │ORB-SLAM3 │         │    │   │   │
│  │  │  │  └──────────┘ └──────────┘ └──────────┘         │    │   │   │
│  │  │  └──────────────────────────────────────────────────┘    │   │   │
│  │  └──────────────────────────────────────────────────────────┘   │   │
│  │                            │                                      │   │
│  │                            ▼                                      │   │
│  │  ┌──────────────────────────────────────────────────────────┐   │   │
│  │  │                   Output Manager                          │   │   │
│  │  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │   │   │
│  │  │  │ Pose API   │  │ Visualizer │  │  Exporter  │          │   │   │
│  │  │  │ (ROS/ZMQ)  │  │  (RViz)    │  │ (TUM/PLY)  │          │   │   │
│  │  │  └────────────┘  └────────────┘  └────────────┘          │   │   │
│  │  └──────────────────────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 10.2 Component Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Component Diagram                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                     Android App Components                       │    │
│  │                                                                  │    │
│  │   ┌─────────────┐     ┌─────────────┐     ┌─────────────┐      │    │
│  │   │ MainActivity│────▶│   Camera    │     │     IMU     │      │    │
│  │   │             │     │   Service   │     │   Service   │      │    │
│  │   └─────────────┘     └──────┬──────┘     └──────┬──────┘      │    │
│  │                              │                   │              │    │
│  │   ┌─────────────┐           ▼                   ▼              │    │
│  │   │  Settings   │     ┌─────────────────────────────────┐      │    │
│  │   │  Fragment   │────▶│      Timestamp Synchronizer     │      │    │
│  │   └─────────────┘     └──────────────┬──────────────────┘      │    │
│  │                                       │                         │    │
│  │   ┌─────────────┐                    ▼                         │    │
│  │   │ Calibration │     ┌─────────────────────────────────┐      │    │
│  │   │  Fragment   │────▶│         Data Manager            │      │    │
│  │   └─────────────┘     │  ┌─────────┐  ┌─────────────┐  │      │    │
│  │                       │  │ Recorder│  │  Streamer   │  │      │    │
│  │                       │  └─────────┘  └─────────────┘  │      │    │
│  │                       └─────────────────────────────────┘      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      PC Client Components                        │    │
│  │                                                                  │    │
│  │   ┌─────────────┐     ┌─────────────┐     ┌─────────────┐      │    │
│  │   │    Main     │────▶│   WebRTC    │     │     UDP     │      │    │
│  │   │  Controller │     │   Client    │     │   Receiver  │      │    │
│  │   └─────────────┘     └──────┬──────┘     └──────┬──────┘      │    │
│  │                              │                   │              │    │
│  │   ┌─────────────┐           ▼                   ▼              │    │
│  │   │   Config    │     ┌─────────────────────────────────┐      │    │
│  │   │   Manager   │────▶│         Data Synchronizer       │      │    │
│  │   └─────────────┘     └──────────────┬──────────────────┘      │    │
│  │                                       │                         │    │
│  │                                       ▼                         │    │
│  │                       ┌─────────────────────────────────┐      │    │
│  │                       │      SLAM Framework Adapter     │      │    │
│  │                       │  ┌────────┐ ┌────────┐         │      │    │
│  │                       │  │ VINS   │ │OpenVINS│  ...    │      │    │
│  │                       │  └────────┘ └────────┘         │      │    │
│  │                       └──────────────┬──────────────────┘      │    │
│  │                                       │                         │    │
│  │                                       ▼                         │    │
│  │                       ┌─────────────────────────────────┐      │    │
│  │                       │         Output Manager          │      │    │
│  │                       │  ┌────────┐ ┌────────┐ ┌─────┐ │      │    │
│  │                       │  │ROS Pub │ │Visualiz│ │Export│ │      │    │
│  │                       │  └────────┘ └────────┘ └─────┘ │      │    │
│  │                       └─────────────────────────────────┘      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 10.3 Sequence Diagram: Real-time SLAM

```
┌─────┐          ┌─────────┐    ┌──────────┐    ┌──────┐    ┌──────────┐
│User │          │   App   │    │ Network  │    │  PC  │    │ VI-SLAM  │
└──┬──┘          └────┬────┘    └────┬─────┘    └──┬───┘    └────┬─────┘
   │                  │              │             │              │
   │  Start Session   │              │             │              │
   │─────────────────▶│              │             │              │
   │                  │              │             │              │
   │                  │ Connect      │             │              │
   │                  │─────────────▶│             │              │
   │                  │              │  Accept     │              │
   │                  │              │────────────▶│              │
   │                  │  Connected   │             │              │
   │                  │◀─────────────│             │              │
   │                  │              │             │              │
   │                  │────┐ Start   │             │              │
   │                  │    │ Camera  │             │              │
   │                  │◀───┘ & IMU   │             │              │
   │                  │              │             │              │
   │        ┌─────────┴──────────────┴─────────────┴──────────────┴─────┐
   │        │                         Loop                              │
   │        │ ┌───────────────────────────────────────────────────────┐ │
   │        │ │                                                       │ │
   │        │ │  │ Frame + IMU │             │              │         │ │
   │        │ │  │────────────▶│ Stream      │              │         │ │
   │        │ │  │             │────────────▶│              │         │ │
   │        │ │  │             │             │  Sync &      │         │ │
   │        │ │  │             │             │  Process     │         │ │
   │        │ │  │             │             │─────────────▶│         │ │
   │        │ │  │             │             │              │         │ │
   │        │ │  │             │             │  Pose        │         │ │
   │        │ │  │             │             │◀─────────────│         │ │
   │        │ │  │             │             │              │         │ │
   │        │ │  │             │   Pose      │              │         │ │
   │        │ │  │◀────────────│◀────────────│              │         │ │
   │        │ │  │             │             │              │         │ │
   │        │ └───────────────────────────────────────────────────────┘ │
   │        │                      [30 Hz]                              │
   │        └───────────────────────────────────────────────────────────┘
   │                  │              │             │              │
   │  Stop Session    │              │             │              │
   │─────────────────▶│              │             │              │
   │                  │ Disconnect   │             │              │
   │                  │─────────────▶│             │              │
   │                  │              │             │              │
   │  Session Stats   │              │             │              │
   │◀─────────────────│              │             │              │
   │                  │              │             │              │
```

---

## 11. Constraints and Assumptions

### 11.1 Technical Constraints

| ID | Constraint | Impact | Mitigation |
|----|------------|--------|------------|
| C1 | Camera2 API support required | Older devices not supported | Restrict to minimum Android 8.0 |
| C2 | Network latency exists | Real-time limitation | Buffering and prediction algorithms |
| C3 | IMU sampling limitation (device-dependent) | Some devices below 200Hz | Handle with minimum 100Hz, interpolation |
| C4 | Battery consumption | Limited long-term use | Power-saving mode, warning display |
| C5 | Rolling Shutter | Distortion during fast movement | Use VINS-Mono RS model |

### 11.2 Business Constraints

| ID | Constraint | Impact | Mitigation |
|----|------------|--------|------------|
| C6 | Open source license | Commercialization limitations (GPLv3) | Provide Basalt (BSD) option |
| C7 | Single-developer resources | Development speed limitation | Prioritize core features |
| C8 | Test device limitations | Compatibility verification limitations | Community collaboration |

### 11.3 Assumptions

| ID | Assumption | Risk | Verification Method |
|----|------------|------|---------------------|
| A1 | 5GHz WiFi environment available | Network performance degradation | Test in various environments |
| A2 | User has basic development knowledge | Usability issues | Detailed documentation |
| A3 | Primarily used in static environments | Dynamic environment performance degradation | Specify use scenarios |
| A4 | Sufficient lighting environment | Low-light performance degradation | Specify recommended environment |

---

## 12. Roadmap and Milestones

### 12.1 Overall Roadmap

```
2026
Q1                    Q2                    Q3                    Q4
├─────────────────────┼─────────────────────┼─────────────────────┼─────────
│                     │                     │                     │
│ ┌─────────────────┐ │ ┌─────────────────┐ │ ┌─────────────────┐ │
│ │    Phase 1      │ │ │    Phase 2      │ │ │    Phase 3      │ │
│ │    MVP          │ │ │   Real-time     │ │ │   Expansion     │ │
│ │                 │ │ │                 │ │ │                 │ │
│ │ • Android app   │ │ │ • Real-time     │ │ │ • iOS app       │ │
│ │ • PC client     │ │ │   streaming     │ │ │ • Auto calib    │ │
│ │ • VINS-Mono     │ │ │ • Multi-        │ │ │ • Cloud         │ │
│ │ • Offline       │ │ │   framework     │ │ │   integration   │ │
│ │   processing    │ │ │ • Visualization │ │ │ • SDK/plugins   │ │
│ │                 │ │ │ • ROS integration│ │ │                 │ │
│ └─────────────────┘ │ └─────────────────┘ │ └─────────────────┘ │
│                     │                     │                     │
│ M1 ─────────────────│ M2 ─────────────────│ M3 ────────────────│ M4
│ MVP Release         │ v1.0 Release        │ v1.5 Release       │ v2.0 Release
│                     │                     │                     │
```

### 12.2 Phase 1: MVP (Q1 2026)

**Goal**: Basic data collection and offline SLAM processing

| Milestone | Period | Deliverables |
|-----------|--------|--------------|
| M1.1 | Week 1-2 | Android app prototype (camera/IMU capture) |
| M1.2 | Week 3-4 | Local storage feature, data format definition |
| M1.3 | Week 5-6 | PC data loader, VINS-Mono integration |
| M1.4 | Week 7-8 | Offline processing pipeline, result visualization |
| M1.5 | Week 9-10 | Testing, documentation, MVP release |

**Key Features**:
- FR1.1-1.4: Camera/IMU capture, local storage
- FR3.4: VINS-Mono integration
- FR6.1, FR6.4: EuRoC/CSV format support
- FR9.1: Trajectory export

### 12.3 Phase 2: Real-time (Q2 2026)

**Goal**: Real-time streaming and multi-framework support

| Milestone | Period | Deliverables |
|-----------|--------|--------------|
| M2.1 | Week 1-3 | WebRTC video streaming |
| M2.2 | Week 4-5 | UDP IMU streaming, synchronization |
| M2.3 | Week 6-7 | OpenVINS, ORB-SLAM3 integration |
| M2.4 | Week 8-9 | ROS integration, real-time visualization |
| M2.5 | Week 10-12 | Performance optimization, v1.0 release |

**Key Features**:
- FR1.5, FR4.1-4.4: Real-time streaming
- FR3.4 extension: OpenVINS, ORB-SLAM3
- FR3.7: ROS topic publishing
- FR3.5: Real-time visualization

### 12.4 Phase 3: Expansion (Q3-Q4 2026)

**Goal**: Platform expansion and advanced features

| Milestone | Period | Deliverables |
|-----------|--------|--------------|
| M3.1 | Q3 Week 1-6 | iOS app development |
| M3.2 | Q3 Week 7-12 | Auto calibration tool |
| M3.3 | Q4 Week 1-6 | Unity/Unreal SDK |
| M3.4 | Q4 Week 7-12 | Cloud processing option, v2.0 release |

**Key Features**:
- FR2.x: Complete iOS app
- FR8.5: Auto calibration
- New requirements: SDK, cloud

### 12.5 Release Plan

| Version | Date | Key Content |
|---------|------|-------------|
| v0.1.0-alpha | 2026-02 | Internal testing MVP |
| v0.5.0-beta | 2026-03 | Public beta (Android + offline) |
| v1.0.0 | 2026-06 | First official release (real-time + multi-framework) |
| v1.5.0 | 2026-09 | iOS added, auto calibration |
| v2.0.0 | 2026-12 | SDK, cloud integration |

---

## 13. Risk Analysis

### 13.1 Risk Matrix

```
         Impact
    High │  R3      R1
         │
    Med  │  R5   R2   R4
         │
    Low  │  R6
         └──────────────────
           Low    Med   High
                Likelihood
```

### 13.2 Risk Details

| ID | Risk | Likelihood | Impact | Mitigation Strategy |
|----|------|------------|--------|---------------------|
| **R1** | Smartphone device compatibility issues | High | High | Manage compatible device list, community feedback |
| **R2** | Real-time requirements not met due to network latency | Medium | Medium | Buffering optimization, USB tethering alternative |
| **R3** | SLAM framework update compatibility | Medium | High | Version pinning, adapter pattern |
| **R4** | Insufficient sensor calibration accuracy | High | Medium | Develop auto calibration, strengthen guides |
| **R5** | Development schedule delays | Medium | Medium | Prioritize core features, scope adjustment |
| **R6** | Competing product emergence | Low | Low | Strengthen differentiating features, build community |

### 13.3 Risk Mitigation Plans

#### R1: Device Compatibility (High/High)

```
Mitigation Strategy:
1. Manage and publish official supported device list
2. Implement Camera2 API level check logic
3. Community-based compatibility reporting system
4. Clear warning messages for incompatible devices

Response Plan:
- On occurrence: Analyze device → Workaround or unsupported list
- Budget: Secure test device purchase budget
```

#### R4: Calibration Accuracy (High/Medium)

```
Mitigation Strategy:
1. Provide step-by-step calibration guide
2. Real-time display of calibration quality metrics
3. Provide pre-configured device-specific defaults
4. Implement auto calibration in Phase 2

Response Plan:
- On occurrence: Provide troubleshooting guide
- Long-term: Strengthen online calibration (VINS estimate_td)
```

---

## 14. Appendix

### 14.1 Glossary

| Term | Definition |
|------|------------|
| **VI-SLAM** | Visual-Inertial SLAM. SLAM combining camera and IMU |
| **IMU** | Inertial Measurement Unit. Accelerometer + Gyroscope |
| **6DoF** | 6 Degrees of Freedom. Position (x,y,z) + Orientation (roll,pitch,yaw) |
| **ATE** | Absolute Trajectory Error |
| **RMSE** | Root Mean Square Error |
| **Extrinsic** | Relative position/orientation transformation between sensors |
| **Intrinsic** | Internal sensor parameters (focal length, distortion coefficients, etc.) |
| **Rolling Shutter** | Row-by-row sequential exposure method |
| **Loop Closing** | Error correction through recognition of previously visited locations |
| **Tightly Coupled** | Processing Visual and Inertial in a single optimization |

### 14.2 Reference Documents

| Document | Location |
|----------|----------|
| VI-SLAM Overview | `docs/reference/01_vi_slam_overview.md` |
| Android Camera2 API | `docs/reference/06_android_camera2_api.md` |
| Streaming Protocols | `docs/reference/03_streaming_protocols.md` |
| App Development Guide | `docs/reference/11_app_development_guide.md` |
| Framework Comparison | `docs/reference/05_vislam_frameworks.md` |
| Comprehensive Analysis | `docs/comprehensive_analysis.md` |

### 14.3 Change History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2026-01-20 | Initial creation | VI-SLAM Team |

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Product Owner | | | |
| Tech Lead | | | |
| QA Lead | | | |

---

*This document defines the product requirements for the VI-SLAM project.*
*Document created: 2026-01-20*
*Version: 1.0.0*
