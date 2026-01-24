# Software Requirements Specification (SRS)
# VI-SLAM: Smartphone Visual-Inertial SLAM System

---

| Field | Value |
|-------|-------|
| Document ID | SRS-VISLAM-001 |
| Source PRD | PRD-VISLAM-001 (PRD.md) |
| Version | 1.0.0 |
| Status | Draft |
| Created Date | 2026-01-24 |
| Author | VI-SLAM Project Team |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Overall Description](#2-overall-description)
3. [System Features](#3-system-features)
4. [External Interface Requirements](#4-external-interface-requirements)
5. [Non-Functional Requirements](#5-non-functional-requirements)
6. [Data Requirements](#6-data-requirements)
7. [Traceability Matrix](#7-traceability-matrix)
8. [Appendix](#8-appendix)

---

## 1. Introduction

### 1.1 Purpose

This Software Requirements Specification (SRS) document provides a comprehensive description of the VI-SLAM system requirements. It is intended to serve as the primary reference for the development team, testers, and stakeholders to understand the detailed functional and non-functional requirements of the system.

This document decomposes the high-level requirements from the Product Requirements Document (PRD) into detailed, testable specifications with complete use cases, interface definitions, and traceability.

### 1.2 Scope

The VI-SLAM system encompasses:

- **Android Application**: Captures camera frames and IMU data from smartphone sensors
- **iOS Application** (Phase 2): Similar functionality for Apple devices
- **PC Client**: Receives, synchronizes, and processes data using VI-SLAM frameworks
- **Communication Layer**: Real-time streaming via WebRTC and UDP protocols
- **VI-SLAM Engine**: Integration with multiple SLAM frameworks (VINS-Mono, OpenVINS, ORB-SLAM3, Basalt)
- **Output System**: Trajectory export, point cloud generation, and real-time pose APIs

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| VI-SLAM | Visual-Inertial Simultaneous Localization and Mapping |
| IMU | Inertial Measurement Unit (accelerometer + gyroscope) |
| 6DoF | Six Degrees of Freedom (position + orientation) |
| ATE | Absolute Trajectory Error |
| RMSE | Root Mean Square Error |
| FPS | Frames Per Second |
| WebRTC | Web Real-Time Communication protocol |
| ROS | Robot Operating System |
| EuRoC | European Robotics Challenge (benchmark dataset format) |
| TUM | Technical University of Munich (trajectory format) |
| Extrinsic | Relative position/orientation between sensors |
| Intrinsic | Internal camera parameters (focal length, distortion) |
| Rolling Shutter | Sequential row-by-row image capture method |
| Loop Closing | Error correction via revisited location recognition |

### 1.4 References

| Document | Description |
|----------|-------------|
| PRD.md | Product Requirements Document |
| 01_vi_slam_overview.md | VI-SLAM technical overview |
| 03_streaming_protocols.md | Streaming protocol analysis |
| 05_vislam_frameworks.md | Framework comparison |
| 06_android_camera2_api.md | Android Camera2 API reference |
| 09_timestamp_synchronization.md | Timestamp synchronization techniques |
| 19_calibration_practical_guide.md | Calibration procedures |

---

## 2. Overall Description

### 2.1 Product Perspective

VI-SLAM is a standalone system that transforms a standard smartphone into a professional-grade visual-inertial sensor platform. The system operates in a client-server architecture where:

- The smartphone acts as a sensor hub and data transmitter
- The PC serves as the processing server running VI-SLAM algorithms
- Network connectivity (WiFi/USB tethering) bridges the two components

```
+------------------+       Network        +------------------+
|   Smartphone     |   (WebRTC/UDP)       |      PC          |
|   - Camera       | ===================> |   - VI-SLAM      |
|   - IMU          |                      |   - Visualizer   |
|   - Streamer     |                      |   - ROS Bridge   |
+------------------+                      +------------------+
```

### 2.2 Product Functions Summary

| Function Category | Key Capabilities |
|-------------------|------------------|
| **Data Acquisition** | Camera capture (30Hz), IMU capture (200Hz), synchronized timestamps |
| **Data Storage** | Local recording, EuRoC/TUM format export, metadata generation |
| **Real-time Streaming** | WebRTC video, UDP IMU, low-latency transmission (<100ms) |
| **SLAM Processing** | Multi-framework support, 6DoF pose estimation, map generation |
| **Calibration** | Camera intrinsic, camera-IMU extrinsic, time offset estimation |
| **Output** | Trajectory export, point cloud, ROS topics, real-time API |

### 2.3 User Classes and Characteristics

| User Class | Characteristics | Primary Use Cases |
|------------|-----------------|-------------------|
| **AR/VR Developer** | Unity/Unreal experience, basic SLAM knowledge | Real-time pose for AR applications |
| **Robotics Researcher** | ROS expertise, algorithm development | Data collection, algorithm testing |
| **Mapping Engineer** | 3D modeling background, data processing | Offline SLAM, map generation |
| **Indoor Positioning Developer** | Location services experience | Pose estimation integration |

### 2.4 Operating Environment

#### 2.4.1 Smartphone Environment

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| Android Version | 8.0 (API 26) | 11.0+ (API 30+) |
| Camera2 API Level | HARDWARE_LEVEL_LIMITED | HARDWARE_LEVEL_FULL |
| RAM | 2 GB | 4 GB+ |
| Storage | 1 GB free | 10 GB+ free |
| Sensors | Accelerometer, Gyroscope | + Magnetometer, Barometer |

#### 2.4.2 PC Environment

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| Operating System | Ubuntu 18.04 | Ubuntu 20.04/22.04 |
| CPU | Intel i5 / AMD Ryzen 5 | Intel i7 / AMD Ryzen 7 |
| RAM | 8 GB | 16 GB+ |
| GPU | Integrated | NVIDIA GTX 1060+ |
| Storage | 50 GB | 200 GB+ SSD |

#### 2.4.3 Network Environment

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| Connection Type | 2.4 GHz WiFi | 5 GHz WiFi / USB Tethering |
| Bandwidth | 5 Mbps | 20 Mbps+ |
| Latency | < 50 ms | < 20 ms |

### 2.5 Design and Implementation Constraints

| Constraint ID | Description | Impact |
|---------------|-------------|--------|
| DC-001 | Camera2 API HARDWARE_LEVEL_FULL required for full features | Limits compatible devices |
| DC-002 | IMU sampling rate device-dependent (some < 200Hz) | May require interpolation |
| DC-003 | Rolling shutter cameras require compensation | Affects fast motion accuracy |
| DC-004 | Network latency affects real-time performance | Requires buffering/prediction |
| DC-005 | Open-source licenses (GPLv3 for some frameworks) | Commercial use restrictions |
| DC-006 | Battery consumption limits operation duration | Power management required |

---

## 3. System Features

### SF-001: Camera Frame Capture

**Source**: FR1.1 (PRD)
**Priority**: P0
**Description**: Capture frames from smartphone rear camera using Camera2 API with accurate sensor timestamps.

#### 3.1.1 Use Cases

##### UC-001: Standard Camera Capture

- **Actor**: Android Application
- **Preconditions**:
  1. Device supports Camera2 API HARDWARE_LEVEL_LIMITED or higher
  2. Camera permission granted
  3. Camera not in use by another application
- **Main Flow**:
  1. Application initializes Camera2 session with specified parameters
  2. System opens rear camera device
  3. System configures ImageReader for YUV420 or JPEG format
  4. System creates capture session with target surfaces
  5. System starts repeating capture request
  6. For each frame:
     a. System captures image buffer
     b. System extracts SENSOR_TIMESTAMP
     c. System packages frame with timestamp
     d. System passes to Data Manager
  7. System maintains capture at target FPS
- **Alternative Flows**:
  - 2a. Camera device unavailable:
    - System displays error message
    - System attempts retry after 1 second (max 3 retries)
    - System notifies user of failure
  - 6a. Frame drop detected:
    - System logs frame drop event
    - System continues capture without interruption
    - System updates frame drop statistics
- **Exception Flows**:
  - E1. Camera disconnected during capture:
    - System releases camera resources
    - System notifies user
    - System offers reconnection option
  - E2. Memory pressure detected:
    - System reduces buffer size
    - System may reduce resolution temporarily
- **Postconditions**:
  1. Frames captured at specified FPS (within 5% tolerance)
  2. Each frame has nanosecond-precision timestamp
  3. Frame metadata (exposure, ISO) available

#### 3.1.2 Acceptance Criteria

- [ ] AC-001.1: Frame capture rate maintains 30 FPS with < 1% drop rate
- [ ] AC-001.2: Timestamp accuracy within 1ms of actual capture time
- [ ] AC-001.3: Supports 720p and 1080p resolutions
- [ ] AC-001.4: No memory leaks after 1-hour continuous operation
- [ ] AC-001.5: Camera initialization completes within 2 seconds
- [ ] AC-001.6: Graceful handling of camera interrupts (phone call, etc.)

#### 3.1.3 Dependencies

- Depends on: None (base feature)
- Blocks: SF-003 (Timestamp Synchronization), SF-005 (Real-time Streaming)

---

### SF-002: IMU Data Capture

**Source**: FR1.2 (PRD)
**Priority**: P0
**Description**: Capture accelerometer and gyroscope data at high frequency using Android SensorManager.

#### 3.2.1 Use Cases

##### UC-002: High-Frequency IMU Sampling

- **Actor**: Android Application
- **Preconditions**:
  1. Device has accelerometer and gyroscope sensors
  2. Sensor permission granted (if required by Android version)
- **Main Flow**:
  1. Application registers sensor listeners with SENSOR_DELAY_FASTEST
  2. System queries available sensor types (calibrated/uncalibrated)
  3. System initializes circular buffer for IMU data
  4. For each sensor event:
     a. System extracts acceleration (ax, ay, az) in m/s^2
     b. System extracts angular velocity (gx, gy, gz) in rad/s
     c. System records sensor timestamp (nanoseconds)
     d. System adds sample to circular buffer
     e. System notifies Data Manager of new data
  5. System maintains sampling at target frequency
- **Alternative Flows**:
  - 1a. SENSOR_DELAY_FASTEST not achieving target rate:
    - System logs actual achieved rate
    - System continues with best available rate
    - System warns user if rate < 100Hz
  - 4a. Buffer overflow imminent:
    - System expands buffer if memory available
    - System drops oldest samples if necessary
    - System logs overflow event
- **Exception Flows**:
  - E1. Sensor disconnected:
    - System attempts reconnection
    - System notifies user after 3 failed attempts
  - E2. Sensor returning invalid data:
    - System validates data range
    - System discards invalid samples
    - System logs anomaly
- **Postconditions**:
  1. IMU samples available at >= 95% of target rate
  2. Each sample has nanosecond-precision timestamp
  3. Timestamps are monotonically increasing

#### 3.2.2 Acceptance Criteria

- [ ] AC-002.1: Achieves >= 200Hz sampling rate on compatible devices
- [ ] AC-002.2: Sample loss rate < 0.1%
- [ ] AC-002.3: Timestamp monotonicity guaranteed
- [ ] AC-002.4: Supports both calibrated and uncalibrated sensor types
- [ ] AC-002.5: Circular buffer handles 5 seconds of data at 500Hz
- [ ] AC-002.6: CPU usage for IMU capture < 5%

#### 3.2.3 Dependencies

- Depends on: None (base feature)
- Blocks: SF-003 (Timestamp Synchronization), SF-005 (Real-time Streaming)

---

### SF-003: Timestamp Synchronization

**Source**: FR1.3, FR5.1, FR5.2 (PRD)
**Priority**: P0
**Description**: Integrate camera and IMU timestamps into unified timeline and provide IMU interpolation for frame timestamps.

#### 3.3.1 Use Cases

##### UC-003: Camera-IMU Time Alignment

- **Actor**: Data Manager
- **Preconditions**:
  1. Camera capture active (SF-001)
  2. IMU capture active (SF-002)
  3. System clock stable
- **Main Flow**:
  1. Data Manager receives camera frame with timestamp T_cam
  2. Data Manager queries IMU buffer for samples around T_cam
  3. System identifies IMU samples: T_imu_before <= T_cam <= T_imu_after
  4. System performs linear interpolation for IMU at T_cam:
     - ratio = (T_cam - T_imu_before) / (T_imu_after - T_imu_before)
     - acc_interp = acc_before + ratio * (acc_after - acc_before)
     - gyro_interp = gyro_before + ratio * (gyro_after - gyro_before)
  5. System associates interpolated IMU with frame
  6. System also provides IMU samples between consecutive frames
  7. System packages synchronized data for downstream processing
- **Alternative Flows**:
  - 3a. No IMU samples available for timestamp:
    - System marks frame as IMU-unavailable
    - System logs synchronization gap
  - 4a. Time gap too large (> 50ms):
    - System flags potential synchronization issue
    - System uses nearest-neighbor instead of interpolation
- **Exception Flows**:
  - E1. Clock discontinuity detected:
    - System recalibrates time reference
    - System logs discontinuity event
- **Postconditions**:
  1. Each frame has associated IMU data
  2. Synchronization accuracy within 1ms

#### 3.3.2 Acceptance Criteria

- [ ] AC-003.1: Timestamp alignment accuracy < 1ms
- [ ] AC-003.2: IMU interpolation computationally efficient (< 1ms per frame)
- [ ] AC-003.3: Handles variable IMU sampling rates gracefully
- [ ] AC-003.4: Detects and reports clock synchronization issues
- [ ] AC-003.5: Maintains synchronization during 30-minute sessions

#### 3.3.3 Dependencies

- Depends on: SF-001 (Camera Capture), SF-002 (IMU Capture)
- Blocks: SF-007 (Data Storage), SF-005 (Real-time Streaming)

---

### SF-004: Local Data Storage

**Source**: FR1.4 (PRD)
**Priority**: P0
**Description**: Store video and IMU data locally on smartphone with proper format and metadata.

#### 3.4.1 Use Cases

##### UC-004: Recording Session

- **Actor**: User, Data Manager
- **Preconditions**:
  1. Sufficient storage space available (> 500MB free)
  2. Camera and IMU capture initialized
  3. Recording directory accessible
- **Main Flow**:
  1. User taps "Record" button
  2. System creates timestamped session directory
  3. System initializes video encoder (H.264)
  4. System creates IMU CSV file with headers
  5. System starts recording:
     a. Video frames encoded and written to MP4
     b. IMU samples written to CSV in real-time
     c. System displays recording statistics (frames, duration)
  6. User taps "Stop" button
  7. System finalizes video file
  8. System closes IMU file
  9. System generates metadata.json
  10. System verifies data integrity (checksums)
  11. System displays session summary
- **Alternative Flows**:
  - 5a. Storage space running low:
    - System warns user at 200MB remaining
    - System auto-stops at 50MB remaining
  - 7a. Video encoding error:
    - System saves raw frames as fallback
    - System logs encoding failure
- **Exception Flows**:
  - E1. App crash during recording:
    - On restart, system recovers partial data
    - System marks session as incomplete
  - E2. Storage write failure:
    - System attempts alternative storage location
    - System notifies user of failure
- **Postconditions**:
  1. Complete session data saved to storage
  2. All files pass integrity verification
  3. Metadata accurately describes session

#### 3.4.2 Acceptance Criteria

- [ ] AC-004.1: Video saved in H.264 MP4 format
- [ ] AC-004.2: IMU saved in CSV with nanosecond timestamps
- [ ] AC-004.3: Metadata JSON includes device info, calibration, statistics
- [ ] AC-004.4: Frame drop rate < 1% during recording
- [ ] AC-004.5: Supports sessions up to 60 minutes
- [ ] AC-004.6: Data recovery possible after unexpected termination

#### 3.4.3 Dependencies

- Depends on: SF-001, SF-002, SF-003
- Blocks: SF-009 (Offline SLAM Processing)

---

### SF-005: Real-time Data Streaming

**Source**: FR1.5, FR4.1, FR4.2, FR4.3, FR4.4 (PRD)
**Priority**: P1
**Description**: Stream camera and IMU data to PC client in real-time via WebRTC (video) and UDP (IMU).

#### 3.5.1 Use Cases

##### UC-005: Real-time Streaming Session

- **Actor**: User, Android App, PC Client
- **Preconditions**:
  1. PC client running and listening
  2. Network connectivity established
  3. Camera and IMU capture ready
- **Main Flow**:
  1. User enters PC IP address in app settings
  2. User taps "Connect" button
  3. App initiates WebRTC signaling with PC
  4. PC accepts connection
  5. WebRTC peer connection established
  6. App establishes UDP socket for IMU data
  7. Connection status shown as "Connected"
  8. User taps "Start Streaming" button
  9. For each captured frame:
     a. App encodes frame via WebRTC video track
     b. App sends frame over WebRTC data channel
  10. For each IMU sample:
      a. App packages IMU with timestamp
      b. App sends via UDP socket
  11. PC receives and processes data
  12. User taps "Stop Streaming" button
  13. App terminates streaming session
  14. WebRTC connection gracefully closed
- **Alternative Flows**:
  - 4a. Connection timeout:
    - App displays "Connection Failed"
    - App suggests checking PC IP/firewall
  - 9a. Network congestion detected:
    - App enables adaptive bitrate
    - App may reduce resolution temporarily
  - 10a. UDP packet loss detected:
    - System logs packet loss statistics
    - PC uses interpolation for missing samples
- **Exception Flows**:
  - E1. Network disconnection:
    - App buffers data locally
    - App attempts reconnection (max 5 attempts)
    - App notifies user of disconnection
  - E2. PC client crash:
    - App detects connection loss
    - App offers to switch to local recording
- **Postconditions**:
  1. Data successfully transmitted to PC
  2. End-to-end latency < 100ms
  3. Session statistics recorded

#### 3.5.2 Acceptance Criteria

- [ ] AC-005.1: WebRTC video stream at 30 FPS
- [ ] AC-005.2: UDP IMU transmission at 200Hz
- [ ] AC-005.3: End-to-end latency < 100ms (95th percentile)
- [ ] AC-005.4: Automatic reconnection on network interruption
- [ ] AC-005.5: Adaptive bitrate responds within 2 seconds
- [ ] AC-005.6: Works over 5GHz WiFi and USB tethering

#### 3.5.3 Dependencies

- Depends on: SF-001, SF-002, SF-003
- Blocks: SF-010 (Real-time SLAM Processing)

---

### SF-006: PC Data Reception

**Source**: FR3.1, FR3.2, FR3.3 (PRD)
**Priority**: P0
**Description**: Receive and decode video/IMU data from smartphone on PC client.

#### 3.6.1 Use Cases

##### UC-006: Data Reception and Decoding

- **Actor**: PC Client
- **Preconditions**:
  1. PC client application running
  2. Network interface configured
  3. Video decoder initialized
- **Main Flow**:
  1. PC client starts WebRTC listener on configured port
  2. PC client starts UDP listener for IMU
  3. Client waits for smartphone connection
  4. On connection established:
     a. Client allocates receive buffers
     b. Client initializes video decoder (H.264/H.265)
  5. For each received video frame:
     a. Client decodes compressed frame
     b. Client extracts timestamp from metadata
     c. Client converts to cv::Mat format
     d. Client adds to synchronized queue
  6. For each received IMU packet:
     a. Client parses binary IMU data
     b. Client validates timestamp
     c. Client adds to IMU buffer
  7. Client runs synchronization algorithm
  8. Client feeds synchronized data to SLAM engine
- **Alternative Flows**:
  - 5a. Decode error:
    - Client requests keyframe
    - Client logs decode failure
  - 6a. Out-of-order IMU packets:
    - Client reorders based on timestamp
    - Client logs reordering event
- **Exception Flows**:
  - E1. Buffer overflow:
    - Client drops oldest data
    - Client warns of processing lag
- **Postconditions**:
  1. Video frames decoded and available
  2. IMU data buffered and synchronized
  3. Data ready for SLAM processing

#### 3.6.2 Acceptance Criteria

- [ ] AC-006.1: H.264 decoding at 30 FPS
- [ ] AC-006.2: H.265 decoding support (optional)
- [ ] AC-006.3: IMU parsing at 500Hz capacity
- [ ] AC-006.4: Synchronization accuracy < 1ms
- [ ] AC-006.5: Buffer handles 2-second network jitter

#### 3.6.3 Dependencies

- Depends on: SF-005 (Streaming)
- Blocks: SF-010 (Real-time SLAM)

---

### SF-007: VI-SLAM Framework Integration

**Source**: FR3.4, FR7.1, FR7.2, FR7.3, FR7.4, FR7.5 (PRD)
**Priority**: P0
**Description**: Integrate multiple VI-SLAM frameworks with unified interface.

#### 3.7.1 Use Cases

##### UC-007: VINS-Mono Integration

- **Actor**: PC Client, SLAM Engine
- **Preconditions**:
  1. VINS-Mono installed and configured
  2. Calibration file available
  3. Data source ready (streaming or file)
- **Main Flow**:
  1. User selects VINS-Mono as SLAM framework
  2. System loads VINS-Mono configuration
  3. System initializes VINS-Mono estimator
  4. System enters initialization phase:
     a. Collects initial frames and IMU
     b. Performs SfM initialization
     c. Aligns visual and inertial reference
  5. On successful initialization:
     a. System transitions to tracking mode
     b. Status changed to "TRACKING"
  6. For each synchronized frame+IMU:
     a. System feeds image to front-end
     b. System feeds IMU to pre-integration
     c. System runs optimization
     d. System outputs 6DoF pose
  7. System maintains map of 3D points
  8. On tracking loss:
     a. System attempts relocalization
     b. If failed, system reinitializes
- **Alternative Flows**:
  - 4a. Initialization fails (insufficient motion):
    - System provides guidance message
    - System resets and retries
  - 6a. Poor feature quality:
    - System warns of degraded tracking
    - System may trigger relocalization
- **Exception Flows**:
  - E1. VINS-Mono crash:
    - System captures error state
    - System offers restart option
- **Postconditions**:
  1. Continuous 6DoF pose output at 30Hz
  2. Map updated with new observations
  3. Trajectory logged

##### UC-008: Multi-Framework Selection

- **Actor**: User, PC Client
- **Preconditions**:
  1. At least one framework installed
  2. Framework configuration files available
- **Main Flow**:
  1. User opens framework selection dialog
  2. System displays available frameworks:
     - VINS-Mono (installed/not installed)
     - OpenVINS (installed/not installed)
     - ORB-SLAM3 (installed/not installed)
     - Basalt (installed/not installed)
  3. User selects desired framework
  4. System validates framework installation
  5. System loads framework-specific adapter
  6. System applies calibration conversion if needed
  7. System confirms framework ready
- **Alternative Flows**:
  - 4a. Framework not installed:
    - System provides installation instructions
    - System offers alternative frameworks
- **Postconditions**:
  1. Selected framework initialized
  2. Unified interface ready for use

#### 3.7.2 Acceptance Criteria

- [ ] AC-007.1: VINS-Mono integration complete (P0)
- [ ] AC-007.2: OpenVINS integration complete (P1)
- [ ] AC-007.3: ORB-SLAM3 integration complete (P1)
- [ ] AC-007.4: Basalt integration available (P2)
- [ ] AC-007.5: Framework switch without restart
- [ ] AC-007.6: Position accuracy ATE RMSE < 0.15m on EuRoC
- [ ] AC-007.7: Real-time processing at 30Hz input/output

#### 3.7.3 Dependencies

- Depends on: SF-006 (Data Reception)
- Blocks: SF-011 (Output and Export)

---

### SF-008: Sensor Calibration

**Source**: FR8.1, FR8.2, FR8.3, FR8.4, FR8.5 (PRD)
**Priority**: P0 (Intrinsic), P1 (Extrinsic), P2 (Auto)
**Description**: Calibrate camera intrinsics, camera-IMU extrinsics, and time offset.

#### 3.8.1 Use Cases

##### UC-009: Camera Intrinsic Calibration

- **Actor**: User, Calibration Module
- **Preconditions**:
  1. Calibration target (checkerboard) available
  2. Camera capture initialized
  3. Good lighting conditions
- **Main Flow**:
  1. User selects "Camera Calibration" mode
  2. System displays calibration guide
  3. System shows live camera feed with detection overlay
  4. User captures checkerboard from various angles:
     a. System detects corner points
     b. System provides coverage feedback (25 regions)
     c. System indicates "good" pose for capture
     d. User triggers capture (auto or manual)
  5. After minimum 20 captures:
     a. System enables "Calibrate" button
  6. User taps "Calibrate"
  7. System runs Zhang's method calibration
  8. System computes:
     - Focal length (fx, fy)
     - Principal point (cx, cy)
     - Distortion coefficients (k1, k2, p1, p2)
  9. System displays reprojection error
  10. If error < 0.5 pixel:
      a. System saves calibration
      b. System shows success message
  11. If error >= 0.5 pixel:
      a. System suggests recapturing
      b. System highlights problematic images
- **Alternative Flows**:
  - 4a. Corner detection fails:
    - System provides feedback on target visibility
    - System suggests lighting/angle adjustment
  - 9a. High reprojection error:
    - System allows removing outlier images
    - System recalculates calibration
- **Postconditions**:
  1. Camera intrinsic parameters saved
  2. Reprojection error < 0.5 pixel

##### UC-010: Camera-IMU Extrinsic Calibration

- **Actor**: User, Calibration Module, Kalibr Tool
- **Preconditions**:
  1. Camera intrinsics calibrated
  2. AprilGrid target available
  3. Kalibr installed on PC
- **Main Flow**:
  1. User selects "Camera-IMU Calibration" mode
  2. System displays motion guide
  3. User moves device in various directions for 60 seconds:
     - Roll, pitch, yaw rotations
     - Translational movements
  4. System records synchronized camera + IMU data
  5. System exports data in Kalibr format
  6. User runs Kalibr on PC:
     a. Kalibr processes calibration data
     b. Kalibr estimates extrinsic transformation
     c. Kalibr estimates time offset
  7. System imports Kalibr results
  8. System saves calibration.yaml
- **Alternative Flows**:
  - 3a. Insufficient motion variety:
    - System provides specific motion guidance
    - System extends recording duration
- **Postconditions**:
  1. Camera-IMU extrinsic (T_cam_imu) saved
  2. Time offset (td) estimated
  3. Calibration accuracy verified

#### 3.8.2 Acceptance Criteria

- [ ] AC-008.1: Camera intrinsic reprojection error < 0.5 pixel
- [ ] AC-008.2: Extrinsic calibration compatible with Kalibr
- [ ] AC-008.3: Time offset accuracy < 1ms
- [ ] AC-008.4: Calibration process completes within 10 minutes
- [ ] AC-008.5: Calibration results exportable in YAML/JSON format
- [ ] AC-008.6: Supports pinhole and fisheye camera models

#### 3.8.3 Dependencies

- Depends on: SF-001, SF-002, SF-004
- Blocks: SF-007 (SLAM Integration)

---

### SF-009: Offline SLAM Processing

**Source**: UC3 (PRD)
**Priority**: P1
**Description**: Process recorded datasets through VI-SLAM frameworks offline.

#### 3.9.1 Use Cases

##### UC-011: Offline Dataset Processing

- **Actor**: User, PC Client
- **Preconditions**:
  1. Recorded dataset available
  2. VI-SLAM framework installed
  3. Calibration file available
- **Main Flow**:
  1. User selects dataset directory
  2. System validates dataset format:
     - Checks for video.mp4 / images/
     - Checks for imu.csv
     - Checks for metadata.json
  3. System displays dataset statistics
  4. User selects SLAM framework
  5. User configures processing options:
     - Output formats (trajectory, point cloud)
     - Visualization (enable/disable)
  6. User clicks "Start Processing"
  7. System converts data to framework format if needed
  8. System feeds data to SLAM engine:
     - Progress bar shows completion percentage
     - Real-time visualization (optional)
  9. SLAM engine processes all data
  10. System saves results:
      - trajectory.txt (TUM format)
      - pointcloud.ply
      - keyframes/ (optional)
  11. System generates processing report
- **Alternative Flows**:
  - 2a. Invalid dataset format:
    - System displays specific errors
    - System suggests conversion tools
  - 8a. Processing error:
    - System saves partial results
    - System logs error details
- **Postconditions**:
  1. Trajectory file generated
  2. Point cloud saved (if enabled)
  3. Processing statistics logged

#### 3.9.2 Acceptance Criteria

- [ ] AC-009.1: Supports EuRoC dataset format
- [ ] AC-009.2: Supports custom recorded datasets
- [ ] AC-009.3: Trajectory output in TUM format
- [ ] AC-009.4: Point cloud output in PLY format
- [ ] AC-009.5: Progress indication during processing
- [ ] AC-009.6: Processing report includes ATE metrics

#### 3.9.3 Dependencies

- Depends on: SF-004, SF-007, SF-008
- Blocks: SF-011 (Output and Export)

---

### SF-010: Real-time SLAM Processing

**Source**: UC1 (PRD)
**Priority**: P1
**Description**: Perform VI-SLAM in real-time with streaming data.

#### 3.10.1 Use Cases

##### UC-012: Real-time Pose Estimation

- **Actor**: User, Android App, PC Client, External Application
- **Preconditions**:
  1. Streaming connection established
  2. SLAM framework initialized
  3. Calibration loaded
- **Main Flow**:
  1. User starts streaming from app
  2. PC receives synchronized data
  3. SLAM engine initializes:
     - Status: INITIALIZING
     - User moves device to provide motion
  4. On successful initialization:
     - Status: TRACKING
     - System begins outputting poses
  5. For each frame at 30Hz:
     a. System processes frame + IMU
     b. System estimates 6DoF pose
     c. System publishes pose via API/ROS
     d. System updates visualization
  6. External application receives pose data
  7. User stops streaming
  8. System outputs session summary
- **Alternative Flows**:
  - 3a. Initialization timeout (30 seconds):
    - System provides motion guidance
    - System resets initialization
  - 5a. Tracking lost:
    - Status: LOST
    - System attempts relocalization
    - If successful: Status: TRACKING
    - If failed after 5 seconds: reinitialize
- **Exception Flows**:
  - E1. End-to-end latency exceeds 100ms:
    - System logs warning
    - System may skip frames to catch up
- **Postconditions**:
  1. Continuous pose output at 30Hz
  2. Latency < 100ms maintained
  3. Tracking retention > 95%

#### 3.10.2 Acceptance Criteria

- [ ] AC-010.1: Pose output at 30Hz during tracking
- [ ] AC-010.2: End-to-end latency < 100ms (95th percentile)
- [ ] AC-010.3: Position accuracy ATE RMSE < 0.1m
- [ ] AC-010.4: Tracking retention rate > 95%
- [ ] AC-010.5: Automatic recovery from tracking loss
- [ ] AC-010.6: Session duration up to 30 minutes

#### 3.10.3 Dependencies

- Depends on: SF-005, SF-006, SF-007, SF-008
- Blocks: SF-011 (Output and Export)

---

### SF-011: Output and Export

**Source**: FR9.1, FR9.2, FR9.3, FR9.4 (PRD)
**Priority**: P0 (Trajectory), P1 (Point Cloud, API), P2 (Mesh)
**Description**: Export SLAM results and provide real-time pose API.

#### 3.11.1 Use Cases

##### UC-013: Trajectory Export

- **Actor**: User, Export Module
- **Preconditions**:
  1. SLAM session completed or in progress
  2. Trajectory data available
- **Main Flow**:
  1. User selects "Export Trajectory"
  2. System displays format options:
     - TUM format (timestamp tx ty tz qx qy qz qw)
     - KITTI format (3x4 transformation matrix)
     - EuRoC format
  3. User selects output format
  4. User specifies output path
  5. System exports trajectory
  6. System displays export confirmation
- **Postconditions**:
  1. Trajectory file saved in selected format
  2. All poses included with timestamps

##### UC-014: Real-time Pose API

- **Actor**: External Application, Output Module
- **Preconditions**:
  1. Real-time SLAM in progress
  2. ROS or ZMQ interface configured
- **Main Flow**:
  1. External application connects to pose API
  2. System publishes pose at each SLAM output:
     a. ROS: /vi_slam/pose (geometry_msgs/PoseStamped)
     b. ZMQ: JSON-formatted pose message
  3. External application receives and uses pose
  4. Connection maintained until session ends
- **Alternative Flows**:
  - 1a. Connection failure:
    - System retries connection
    - System logs connection errors
- **Postconditions**:
  1. Poses delivered in real-time
  2. API latency < 10ms

##### UC-015: Point Cloud Export

- **Actor**: User, Export Module
- **Preconditions**:
  1. Map available from SLAM
  2. SLAM session allows map export
- **Main Flow**:
  1. User selects "Export Point Cloud"
  2. System displays format options:
     - PLY (ASCII/Binary)
     - PCD
  3. User selects format and options
  4. System exports point cloud
  5. System displays export confirmation
- **Postconditions**:
  1. Point cloud file saved
  2. Color information included if available

#### 3.11.2 Acceptance Criteria

- [ ] AC-011.1: TUM format trajectory export
- [ ] AC-011.2: KITTI format trajectory export
- [ ] AC-011.3: PLY point cloud export
- [ ] AC-011.4: ROS topic publishing at pose rate
- [ ] AC-011.5: ZMQ API with < 10ms latency
- [ ] AC-011.6: Export completes within 5 seconds for typical sessions

#### 3.11.3 Dependencies

- Depends on: SF-007, SF-009, SF-010
- Blocks: None (output feature)

---

### SF-012: Visualization

**Source**: FR3.5 (PRD)
**Priority**: P1
**Description**: Real-time visualization of trajectory, point cloud, and camera feed.

#### 3.12.1 Use Cases

##### UC-016: Real-time Visualization

- **Actor**: User, Visualization Module
- **Preconditions**:
  1. SLAM session active
  2. Display available
- **Main Flow**:
  1. User enables visualization
  2. System creates visualization window
  3. System displays:
     - Camera feed with features
     - 3D trajectory plot
     - Point cloud (if available)
     - Status information
  4. Visualization updates at 30Hz
  5. User can interact:
     - Rotate/zoom 3D view
     - Toggle display elements
  6. User closes visualization
- **Postconditions**:
  1. Real-time visual feedback provided
  2. Performance impact < 20% CPU

#### 3.12.2 Acceptance Criteria

- [ ] AC-012.1: 30 FPS visualization refresh
- [ ] AC-012.2: Interactive 3D view (rotate, zoom)
- [ ] AC-012.3: Trajectory, point cloud, camera feed display
- [ ] AC-012.4: Status overlay (FPS, tracking status)
- [ ] AC-012.5: CPU overhead < 20%

#### 3.12.3 Dependencies

- Depends on: SF-007, SF-010
- Blocks: None

---

### SF-013: ROS Integration

**Source**: FR3.7 (PRD)
**Priority**: P1
**Description**: Publish SLAM outputs to ROS topics for integration with robotics stack.

#### 3.13.1 Use Cases

##### UC-017: ROS Topic Publishing

- **Actor**: PC Client, ROS Network
- **Preconditions**:
  1. ROS (Noetic or Humble) installed
  2. roscore running (ROS 1) or daemon active (ROS 2)
  3. SLAM session active
- **Main Flow**:
  1. User enables ROS publishing
  2. System creates ROS node "vi_slam_node"
  3. System advertises topics:
     - /vi_slam/pose (PoseStamped)
     - /vi_slam/odometry (Odometry)
     - /vi_slam/path (Path)
     - /vi_slam/pointcloud (PointCloud2)
     - /vi_slam/image (Image) - optional
  4. For each SLAM output:
     a. System creates ROS message
     b. System publishes to appropriate topic
  5. ROS subscribers receive data
  6. On session end, system shuts down node cleanly
- **Postconditions**:
  1. ROS topics actively publishing
  2. TF transforms published (map -> odom -> base_link)

#### 3.13.2 Acceptance Criteria

- [ ] AC-013.1: ROS Noetic (ROS 1) support
- [ ] AC-013.2: ROS Humble (ROS 2) support
- [ ] AC-013.3: Standard message types used
- [ ] AC-013.4: TF tree published
- [ ] AC-013.5: Works with RViz visualization

#### 3.13.3 Dependencies

- Depends on: SF-010
- Blocks: None

---

### SF-014: Settings and Configuration UI

**Source**: FR1.6, FR1.7 (PRD)
**Priority**: P1
**Description**: User interface for configuring capture and display settings.

#### 3.14.1 Use Cases

##### UC-018: Configuration Management

- **Actor**: User
- **Preconditions**:
  1. Application running
- **Main Flow**:
  1. User opens Settings
  2. System displays configuration categories:
     - Camera: resolution, FPS, exposure mode
     - IMU: sampling frequency, sensor type
     - Streaming: server IP, port, protocol
     - Storage: output directory, file format
  3. User modifies settings
  4. System validates settings
  5. User saves settings
  6. System applies changes
- **Postconditions**:
  1. Settings persisted
  2. Changes take effect

#### 3.14.2 Acceptance Criteria

- [ ] AC-014.1: Camera resolution selection (720p, 1080p)
- [ ] AC-014.2: FPS selection (15, 30, 60)
- [ ] AC-014.3: IMU frequency selection (100-500Hz)
- [ ] AC-014.4: Settings persistence across restarts
- [ ] AC-014.5: Real-time preview of settings

#### 3.14.3 Dependencies

- Depends on: SF-001, SF-002
- Blocks: None

---

### SF-015: Status Display

**Source**: FR1.7 (PRD)
**Priority**: P1
**Description**: Display real-time system status including FPS, connection, and buffer states.

#### 3.15.1 Use Cases

##### UC-019: Status Monitoring

- **Actor**: User
- **Preconditions**:
  1. Application running
  2. Capture or streaming active
- **Main Flow**:
  1. System displays status overlay:
     - Camera FPS (actual)
     - IMU rate (actual)
     - Connection status (connected/disconnected)
     - Buffer utilization (%)
     - Recording duration/size
  2. Status updates every 500ms
  3. Warnings highlighted in amber/red
- **Postconditions**:
  1. User aware of system state

#### 3.15.2 Acceptance Criteria

- [ ] AC-015.1: Real-time FPS display
- [ ] AC-015.2: Connection status indicator
- [ ] AC-015.3: Buffer utilization display
- [ ] AC-015.4: Warning indicators for issues
- [ ] AC-015.5: Update rate >= 2Hz

#### 3.15.3 Dependencies

- Depends on: SF-001, SF-002, SF-005
- Blocks: None

---

## 4. External Interface Requirements

### 4.1 User Interfaces

#### 4.1.1 Android Application UI

| Screen ID | Name | Description |
|-----------|------|-------------|
| UI-001 | Main Screen | Camera preview, record/stream buttons, status display |
| UI-002 | Settings Screen | Capture configuration options |
| UI-003 | Calibration Screen | Step-by-step calibration guide |
| UI-004 | Session History | List of recorded sessions |
| UI-005 | Connection Screen | Server IP entry, connection status |

**UI-001: Main Screen Wireframe**
```
+----------------------------------+
|  VI-SLAM            [Settings]  |
+----------------------------------+
|                                  |
|                                  |
|       [Camera Preview]           |
|                                  |
|                                  |
+----------------------------------+
| FPS: 30  IMU: 200Hz  [Connected] |
+----------------------------------+
|  [Record]    [Stream]    [Stop]  |
+----------------------------------+
```

#### 4.1.2 PC Client UI

| Screen ID | Name | Description |
|-----------|------|-------------|
| UI-101 | Dashboard | Connection status, data reception stats |
| UI-102 | Visualization | 3D trajectory, point cloud viewer |
| UI-103 | Configuration | Framework selection, calibration loader |
| UI-104 | Processing | Offline dataset processor |
| UI-105 | Export | Output format selection |

### 4.2 API Interfaces

#### 4.2.1 Smartphone to PC Communication

| Endpoint | Protocol | Direction | Description |
|----------|----------|-----------|-------------|
| WebRTC Video | WebRTC/DTLS | App -> PC | H.264 encoded video frames |
| WebRTC Data | WebRTC/SCTP | App <-> PC | Control messages, metadata |
| IMU Stream | UDP | App -> PC | Binary IMU data packets |
| Control | TCP | PC -> App | Start/stop commands |

**IMU UDP Packet Format:**
```
Offset  Size  Type    Field
0       8     int64   timestamp_ns
8       8     double  acc_x (m/s^2)
16      8     double  acc_y
24      8     double  acc_z
32      8     double  gyro_x (rad/s)
40      8     double  gyro_y
48      8     double  gyro_z
Total: 56 bytes per sample
```

**WebRTC Data Channel Messages:**
```json
{
  "type": "frame_metadata",
  "timestamp_ns": 1234567890123456789,
  "sequence": 12345,
  "exposure_ns": 8333333,
  "iso": 100
}
```

#### 4.2.2 SLAM Framework Adapter Interface

```cpp
class ISLAMFramework {
public:
    virtual ~ISLAMFramework() = default;

    // Initialization
    virtual bool initialize(const std::string& config_path) = 0;
    virtual bool loadCalibration(const std::string& calib_path) = 0;

    // Data input
    virtual void processImage(const cv::Mat& image, int64_t timestamp_ns) = 0;
    virtual void processIMU(const IMUSample& imu) = 0;

    // Output
    virtual bool getPose(Pose6DoF& pose) const = 0;
    virtual TrackingStatus getStatus() const = 0;
    virtual std::vector<MapPoint> getMapPoints() const = 0;

    // Control
    virtual void reset() = 0;
    virtual void shutdown() = 0;
};

struct Pose6DoF {
    int64_t timestamp_ns;
    double position[3];      // x, y, z in meters
    double orientation[4];   // qw, qx, qy, qz (quaternion)
    double covariance[36];   // 6x6 covariance matrix (optional)
};

enum class TrackingStatus {
    INITIALIZING,
    TRACKING,
    LOST,
    RELOCALIZATION
};
```

#### 4.2.3 ROS Interface

**Published Topics (ROS 1/2):**

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| /vi_slam/pose | geometry_msgs/PoseStamped | 30 Hz | Current pose |
| /vi_slam/odometry | nav_msgs/Odometry | 30 Hz | Pose with velocity |
| /vi_slam/path | nav_msgs/Path | 1 Hz | Full trajectory |
| /vi_slam/pointcloud | sensor_msgs/PointCloud2 | 1 Hz | Map points |
| /vi_slam/image | sensor_msgs/Image | 30 Hz | Camera feed |
| /vi_slam/status | std_msgs/String | 1 Hz | Tracking status |

**TF Frames:**
```
map
 └── odom
      └── base_link
           └── camera_link
           └── imu_link
```

#### 4.2.4 ZMQ Pose API

**Socket Configuration:**
- Type: PUB/SUB
- Port: 5555 (configurable)
- Protocol: TCP

**Message Format (JSON):**
```json
{
  "timestamp_ns": 1234567890123456789,
  "position": {
    "x": 1.234,
    "y": 2.345,
    "z": 0.567
  },
  "orientation": {
    "w": 0.707,
    "x": 0.0,
    "y": 0.707,
    "z": 0.0
  },
  "status": "TRACKING",
  "velocity": {
    "linear": [0.1, 0.0, 0.0],
    "angular": [0.0, 0.0, 0.01]
  }
}
```

### 4.3 Hardware Interfaces

| Interface | Description | Requirements |
|-----------|-------------|--------------|
| Camera | Rear camera access via Camera2 API | HARDWARE_LEVEL_LIMITED+ |
| IMU | Accelerometer + Gyroscope via SensorManager | TYPE_ACCELEROMETER, TYPE_GYROSCOPE |
| Storage | Internal/External storage for recordings | Read/Write permission |
| Network | WiFi or USB tethering for streaming | 5 Mbps+ bandwidth |
| Display | Screen for UI and preview | Minimum 720p |

### 4.4 Software Interfaces

| Interface | Description | Version |
|-----------|-------------|---------|
| Android Camera2 API | Camera capture interface | API 26+ |
| Android SensorManager | IMU sensor access | API 26+ |
| Google libwebrtc | WebRTC implementation | M100+ |
| OpenCV | Image processing | 4.5+ |
| Eigen3 | Linear algebra | 3.3+ |
| Ceres Solver | Nonlinear optimization | 2.0+ |
| ROS | Robot Operating System | Noetic/Humble |
| ZeroMQ | Message queue library | 4.3+ |

### 4.5 Communication Interfaces

#### 4.5.1 WebRTC Signaling

**Signaling Server Protocol (WebSocket):**

```json
// Offer (App -> Signaling Server -> PC)
{
  "type": "offer",
  "sdp": "v=0\r\no=- ..."
}

// Answer (PC -> Signaling Server -> App)
{
  "type": "answer",
  "sdp": "v=0\r\no=- ..."
}

// ICE Candidate
{
  "type": "candidate",
  "candidate": "candidate:..."
}
```

#### 4.5.2 Network Configuration

| Parameter | Default | Range |
|-----------|---------|-------|
| WebRTC Port | 8080 | 1024-65535 |
| UDP IMU Port | 9000 | 1024-65535 |
| TCP Control Port | 9001 | 1024-65535 |
| ZMQ Pose Port | 5555 | 1024-65535 |
| ROS Master URI | localhost:11311 | Configurable |

---

## 5. Non-Functional Requirements

### 5.1 Performance Requirements

| ID | Requirement | Metric | Target | Measurement Method |
|----|-------------|--------|--------|-------------------|
| NFR-P001 | End-to-end Latency | Time from capture to pose output | < 100 ms | Timestamp difference measurement |
| NFR-P002 | Position Accuracy | ATE RMSE | < 0.1 m | EuRoC benchmark comparison |
| NFR-P003 | Processing Speed | SLAM output rate | >= 30 FPS | Output rate measurement |
| NFR-P004 | IMU Sampling Rate | Actual sampling frequency | >= 200 Hz | In-app measurement |
| NFR-P005 | Frame Drop Rate | Percentage of dropped frames | < 1% | Frame counter analysis |
| NFR-P006 | App CPU Usage | CPU utilization by app | < 50% | Android profiler |
| NFR-P007 | App Memory Usage | RAM used by app | < 500 MB | Android profiler |
| NFR-P008 | PC CPU Usage | VI-SLAM processing | < 80% | System monitor |
| NFR-P009 | PC Memory Usage | Total RAM usage | < 4 GB | System monitor |
| NFR-P010 | Network Bandwidth | Required bandwidth for streaming | < 10 Mbps | Network monitor |

### 5.2 Reliability Requirements

| ID | Requirement | Metric | Target | Verification |
|----|-------------|--------|--------|--------------|
| NFR-R001 | System Uptime | Continuous operation time | >= 30 min | Extended testing |
| NFR-R002 | Tracking Retention | Percentage of successful tracking | >= 95% | Normal conditions testing |
| NFR-R003 | Data Integrity | Corruption-free data storage | 100% | Checksum verification |
| NFR-R004 | Error Recovery | Auto-reconnection on network loss | Within 5 sec | Disconnection testing |
| NFR-R005 | Crash Rate | Application crashes per session | < 0.1% | Crash reporting |
| NFR-R006 | Initialization Success | Successful SLAM initialization | >= 95% | Repeated initialization tests |

### 5.3 Usability Requirements

| ID | Requirement | Metric | Target | Verification |
|----|-------------|--------|--------|--------------|
| NFR-U001 | Installation Time | Time to complete setup | < 30 min | User testing |
| NFR-U002 | Calibration Time | Time to complete calibration | < 10 min | User testing |
| NFR-U003 | Learning Curve | Time to basic proficiency | < 1 day | User feedback |
| NFR-U004 | Documentation | Feature coverage | 100% | Documentation review |
| NFR-U005 | Error Messages | Clarity and actionability | Include resolution | User feedback |
| NFR-U006 | First Run Success | Users completing first session | >= 90% | User testing |

### 5.4 Compatibility Requirements

| ID | Requirement | Specification |
|----|-------------|---------------|
| NFR-C001 | Android Version | 8.0 (API 26) or higher |
| NFR-C002 | iOS Version | 11.0 or higher (Phase 2) |
| NFR-C003 | PC Operating System | Ubuntu 20.04/22.04, Windows 10/11 (WSL) |
| NFR-C004 | ROS Version | Noetic (ROS 1), Humble (ROS 2) |
| NFR-C005 | Supported Devices | >= 20 smartphone models verified |
| NFR-C006 | Framework Compatibility | VINS-Mono, OpenVINS, ORB-SLAM3, Basalt |

### 5.5 Security Requirements

| ID | Requirement | Description | Implementation |
|----|-------------|-------------|----------------|
| NFR-S001 | Data Encryption | Streaming data encryption | DTLS via WebRTC |
| NFR-S002 | Local Storage | App-exclusive storage protection | Android app sandbox |
| NFR-S003 | Permissions | Minimal permission requests | Camera, Storage, Network only |
| NFR-S004 | Network Auth | Optional connection authentication | Pre-shared key option |
| NFR-S005 | Data Privacy | No data sent to external servers | Local-only processing |

### 5.6 Maintainability Requirements

| ID | Requirement | Metric | Target |
|----|-------------|--------|--------|
| NFR-M001 | Code Coverage | Test coverage percentage | >= 70% |
| NFR-M002 | Documentation | Public API documentation | 100% |
| NFR-M003 | Modularity | Module coupling | Loose coupling design |
| NFR-M004 | CI/CD | Automated build and test | All commits |
| NFR-M005 | Code Style | Style guide compliance | Enforced by linter |

### 5.7 Portability Requirements

| ID | Requirement | Description |
|----|-------------|-------------|
| NFR-PT001 | Platform Abstraction | Framework-agnostic data interface |
| NFR-PT002 | Configuration | External configuration files |
| NFR-PT003 | Dependencies | Documented dependency management |
| NFR-PT004 | Cross-platform | PC client runs on Linux and Windows (WSL) |

---

## 6. Data Requirements

### 6.1 Data Entities

#### 6.1.1 IMU Sample

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| timestamp | int64 | nanoseconds | Sensor timestamp (CLOCK_BOOTTIME) |
| acc_x | float64 | m/s^2 | X-axis acceleration |
| acc_y | float64 | m/s^2 | Y-axis acceleration |
| acc_z | float64 | m/s^2 | Z-axis acceleration |
| gyro_x | float64 | rad/s | X-axis angular velocity |
| gyro_y | float64 | rad/s | Y-axis angular velocity |
| gyro_z | float64 | rad/s | Z-axis angular velocity |

#### 6.1.2 Camera Frame

| Field | Type | Description |
|-------|------|-------------|
| timestamp | int64 | Sensor timestamp (nanoseconds) |
| image | byte[] | Pixel data (YUV420/RGB/Grayscale) |
| width | int32 | Image width in pixels |
| height | int32 | Image height in pixels |
| format | string | Pixel format identifier |
| exposure_time | int64 | Exposure duration (nanoseconds) |
| iso | int32 | ISO sensitivity value |
| focal_length | float32 | Physical focal length (mm) |

#### 6.1.3 Pose (6DoF)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| timestamp | int64 | nanoseconds | Pose timestamp |
| position_x | float64 | meters | X position |
| position_y | float64 | meters | Y position |
| position_z | float64 | meters | Z position |
| orientation_w | float64 | - | Quaternion W |
| orientation_x | float64 | - | Quaternion X |
| orientation_y | float64 | - | Quaternion Y |
| orientation_z | float64 | - | Quaternion Z |

#### 6.1.4 Calibration Parameters

| Field | Type | Description |
|-------|------|-------------|
| camera_model | string | Camera model (pinhole, fisheye) |
| fx, fy | float64 | Focal lengths (pixels) |
| cx, cy | float64 | Principal point (pixels) |
| distortion | float64[4-8] | Distortion coefficients |
| T_cam_imu | float64[4x4] | Camera-IMU transformation |
| time_offset | float64 | Camera-IMU time offset (seconds) |
| imu_noise | object | IMU noise parameters |

#### 6.1.5 Map Point

| Field | Type | Description |
|-------|------|-------------|
| id | int64 | Unique point identifier |
| position | float64[3] | 3D position (x, y, z) |
| color | uint8[3] | RGB color (optional) |
| observations | int32 | Number of observations |

### 6.2 Data Relationships

```
+----------------+       +---------------+       +---------------+
|  IMU Sample    |       | Camera Frame  |       |     Pose      |
+----------------+       +---------------+       +---------------+
| timestamp (PK) |       | timestamp (PK)|       | timestamp (PK)|
| acc[3]         |       | image         |       | position[3]   |
| gyro[3]        |       | metadata      |       | orientation[4]|
+----------------+       +---------------+       +---------------+
        |                       |                       |
        +----------+------------+                       |
                   |                                    |
                   v                                    |
        +-------------------+                          |
        | Synchronized Data |                          |
        +-------------------+                          |
        | frame             |------------------------->|
        | imu_samples[]     |                          |
        +-------------------+                          |
                   |                                    |
                   v                                    |
        +-------------------+       +------------------+
        |   SLAM Engine     |------>|   Map Points     |
        +-------------------+       +------------------+
```

### 6.3 Data Constraints

| Constraint | Description | Enforcement |
|------------|-------------|-------------|
| Timestamp Monotonicity | IMU timestamps must be strictly increasing | Runtime validation |
| Frame Rate Bounds | Camera FPS must be 15-60 | Configuration validation |
| IMU Rate Bounds | IMU rate must be 100-500 Hz | Configuration validation |
| Calibration Validity | All calibration parameters must be within valid ranges | Load-time validation |
| Storage Limits | Recording stops at 50MB free space | Runtime monitoring |
| Buffer Limits | Maximum buffer sizes enforced | Memory management |

### 6.4 Data Formats

#### 6.4.1 IMU CSV Format (Storage)

```csv
#timestamp[ns],ax[m/s^2],ay[m/s^2],az[m/s^2],gx[rad/s],gy[rad/s],gz[rad/s]
1000000000,0.1,-0.2,9.8,0.001,-0.002,0.001
1005000000,0.1,-0.2,9.81,0.001,-0.002,0.001
```

#### 6.4.2 Metadata JSON Format

```json
{
  "format_version": "1.0",
  "device": {
    "model": "Pixel 6",
    "manufacturer": "Google",
    "android_version": "12",
    "api_level": 31
  },
  "camera": {
    "resolution": [1920, 1080],
    "fps": 30,
    "codec": "H.264",
    "timestamp_source": "SENSOR_TIMESTAMP",
    "camera2_level": "FULL"
  },
  "imu": {
    "accelerometer_type": "TYPE_ACCELEROMETER",
    "gyroscope_type": "TYPE_GYROSCOPE",
    "sampling_rate_hz": 200
  },
  "calibration": {
    "camera_model": "pinhole",
    "fx": 1500.0,
    "fy": 1500.0,
    "cx": 960.0,
    "cy": 540.0,
    "distortion_model": "radtan",
    "distortion_coeffs": [-0.28, 0.07, 0.0, 0.0],
    "T_cam_imu": [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ],
    "time_offset_s": 0.0
  },
  "recording": {
    "start_time_utc": "2026-01-20T10:00:00.000Z",
    "duration_ms": 60000,
    "frame_count": 1800,
    "imu_sample_count": 12000,
    "file_size_bytes": 125000000
  }
}
```

#### 6.4.3 Trajectory TUM Format

```
# timestamp tx ty tz qx qy qz qw
1403636579.763555992 4.688319 -1.786938 0.783338 0.531152 -0.083869 -0.018332 0.843092
1403636579.863555992 4.688123 -1.786742 0.783421 0.531248 -0.083712 -0.018421 0.843012
```

#### 6.4.4 Calibration YAML Format (VINS-Mono Compatible)

```yaml
%YAML:1.0

model_type: PINHOLE
camera_name: smartphone_cam
image_width: 1920
image_height: 1080

distortion_parameters:
  k1: -0.28
  k2: 0.07
  p1: 0.0
  p2: 0.0

projection_parameters:
  fx: 1500.0
  fy: 1500.0
  cx: 960.0
  cy: 540.0

body_T_cam0:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]

td: 0.0

imu_params:
  acc_n: 0.08
  acc_w: 0.00004
  gyr_n: 0.004
  gyr_w: 2.0e-6
```

---

## 7. Traceability Matrix

### 7.1 PRD to SRS Traceability

| PRD Requirement | SRS Feature(s) | Use Case(s) | Priority |
|-----------------|----------------|-------------|----------|
| FR1.1 Camera Capture | SF-001 | UC-001 | P0 |
| FR1.2 IMU Capture | SF-002 | UC-002 | P0 |
| FR1.3 Timestamp Sync | SF-003 | UC-003 | P0 |
| FR1.4 Local Storage | SF-004 | UC-004 | P0 |
| FR1.5 Real-time Streaming | SF-005 | UC-005 | P1 |
| FR1.6 Settings UI | SF-014 | UC-018 | P1 |
| FR1.7 Status Display | SF-015 | UC-019 | P1 |
| FR1.8 Calibration UI | SF-008 | UC-009, UC-010 | P2 |
| FR1.9 OIS Disable | SF-001 (extension) | - | P2 |
| FR3.1 Data Reception | SF-006 | UC-006 | P0 |
| FR3.2 Video Decoding | SF-006 | UC-006 | P0 |
| FR3.3 Data Synchronization | SF-003, SF-006 | UC-003, UC-006 | P0 |
| FR3.4 SLAM Integration | SF-007 | UC-007, UC-008 | P0 |
| FR3.5 Visualization | SF-012 | UC-016 | P1 |
| FR3.6 Data Storage | SF-004 | UC-004 | P1 |
| FR3.7 ROS Integration | SF-013 | UC-017 | P1 |
| FR4.1 WebRTC Video | SF-005 | UC-005 | P1 |
| FR4.2 UDP IMU | SF-005 | UC-005 | P1 |
| FR4.3 Connection Mgmt | SF-005 | UC-005 | P1 |
| FR4.4 Buffering | SF-005, SF-006 | UC-005, UC-006 | P1 |
| FR4.5 Bandwidth Adaptation | SF-005 (extension) | UC-005 | P2 |
| FR5.1 Timestamp Alignment | SF-003 | UC-003 | P0 |
| FR5.2 IMU Interpolation | SF-003 | UC-003 | P0 |
| FR5.3 Dropped Frame Handling | SF-003, SF-005 | UC-003 | P1 |
| FR5.4 Latency Compensation | SF-005, SF-006 | UC-005 | P1 |
| FR6.1 EuRoC Format | SF-004, SF-009 | UC-004, UC-011 | P0 |
| FR6.2 TUM Format | SF-011 | UC-013 | P1 |
| FR6.3 ROS bag | SF-011 (extension) | - | P1 |
| FR6.4 CSV/Binary | SF-004 | UC-004 | P0 |
| FR7.1 Initialization | SF-007 | UC-007 | P0 |
| FR7.2 Tracking | SF-007 | UC-007, UC-012 | P0 |
| FR7.3 Map Management | SF-007 | UC-007 | P1 |
| FR7.4 Loop Closing | SF-007 (extension) | - | P2 |
| FR7.5 Relocalization | SF-007 | UC-007 | P1 |
| FR8.1 Camera Intrinsic | SF-008 | UC-009 | P0 |
| FR8.2 Camera-IMU Extrinsic | SF-008 | UC-010 | P1 |
| FR8.3 Time Offset | SF-008 | UC-010 | P1 |
| FR8.4 IMU Noise Parameters | SF-008 (extension) | - | P2 |
| FR8.5 Auto Calibration | SF-008 (extension) | - | P2 |
| FR9.1 Trajectory Export | SF-011 | UC-013 | P0 |
| FR9.2 Point Cloud | SF-011 | UC-015 | P1 |
| FR9.3 Mesh | SF-011 (extension) | - | P2 |
| FR9.4 Real-time API | SF-011 | UC-014 | P1 |

### 7.2 Use Case to PRD Traceability

| PRD Use Case | SRS Use Case(s) | SRS Feature(s) |
|--------------|-----------------|----------------|
| UC1: Real-time Pose | UC-005, UC-006, UC-007, UC-012, UC-014, UC-017 | SF-005, SF-006, SF-007, SF-010, SF-011, SF-013 |
| UC2: Data Collection | UC-001, UC-002, UC-003, UC-004 | SF-001, SF-002, SF-003, SF-004 |
| UC3: Offline SLAM | UC-011, UC-013, UC-015 | SF-009, SF-011 |
| UC4: Calibration | UC-009, UC-010 | SF-008 |
| UC5: 3D Map Generation | UC-015 | SF-011 |

### 7.3 NFR Traceability

| PRD NFR | SRS NFR ID | Specification |
|---------|------------|---------------|
| NFR1.1 End-to-end Latency | NFR-P001 | < 100 ms |
| NFR1.2 Position Accuracy | NFR-P002 | ATE RMSE < 0.1 m |
| NFR1.3 Processing Speed | NFR-P003 | >= 30 FPS |
| NFR1.4 IMU Sampling Rate | NFR-P004 | >= 200 Hz |
| NFR1.5 Frame Drop Rate | NFR-P005 | < 1% |
| NFR1.6 CPU Usage (App) | NFR-P006 | < 50% |
| NFR1.7 Memory Usage (App) | NFR-P007 | < 500 MB |
| NFR2.1 System Uptime | NFR-R001 | >= 30 min |
| NFR2.2 Tracking Retention | NFR-R002 | >= 95% |
| NFR2.3 Data Integrity | NFR-R003 | 100% |
| NFR2.4 Error Recovery | NFR-R004 | Auto-reconnect |
| NFR2.5 Crash Rate | NFR-R005 | < 0.1% |
| NFR3.1 Installation Time | NFR-U001 | < 30 min |
| NFR3.2 Calibration Time | NFR-U002 | < 10 min |
| NFR3.3 Learning Curve | NFR-U003 | < 1 day |
| NFR3.4 Documentation | NFR-U004 | 100% coverage |
| NFR3.5 Error Messages | NFR-U005 | Actionable |
| NFR4.1 Android Version | NFR-C001 | 8.0+ |
| NFR4.2 iOS Version | NFR-C002 | 11.0+ |
| NFR4.3 PC OS | NFR-C003 | Ubuntu 20.04/22.04 |
| NFR4.4 ROS Version | NFR-C004 | Noetic, Humble |
| NFR4.5 Supported Devices | NFR-C005 | >= 20 models |
| NFR5.1 Data Encryption | NFR-S001 | DTLS |
| NFR5.2 Local Data Protection | NFR-S002 | App sandbox |
| NFR5.3 Minimal Permissions | NFR-S003 | Necessary only |
| NFR5.4 Network Auth | NFR-S004 | Optional |
| NFR6.1 Code Coverage | NFR-M001 | >= 70% |
| NFR6.2 Documentation Level | NFR-M002 | All APIs |
| NFR6.3 Module Coupling | NFR-M003 | Loose |
| NFR6.4 CI/CD | NFR-M004 | Automated |

---

## 8. Appendix

### 8.1 Analysis Models

#### 8.1.1 State Machine: SLAM Tracking

```
                    +---------------+
                    | UNINITIALIZED |
                    +-------+-------+
                            |
                            | start()
                            v
                    +---------------+
           +------->| INITIALIZING  |
           |        +-------+-------+
           |                |
           |                | initialization_complete
           |                v
           |        +---------------+
           |   +--->|   TRACKING    |<---+
           |   |    +-------+-------+    |
           |   |            |            |
           |   |            | tracking_lost
           |   |            v            |
           |   |    +---------------+    |
           |   |    |     LOST      |    |
           |   |    +-------+-------+    |
           |   |            |            |
           |   |            | relocalization_success
           |   +------------+------------+
           |
           | reset() / initialization_failed
           |
           +---------------------------+
```

#### 8.1.2 Data Flow Diagram

```
[Camera]     [IMU]
    |           |
    v           v
+-------+   +-------+
|Capture|   |Capture|
+---+---+   +---+---+
    |           |
    +-----+-----+
          |
          v
    +-----------+
    |Synchronize|
    +-----+-----+
          |
    +-----+-----+
    |           |
    v           v
+-------+   +-------+
|Storage|   |Stream |
+-------+   +---+---+
                |
                v
          +-----------+
          |  Receive  |
          +-----+-----+
                |
                v
          +-----------+
          |SLAM Engine|
          +-----+-----+
                |
    +-----------+-----------+
    |           |           |
    v           v           v
+------+   +-------+   +------+
| Pose |   |  Map  |   | Viz  |
+------+   +-------+   +------+
```

### 8.2 Open Issues

| Issue ID | Description | Status | Owner |
|----------|-------------|--------|-------|
| OI-001 | iOS app development timeline TBD | Open | TBD |
| OI-002 | Auto-calibration algorithm selection | Open | TBD |
| OI-003 | Cloud processing architecture | Deferred | TBD |
| OI-004 | Unity/Unreal SDK specification | Deferred | TBD |
| OI-005 | Multi-camera support (stereo) | Future | TBD |

### 8.3 Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2026-01-24 | VI-SLAM Team | Initial SRS creation |

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Technical Lead | | | |
| QA Lead | | | |
| Project Manager | | | |

---

*This Software Requirements Specification defines the detailed requirements for the VI-SLAM system.*
*Document created: 2026-01-24*
*Version: 1.0.0*
