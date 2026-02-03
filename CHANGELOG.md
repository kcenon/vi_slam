# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Doxygen API documentation generation (#177)
- Code coverage reporting with Codecov integration (#178)
- Structured logging framework (planned, #171)

### Changed
- Migrated type definitions to Eigen-based types for better performance (#180)
- Pinned FetchContent dependencies with URL hash for reproducible builds (#179)

## [0.3.0] - 2026-02-01

### Added
- **SLAM Framework Adapters**
  - Basalt VIO adapter implementation (#176)
  - ORB-SLAM3 adapter implementation (#175)
  - OpenVINS adapter implementation (#173)
  - Missing SLAM framework configuration files (#174)

- **PC Client Dashboard**
  - Configuration panel for system settings (#161)
  - 3D visualization panel integration (#159)
  - Export controls for data and results (#158)
  - SLAM framework selection panel (#157)
  - Data reception statistics panel (#156)
  - Connection status dashboard panel (#155)
  - Dear ImGui UI framework setup (#154)

- **3D Visualization**
  - Status overlay display (#137)
  - Interactive 3D view controls (#136)
  - Trajectory visualization (#135)
  - Point cloud rendering (#134)
  - Basic 3D rendering engine (#129)

### Fixed
- CalibrationExporter and CalibrationVerifier test failures (#146)
- MainViewModel test failures and SettingsViewModel tests (#143)
- 31 failing Android unit tests (37% improvement) (#127)

### Changed
- Upgraded Kotlin and Compose for Java 21 compatibility (#139)

## [0.2.0] - 2026-01-15

### Added
- **Android UI**
  - CameraX integration with main screen camera preview (#141)
  - Settings Screen UI implementation (#123)
  - Main Screen UI with camera preview and controls (#122)
  - History Screen implementation with tests (#117)
  - Input validation and error messages for SettingsScreen (#116)
  - Calibration screen with step-by-step wizard (#108)
  - Session history screen with list and filtering (#107)
  - Settings screen UI (#106)
  - Main screen UI with Jetpack Compose (#101)

- **Unit Tests**
  - CalibrationViewModel tests (#125)
  - SessionRepository tests (#124)
  - MainViewModelTest compilation fixes (#115)
  - MainViewModel integration tests (#111)

- **Recording System**
  - Data recovery for interrupted recording sessions (#99)
  - OpenCV Android SDK dependency via JitPack (#98)

- **Calibration**
  - Calibration verification (CMP-011.5) (#97)
  - Calibration export module (CMP-011.4) (#96)
  - Time offset estimation (CMP-011.3) (#95)
  - Camera-IMU extrinsic calibration data collection (#94)
  - Camera intrinsic calibration module (#93)

## [0.1.0] - 2026-01-01

### Added
- **Recording System**
  - Metadata JSON generation for recording sessions (#85)
  - Video encoder for H.264 MP4 recording (#84)
  - IMU CSV writer for nanosecond-precision recording (#83)
  - LocalRecorder basic lifecycle management (#79)
  - IRecorder interface and data models (#77)

- **Data Management**
  - Frame drop detection and optimized statistics tracking (#75)
  - DataManager integration tests (#74)
  - Data routing and consumer management (#73)
  - DataManager core lifecycle management (#72)
  - IDataManager interface and data models (#67)

- **SLAM Processing**
  - BasaltAdapter for Basalt VIO integration (#64)
  - Trajectory export implementation (#61)

- **Sensor Processing**
  - TimestampSynchronizer for camera-IMU alignment (#62)
  - IMUCaptureService and sensor fusion (#59)
  - FrameProcessor and metadata extraction (#58)

- **Streaming**
  - H.264 encoder and WebRTC video track (#57)

### Infrastructure
- GitHub Actions CI/CD pipeline
- Multi-platform build support (Linux, macOS, Windows)
- Android and PC client modular architecture
- ROS integration support (optional)

---

## Version History Summary

| Version | Date | Highlights |
|---------|------|------------|
| 0.3.0 | 2026-02-01 | SLAM adapters, PC dashboard, 3D visualization |
| 0.2.0 | 2026-01-15 | Android UI, calibration system, recording |
| 0.1.0 | 2026-01-01 | Initial release with core functionality |

[Unreleased]: https://github.com/kcenon/vi_slam/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/kcenon/vi_slam/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/kcenon/vi_slam/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/kcenon/vi_slam/releases/tag/v0.1.0
