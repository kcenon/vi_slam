# VI-SLAM: Visual-Inertial SLAM System

[![CI](https://github.com/kcenon/vi_slam/actions/workflows/ci.yml/badge.svg)](https://github.com/kcenon/vi_slam/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/kcenon/vi_slam/branch/main/graph/badge.svg)](https://codecov.io/gh/kcenon/vi_slam)

A Visual-Inertial Simultaneous Localization and Mapping (VI-SLAM) system with Android sensor data collection and PC-based SLAM processing.

## System Architecture

```
┌─────────────────┐       Network        ┌─────────────────┐
│  Android Device │ ◄─────────────────► │   PC Client     │
│  - Camera2 API  │   (WebRTC/ZMQ)      │  - SLAM Backend │
│  - IMU Sensors  │                      │  - Visualization│
└─────────────────┘                      └─────────────────┘
```

## Project Structure

```
vi_slam/
├── android/                  # Android app for sensor data collection
│   ├── app/
│   ├── build.gradle.kts
│   └── settings.gradle.kts
├── pc_client/               # PC client for SLAM processing
│   ├── src/
│   ├── include/
│   ├── python/
│   └── CMakeLists.txt
├── scripts/                 # Build and utility scripts
├── docs/                    # Documentation
├── config/                  # Configuration files
├── docker/                  # Docker development environment
└── .github/                 # GitHub Actions CI/CD
```

## Prerequisites

### For Android Development

- Android Studio Arctic Fox or later
- JDK 17
- Android SDK 34
- Gradle 8.x

### For PC Client Development

- CMake 3.16 or later
- C++17 compatible compiler (GCC 9+, Clang 10+, MSVC 2019+)
- Python 3.8+
- Required libraries:
  - OpenCV 4.5+
  - Eigen3
  - ZeroMQ (optional)
  - Ceres Solver (optional)
  - ROS (optional, for ROS integration)

#### Installing Dependencies

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake \
    libopencv-dev \
    libeigen3-dev \
    libzmq3-dev \
    python3 python3-pip
```

**macOS:**
```bash
brew install cmake opencv eigen zeromq python3
```

**Windows:**
```bash
# Using vcpkg
vcpkg install opencv4 eigen3 zeromq
```

## Building

### Android App

```bash
cd android
./gradlew build
```

Or use the build script:
```bash
./scripts/build_android.sh
```

The APK will be located at:
```
android/app/build/outputs/apk/debug/app-debug.apk
```

### PC Client

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Or use the build script:
```bash
./scripts/build_pc_client.sh Release
```

#### Building with ROS Support

To enable ROS topic publishing:

```bash
mkdir build && cd build
cmake .. -DENABLE_ROS=ON
make -j$(nproc)
```

**Prerequisites for ROS:**
- ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
- Source ROS environment before building:
  ```bash
  source /opt/ros/noetic/setup.bash  # or melodic
  ```

### Python Environment Setup

```bash
./scripts/setup_python_env.sh
source pc_client/python/venv/bin/activate
```

## Docker Development Environment

For a consistent development environment across platforms:

```bash
cd docker
docker-compose up -d
docker-compose exec vi-slam-dev bash
```

Inside the container:
```bash
cd /workspace/pc_client/build
cmake ..
make -j$(nproc)
```

See [docker/README.md](docker/README.md) for more details.

## Running

### Android App

1. Install the APK on your Android device
2. Grant camera and sensor permissions
3. Configure the server address in settings
4. Start streaming

### PC Client

```bash
./pc_client/build/vi_slam_pc_client
```

Or with custom configuration:
```bash
./pc_client/build/vi_slam_pc_client --config config/pc_client_config.yaml
```

## Configuration

### Android Configuration

Edit `config/android_config.yaml`:
```yaml
camera:
  target_resolution: "1920x1080"
  target_fps: 30

imu:
  accelerometer_rate: 200  # Hz
  gyroscope_rate: 200      # Hz
```

### PC Client Configuration

Edit `config/pc_client_config.yaml`:
```yaml
slam:
  type: "vins_mono"
  config_file: "config/vins_mono.yaml"

processing:
  num_threads: 4
  enable_gpu: false
```

## Testing

### PC Client Tests

```bash
cd build
ctest --output-on-failure
```

### Android Tests

```bash
cd android
./gradlew test
./gradlew connectedAndroidTest  # Requires connected device
```

## CI/CD

GitHub Actions automatically:
- Builds Android app and PC client
- Runs tests
- Lints code
- Uploads build artifacts

See [.github/workflows/ci.yml](.github/workflows/ci.yml) for details.

## ROS Integration

When built with `-DENABLE_ROS=ON`, VI-SLAM publishes SLAM output to ROS topics for real-time visualization and integration with the ROS ecosystem.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/slam/pose` | `geometry_msgs/PoseStamped` | Current camera pose |
| `/slam/odom` | `nav_msgs/Odometry` | Odometry with velocity estimation |
| `/slam/path` | `nav_msgs/Path` | Full trajectory history |

### Frame Convention

- **map**: Fixed world frame
- **base_link**: Moving camera/robot frame

### Usage Example

```cpp
#include "slam/slam_engine.hpp"

#ifdef ENABLE_ROS
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "vi_slam_node");
    ros::NodeHandle nh;

    // Create SLAM engine
    vi_slam::SLAMEngine engine;
    engine.initialize("config/slam_config.yaml");

    // Enable ROS publisher with custom configuration
    vi_slam::output::ROSPublisherConfig rosConfig;
    rosConfig.poseTopicName = "/slam/pose";
    rosConfig.frameId = "map";
    rosConfig.maxPathLength = 1000;

    engine.enableROSPublisher(nh, rosConfig);

    // Process images and IMU data...
    // Poses will be automatically published to ROS topics

    ros::spin();
    return 0;
}
#endif
```

### Visualization in RViz

```bash
# Terminal 1: Start ROS core
roscore

# Terminal 2: Run VI-SLAM with ROS enabled
./build/vi_slam_node

# Terminal 3: Launch RViz
rosrun rviz rviz
```

In RViz, add displays:
- **Path**: Subscribe to `/slam/path` with frame `map`
- **Odometry**: Subscribe to `/slam/odom`
- **Pose**: Subscribe to `/slam/pose`

## Supported SLAM Frameworks

| Framework | Status | Config File | Description |
|-----------|--------|-------------|-------------|
| VINS-Mono | Supported | `config/vins_mono.yaml` | Monocular visual-inertial SLAM |
| OpenVINS | Supported | `config/openvins.yaml` | Open-source VIO with MSCKF |
| ORB-SLAM3 | In Progress | `config/orbslam3.yaml` | Feature-based visual SLAM |
| Basalt | In Progress | `config/basalt.yaml` | Optical flow based VIO |

### Configuration Files

Each SLAM framework has its own configuration file in the `config/` directory:

- **`config/vins_mono.yaml`** - VINS-Mono parameters including camera intrinsics, IMU noise, and feature tracking settings
- **`config/openvins.yaml`** - OpenVINS MSCKF parameters and state estimation settings
- **`config/orbslam3.yaml`** - ORB-SLAM3 settings including ORB feature parameters and vocabulary path
- **`config/basalt.yaml`** - Basalt VIO optical flow and solver parameters

Example configuration usage:

```yaml
# config/pc_client_config.yaml
slam:
  type: "openvins"  # or "vins_mono", "orbslam3", "basalt"
  config_file: "config/openvins.yaml"
```

### Vocabulary Files

ORB-SLAM3 and VINS-Mono require vocabulary files for loop closure. These are not included in the repository due to their size. See `vocab/README.md` for download instructions.

```bash
# Download vocabulary files
./scripts/download_vocab.sh

# Or manually download ORB vocabulary
wget https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xzf ORBvoc.txt.tar.gz -C vocab/
```

## Documentation

### API Reference

The API documentation is automatically generated from source code comments using Doxygen.

- **Online**: [API Documentation](https://kcenon.github.io/vi_slam/) (GitHub Pages)
- **Local**: Generate locally with `doxygen Doxyfile`, then open `docs/api/html/index.html`

### Project Documents

- [Product Requirements Document (PRD)](docs/PRD.md)
- [Software Requirements Specification (SRS)](docs/SRS.md)
- [Software Design Specification (SDS)](docs/SDS.md)
- [Reference Documentation](docs/reference/)

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'feat: add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Troubleshooting

### Build Errors

**OpenCV not found:**
```bash
# Ubuntu
sudo apt-get install libopencv-dev

# macOS
brew install opencv
```

**Eigen3 not found:**
```bash
# Ubuntu
sudo apt-get install libeigen3-dev

# macOS
brew install eigen
```

### Runtime Errors

**Cannot connect to Android device:**
- Verify both devices are on the same network
- Check firewall settings
- Ensure server address is correctly configured in Android app

## Contact

For questions or issues, please open a GitHub issue or contact the maintainers.
