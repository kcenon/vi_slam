# VI-SLAM: Visual-Inertial SLAM System

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
cd pc_client
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Or use the build script:
```bash
./scripts/build_pc_client.sh Release
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

## Supported SLAM Frameworks

- VINS-Mono
- ORB-SLAM3 (planned)
- Basalt (planned)

## Documentation

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
