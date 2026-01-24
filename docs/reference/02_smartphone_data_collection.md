# Smartphone Camera/IMU Data Collection

## 1. Overview

Smartphones have built-in high-quality cameras and IMU sensors, making them a viable platform for VI-SLAM research and development.

### Smartphone Sensor Characteristics

| Sensor | Typical Specifications | Purpose |
|--------|------------------------|---------|
| Camera | 30-60Hz, 1080p+ | Visual feature extraction |
| Accelerometer | 100-500Hz | Linear acceleration measurement |
| Gyroscope | 100-500Hz | Angular velocity measurement |
| Magnetometer | 50-100Hz | Direction correction (optional) |

## 2. Major Open-Source Data Collection Apps

### 2.1 OpenCamera Sensors (Android)

**GitHub**: [prime-slam/OpenCamera-Sensors](https://github.com/prime-slam/OpenCamera-Sensors)

#### Features
- Open-source app based on Open Camera
- Synchronized video and IMU data recording
- Remote control via network supported
- Multi-smartphone simultaneous recording possible

#### Collected Data
- Video: H.264/MP4
- IMU: Accelerometer, gyroscope, magnetometer
- Timestamps: Synchronized to the same clock

#### Cautions
- Camera2 API support required
- Not all devices support synchronized timestamps
- Check device compatibility in settings

### 2.2 MARS Logger (Mobile AR Sensor Logger)

**GitHub**: [OSUPCVLab/mobile-ar-sensor-logger](https://github.com/OSUPCVLab/mobile-ar-sensor-logger)

#### Features
- Supports both Android and iOS
- Optimized for AR sensor data collection
- Best synchronization between camera and IMU

#### Performance
- Camera: ~30Hz
- IMU: 50-100Hz
- Recording time: 10+ minutes possible

#### Android Implementation
```
- Uses Camera2 API
- OpenGL ES (GLSurfaceView)
- Video encoding via MediaCodec/MediaMuxer
- IMU recording in background HandlerThread
```

#### iOS Implementation
```
- Uses AVCaptureVideoDataOutput
- Video saving via AVAssetWriter
- Metadata saved in CSV files
```

#### Output Files
- `video.mp4`: H.264 encoded video
- `movie_metadata.csv`: Per-frame timestamp, focal length, exposure time
- `gyro_accel.csv`: IMU data
- `imu_timestamps.txt`: IMU timestamps

### 2.3 VideoIMUCapture-Android

**GitHub**: [DavidGillsjo/VideoIMUCapture-Android](https://github.com/DavidGillsjo/VideoIMUCapture-Android)

#### Features
- Designed for SLAM/SfM research
- OIS (Optical Image Stabilization) data provided
- DVS (Digital Video Stabilization) distinction

#### Performance
- Camera: ~30Hz
- IMU: ~100Hz
- Same clock synchronization

#### Output Format
- Video: H.264/MP4
- Metadata: Protocol Buffers 3 format

#### OIS-Related Cautions
Most modern smartphones have OIS, so camera parameters vary per frame:
- OIS cannot be disabled on many Android devices
- Only some devices provide lens movement data
- May cause issues in 3D reconstruction

### 2.4 Sensors-Data-Logger

**GitHub**: [PyojinKim/Sensors-Data-Logger](https://github.com/PyojinKim/Sensors-Data-Logger)

#### Features
- Android only (Java)
- IMU + WiFi data recording
- Offline usable as text files

#### Supported Sensors
- Accelerometer
- Gyroscope
- Magnetometer
- WiFi signal strength

## 3. Data Collection Considerations

### 3.1 Timestamp Synchronization

The most critical factor for VI-SLAM performance:

```
Issues:
- Camera and IMU clock mismatch
- Trigger delay
- Transmission delay

Solutions:
- Hardware synchronization (best)
- Software synchronization (alternative)
- Online temporal calibration (post-processing)
```

### 3.2 Recommended Collection Settings

| Parameter | Recommended Value | Notes |
|-----------|-------------------|-------|
| Camera FPS | 30Hz | 20-60Hz range |
| IMU sampling | 200Hz | Minimum 100Hz |
| Resolution | 720p-1080p | Trade-off with processing speed |
| Exposure time | Keep low | Minimize motion blur |
| Auto exposure | Fixed recommended | Consistent image quality |

### 3.3 Data Quality Checklist

- [ ] Consistent lighting environment
- [ ] Scene with sufficient texture
- [ ] Minimize motion blur
- [ ] Avoid rapid rotation movements
- [ ] Verify IMU data continuity
- [ ] Verify timestamps are monotonically increasing

## 4. Platform-specific APIs

### Android

```java
// Camera2 API - Timestamp acquisition
CameraCaptureSession.CaptureCallback callback =
    new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureCompleted(CameraCaptureSession session,
                CaptureRequest request, TotalCaptureResult result) {
            Long timestamp = result.get(CaptureResult.SENSOR_TIMESTAMP);
        }
    };

// SensorManager - IMU data
sensorManager.registerListener(listener,
    sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
    SensorManager.SENSOR_DELAY_FASTEST);
```

### iOS

```swift
// AVCaptureVideoDataOutput
func captureOutput(_ output: AVCaptureOutput,
                   didOutput sampleBuffer: CMSampleBuffer,
                   from connection: AVCaptureConnection) {
    let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
}

// CMMotionManager - IMU data
motionManager.startDeviceMotionUpdates(to: .main) { motion, error in
    let attitude = motion?.attitude
    let rotationRate = motion?.rotationRate
    let userAcceleration = motion?.userAcceleration
}
```

## 5. Real-time Streaming vs Offline Recording

### Offline Recording
- **Advantages**: Highest quality data, high synchronization accuracy
- **Disadvantages**: No real-time processing, storage space required
- **Use cases**: Research, algorithm development, dataset building

### Real-time Streaming
- **Advantages**: Immediate feedback, real-time applications possible
- **Disadvantages**: Network delay, potential data loss
- **Use cases**: Real-time AR/VR, robot control

## References

- [Mobile AR Sensor Logger Paper (IEEE 2020)](https://ieeexplore.ieee.org/document/8956816/)
- [OpenCamera Sensors GitHub](https://github.com/prime-slam/OpenCamera-Sensors)
- [VideoIMUCapture-Android GitHub](https://github.com/DavidGillsjo/VideoIMUCapture-Android)
- [MARS Logger arXiv](https://arxiv.org/pdf/2001.00470)
