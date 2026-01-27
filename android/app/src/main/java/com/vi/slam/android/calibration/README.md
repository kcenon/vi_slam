# Camera Calibration Module

This module implements camera intrinsic and camera-IMU extrinsic calibration for the VI-SLAM Android app.

## Features

- **Camera Intrinsic Calibration**
  - Checkerboard pattern detection
  - Pinhole and fisheye camera models
  - Multi-view calibration optimization
  - Reprojection error < 0.5 pixel
  - Coverage-based capture guidance (3x3 grid)

- **Calibration Data Models**
  - `IntrinsicCalibResult`: Camera intrinsic parameters
  - `CalibrationCapture`: Single calibration frame
  - `CoverageMap`: Calibration progress tracking

## Dependencies

### OpenCV Android SDK

The calibration module requires OpenCV Android SDK. Two options for adding OpenCV:

#### Option 1: Manual Download (Recommended for Production)

1. Download OpenCV Android SDK from [opencv.org](https://opencv.org/releases/)
2. Extract and copy to `android/app/libs/opencv-4.8.1/`
3. Add to `build.gradle.kts`:

```kotlin
dependencies {
    implementation(fileTree(mapOf("dir" to "libs/opencv-4.8.1", "include" to listOf("*.aar"))))
}
```

#### Option 2: JitPack Wrapper (Quick Start)

Add JitPack repository and dependency:

```kotlin
// In settings.gradle.kts or build.gradle.kts
repositories {
    maven { url = uri("https://jitpack.io") }
}

dependencies {
    implementation("com.github.QuickBirdEng:opencv-android:4.5.3.0")
}
```

**Note**: Currently, OpenCV dependency is not included to avoid build issues. See [Issue #TBD] for OpenCV integration tracking.

## Usage

### Basic Intrinsic Calibration

```kotlin
val calibrator = IntrinsicCalibrator()

// Start calibration
val config = IntrinsicCalibConfig(
    targetType = CalibrationTargetType.CHECKERBOARD,
    targetSize = TargetSize(width = 9, height = 6, squareSize = 0.025), // 9x6 board, 25mm squares
    cameraModel = CameraModelType.PINHOLE,
    minCaptures = 20,
    coverageRegions = 9
)
calibrator.startIntrinsicCalibration(config)

// Capture calibration images
val imageData: ByteArray = ... // Grayscale camera frame
val result = calibrator.captureCalibrationImage(imageData, width = 640, height = 480)

if (result.isSuccess) {
    val capture = result.getOrNull()
    if (capture?.cornersDetected == true) {
        println("Corners detected in region ${capture.coverageRegion}")
    }
}

// Check progress
val coverageMap = calibrator.getCoverageMap()
println("Coverage: ${coverageMap.progress * 100}%")

// Finish calibration
when (calibrator.getCoverageMap().coveredRegions >= 9) {
    true -> {
        val calibResult = calibrator.finishIntrinsicCalibration()
        if (calibResult.isSuccess) {
            val params = calibResult.getOrNull()
            println("Calibration complete:")
            println("  fx = ${params?.fx}, fy = ${params?.fy}")
            println("  cx = ${params?.cx}, cy = ${params?.cy}")
            println("  Reprojection error = ${params?.reprojectionError} pixels")
        }
    }
    false -> println("Need more coverage")
}
```

## Implementation Details

### Checkerboard Detection

- Uses OpenCV's `findChessboardCorners()` with adaptive thresholding
- Sub-pixel corner refinement with `cornerSubPix()`
- Corner detection criteria: 30 iterations, 0.001 epsilon

### Coverage Tracking

The image is divided into a 3x3 grid (9 regions). Calibration requires at least one capture from each region to ensure good coverage across the entire image.

```
+---+---+---+
| 0 | 1 | 2 |
+---+---+---+
| 3 | 4 | 5 |
+---+---+---+
| 6 | 7 | 8 |
+---+---+---+
```

### Calibration Algorithm

**Pinhole Model**:
- Uses `Calib3d.calibrateCamera()`
- Estimates: fx, fy, cx, cy, distortion coefficients (k1, k2, p1, p2, k3)
- Fixes principal point during optimization

**Fisheye Model**:
- Uses `Calib3d.fisheye_calibrate()`
- Estimates: fx, fy, cx, cy, fisheye distortion (k1, k2, k3, k4)
- Recomputes extrinsics, checks conditioning, fixes skew

## Testing

Run unit tests:

```bash
cd android
./gradlew :app:testDebugUnitTest --tests "com.vi.slam.android.calibration.*"
```

**Note**: Tests requiring OpenCV will be skipped if OpenCV is not available in the test environment.

## Future Work

- [ ] Extrinsic calibration implementation (#88)
- [ ] Time offset estimation (#89)
- [ ] Calibration export (YAML/JSON) (#90)
- [ ] Calibration verification (#91)
- [ ] AprilGrid target support
- [ ] Real-time corner detection visualization
- [ ] Calibration quality scoring

## References

- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Kalibr Documentation](https://github.com/ethz-asl/kalibr/wiki)
- [Camera Calibration Best Practices](https://arxiv.org/abs/1912.02858)

## Related Issues

- Epic: #22 - Calibration Module
- This implementation: #87 - Camera Intrinsic Calibration
- Next: #88 - Camera-IMU Extrinsic Calibration
