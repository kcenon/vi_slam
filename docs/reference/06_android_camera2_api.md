# Android Camera2 API Guide for VI-SLAM

## 1. Overview

Camera2 API is a camera framework introduced in Android 5.0 (API 21), enabling frame-by-frame capture control and precise timestamp acquisition.

### Why Camera2 API?

| Feature | Camera (Legacy) | Camera2 |
|------|-----------------|---------|
| Per-frame timestamp | Limited | Supported |
| Manual exposure control | Limited | Fully supported |
| RAW capture | Not supported | Supported |
| High-speed capture | Limited | Supported |
| IMU synchronization | Not possible | Possible |

## 2. Camera2 API Core Components

```
┌─────────────────────────────────────────────────────────┐
│                    CameraManager                         │
│  (System service, camera device enumeration and opening) │
└───────────────────────┬─────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────┐
│                    CameraDevice                          │
│  (Physical camera device representation)                 │
└───────────────────────┬─────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────┐
│               CameraCaptureSession                       │
│  (Session for processing capture requests)               │
└───────────────────────┬─────────────────────────────────┘
                        │
           ┌────────────┴────────────┐
           ▼                         ▼
┌──────────────────┐      ┌──────────────────┐
│  CaptureRequest  │      │   CaptureResult  │
│  (Capture settings)│    │   (Capture metadata) │
└──────────────────┘      └──────────────────┘
```

## 3. Camera Initialization

### 3.1 Obtaining CameraManager

```kotlin
// Kotlin
class CameraService(private val context: Context) {
    private val cameraManager: CameraManager by lazy {
        context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    }

    // Find back camera ID
    fun findBackCamera(): String? {
        return cameraManager.cameraIdList.find { id ->
            val characteristics = cameraManager.getCameraCharacteristics(id)
            val facing = characteristics.get(CameraCharacteristics.LENS_FACING)
            facing == CameraCharacteristics.LENS_FACING_BACK
        }
    }
}
```

### 3.2 Checking Camera Capabilities

```kotlin
fun checkCameraCapabilities(cameraId: String) {
    val characteristics = cameraManager.getCameraCharacteristics(cameraId)

    // Check hardware support level
    val hardwareLevel = characteristics.get(
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL
    )

    when (hardwareLevel) {
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY -> {
            // Basic features only, limited synchronization
        }
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED -> {
            // Some advanced features supported
        }
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL -> {
            // Full Camera2 features supported
        }
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3 -> {
            // Highest level support including YUV reprocessing, RAW capture
        }
    }

    // Check timestamp source (important for VI-SLAM!)
    val timestampSource = characteristics.get(
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
    )

    when (timestampSource) {
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN -> {
            // Unknown timestamp source - IMU synchronization difficult
        }
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME -> {
            // Same as SystemClock.elapsedRealtimeNanos()
            // Synchronizable with IMU timestamps!
        }
    }
}
```

### 3.3 Opening the Camera

```kotlin
private var cameraDevice: CameraDevice? = null

private val cameraStateCallback = object : CameraDevice.StateCallback() {
    override fun onOpened(camera: CameraDevice) {
        cameraDevice = camera
        createCaptureSession()
    }

    override fun onDisconnected(camera: CameraDevice) {
        camera.close()
        cameraDevice = null
    }

    override fun onError(camera: CameraDevice, error: Int) {
        camera.close()
        cameraDevice = null
        Log.e(TAG, "Camera error: $error")
    }
}

fun openCamera(cameraId: String) {
    if (ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA)
        == PackageManager.PERMISSION_GRANTED) {
        cameraManager.openCamera(cameraId, cameraStateCallback, backgroundHandler)
    }
}
```

## 4. Capture Session Configuration

### 4.1 Surface Setup

```kotlin
// Surface for video recording (MediaRecorder or MediaCodec)
private lateinit var videoSurface: Surface

// ImageReader for frame processing
private lateinit var imageReader: ImageReader

fun setupSurfaces(width: Int, height: Int) {
    // ImageReader setup (YUV format, 3 buffers for 30fps)
    imageReader = ImageReader.newInstance(
        width, height,
        ImageFormat.YUV_420_888,
        3  // maxImages
    )

    imageReader.setOnImageAvailableListener({ reader ->
        val image = reader.acquireLatestImage()
        image?.let {
            // Process image
            processImage(it)
            it.close()
        }
    }, backgroundHandler)
}
```

### 4.2 Session Creation

```kotlin
private var captureSession: CameraCaptureSession? = null

fun createCaptureSession() {
    val surfaces = listOf(imageReader.surface, videoSurface)

    cameraDevice?.createCaptureSession(
        surfaces,
        object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(session: CameraCaptureSession) {
                captureSession = session
                startPreview()
            }

            override fun onConfigureFailed(session: CameraCaptureSession) {
                Log.e(TAG, "Capture session configuration failed")
            }
        },
        backgroundHandler
    )
}
```

## 5. Capture Request and Timestamp Acquisition

### 5.1 Capture Request Configuration

```kotlin
fun startPreview() {
    val captureRequestBuilder = cameraDevice?.createCaptureRequest(
        CameraDevice.TEMPLATE_RECORD
    ) ?: return

    // Add surfaces
    captureRequestBuilder.addTarget(imageReader.surface)
    captureRequestBuilder.addTarget(videoSurface)

    // VI-SLAM optimization settings

    // 1. Auto exposure mode (or manual control)
    captureRequestBuilder.set(
        CaptureRequest.CONTROL_AE_MODE,
        CaptureRequest.CONTROL_AE_MODE_ON
    )

    // 2. Auto focus mode
    captureRequestBuilder.set(
        CaptureRequest.CONTROL_AF_MODE,
        CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
    )

    // 3. Disable OIS (Optical Image Stabilization) - recommended for VI-SLAM
    captureRequestBuilder.set(
        CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
        CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF
    )

    // 4. Disable video stabilization
    captureRequestBuilder.set(
        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF
    )

    // Repeating capture request
    captureSession?.setRepeatingRequest(
        captureRequestBuilder.build(),
        captureCallback,
        backgroundHandler
    )
}
```

### 5.2 Extracting Timestamps from Capture Callback

```kotlin
private val captureCallback = object : CameraCaptureSession.CaptureCallback() {

    override fun onCaptureCompleted(
        session: CameraCaptureSession,
        request: CaptureRequest,
        result: TotalCaptureResult
    ) {
        // Sensor timestamp (nanoseconds, elapsedRealtimeNanos basis)
        val sensorTimestamp = result.get(CaptureResult.SENSOR_TIMESTAMP)

        // Exposure time (nanoseconds)
        val exposureTime = result.get(CaptureResult.SENSOR_EXPOSURE_TIME)

        // Frame duration (nanoseconds)
        val frameDuration = result.get(CaptureResult.SENSOR_FRAME_DURATION)

        // Rolling Shutter Skew (nanoseconds) - time difference between first and last row
        val rollingShutterSkew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW)

        // Focal length (mm)
        val focalLength = result.get(CaptureResult.LENS_FOCAL_LENGTH)

        // OIS offset (supported devices only)
        val oisXShift = result.get(CaptureResult.OIS_DATA_X_SHIFT)
        val oisYShift = result.get(CaptureResult.OIS_DATA_Y_SHIFT)

        // Save frame metadata
        saveFrameMetadata(
            timestamp = sensorTimestamp ?: 0L,
            exposureTime = exposureTime ?: 0L,
            frameDuration = frameDuration ?: 0L,
            rollingShutterSkew = rollingShutterSkew ?: 0L,
            focalLength = focalLength ?: 0f
        )
    }

    override fun onCaptureStarted(
        session: CameraCaptureSession,
        request: CaptureRequest,
        timestamp: Long,
        frameNumber: Long
    ) {
        // Capture start point (frame exposure start)
        // timestamp: sensor timestamp (nanoseconds)
        // frameNumber: frame sequence number
    }
}
```

## 6. Timestamp Synchronization

### 6.1 Camera and IMU Timestamp Comparison

```kotlin
// Camera timestamp: CaptureResult.SENSOR_TIMESTAMP
// IMU timestamp: SensorEvent.timestamp

// Both use SystemClock.elapsedRealtimeNanos() basis (when REALTIME source)

data class SynchronizedFrame(
    val frameTimestamp: Long,      // nanoseconds
    val imuReadings: List<ImuReading>,
    val imageData: ByteArray
)

data class ImuReading(
    val timestamp: Long,           // nanoseconds
    val accelerometer: FloatArray, // [ax, ay, az] m/s²
    val gyroscope: FloatArray      // [gx, gy, gz] rad/s
)

fun synchronizeData(
    frameTimestamp: Long,
    imuBuffer: List<ImuReading>
): SynchronizedFrame {
    // Extract IMU data near frame time
    val relevantImu = imuBuffer.filter { imu ->
        val timeDiff = abs(imu.timestamp - frameTimestamp)
        timeDiff < 50_000_000L  // ±50ms range
    }

    return SynchronizedFrame(
        frameTimestamp = frameTimestamp,
        imuReadings = relevantImu,
        imageData = byteArrayOf()  // Actual image data
    )
}
```

### 6.2 Timestamp Verification

```kotlin
fun verifyTimestampSource(cameraId: String): Boolean {
    val characteristics = cameraManager.getCameraCharacteristics(cameraId)
    val timestampSource = characteristics.get(
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
    )

    return timestampSource == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
}

// Check device synchronization support
fun checkSynchronizationSupport(): SyncSupportLevel {
    val cameraSupport = verifyTimestampSource(findBackCamera() ?: "")

    return when {
        cameraSupport -> SyncSupportLevel.HARDWARE_SYNC
        else -> SyncSupportLevel.SOFTWARE_SYNC_ONLY
    }
}

enum class SyncSupportLevel {
    HARDWARE_SYNC,        // Hardware synchronization supported
    SOFTWARE_SYNC_ONLY    // Software synchronization only
}
```

## 7. Video Encoding

### 7.1 H.264 Encoding with MediaCodec

```kotlin
class VideoEncoder(
    private val width: Int,
    private val height: Int,
    private val frameRate: Int = 30,
    private val bitRate: Int = 4_000_000  // 4 Mbps
) {
    private var mediaCodec: MediaCodec? = null
    private var inputSurface: Surface? = null

    fun initialize(): Surface {
        val format = MediaFormat.createVideoFormat(
            MediaFormat.MIMETYPE_VIDEO_AVC,  // H.264
            width, height
        ).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT,
                MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, bitRate)
            setInteger(MediaFormat.KEY_FRAME_RATE, frameRate)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)  // Keyframe every 1 second
        }

        mediaCodec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
        mediaCodec?.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        inputSurface = mediaCodec?.createInputSurface()
        mediaCodec?.start()

        return inputSurface!!
    }

    fun getEncodedData(): ByteBuffer? {
        val bufferInfo = MediaCodec.BufferInfo()
        val outputBufferIndex = mediaCodec?.dequeueOutputBuffer(bufferInfo, 0) ?: -1

        if (outputBufferIndex >= 0) {
            val outputBuffer = mediaCodec?.getOutputBuffer(outputBufferIndex)
            // Process encoded data
            mediaCodec?.releaseOutputBuffer(outputBufferIndex, false)
            return outputBuffer
        }
        return null
    }

    fun release() {
        mediaCodec?.stop()
        mediaCodec?.release()
        inputSurface?.release()
    }
}
```

## 8. Key Configuration Values

### VI-SLAM Optimized Camera2 Settings

| Setting | Recommended Value | Reason |
|------|-------|------|
| OIS | OFF | Maintain calibration |
| Video Stabilization | OFF | Prevent frame distortion |
| Auto Focus | CONTINUOUS_VIDEO | Stable focus |
| Auto Exposure | ON or MANUAL | Depends on situation |
| Frame Rate | 30fps | Balance with processing speed |
| Resolution | 720p-1080p | Consider processing load |

### Metadata Fields to Save

```kotlin
data class FrameMetadata(
    val frameNumber: Long,
    val timestamp: Long,           // nanoseconds
    val exposureTime: Long,        // nanoseconds
    val frameDuration: Long,       // nanoseconds
    val rollingShutterSkew: Long,  // nanoseconds
    val focalLength: Float,        // mm
    val focalLengthPixels: Float,  // pixels (calculation needed)
    val sensorSensitivity: Int,    // ISO
    val oisXShift: Float?,         // OIS correction value (if supported)
    val oisYShift: Float?
)
```

## 9. Reference Open Source Projects

| Project | Description | Link |
|---------|------|------|
| OpenCamera Sensors | Synchronized camera/IMU app | [GitHub](https://github.com/prime-slam/OpenCamera-Sensors) |
| VideoIMUCapture | Capture with OIS data | [GitHub](https://github.com/DavidGillsjo/VideoIMUCapture-Android) |
| VIRec | Dual camera + IMU + GPS | [GitHub](https://github.com/A3DV/VIRec) |

## References

- [Android Camera2 API Reference](https://developer.android.com/reference/android/hardware/camera2/package-summary)
- [MARS Logger Paper](https://arxiv.org/pdf/2001.00470)
- [Sensor Synchronization for Android Phone VI-SLAM](https://www.researchgate.net/publication/324929837_Sensor_Synchronization_for_Android_Phone_Tightly-Coupled_Visual-Inertial_SLAM)
