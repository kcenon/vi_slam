# Android Camera2 API Guide for VI-SLAM

## 1. 개요

Camera2 API는 Android 5.0 (API 21)부터 도입된 카메라 프레임워크로, 프레임 단위 캡처 제어와 정밀한 타임스탬프 획득이 가능하다.

### 왜 Camera2 API인가?

| 기능 | Camera (Legacy) | Camera2 |
|------|-----------------|---------|
| 프레임별 타임스탬프 | 제한적 | 지원 |
| 수동 노출 제어 | 제한적 | 완전 지원 |
| RAW 캡처 | 미지원 | 지원 |
| 고속 촬영 | 제한적 | 지원 |
| IMU 동기화 | 불가능 | 가능 |

## 2. Camera2 API 핵심 컴포넌트

```
┌─────────────────────────────────────────────────────────┐
│                    CameraManager                         │
│  (시스템 서비스, 카메라 장치 열거 및 열기)                 │
└───────────────────────┬─────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────┐
│                    CameraDevice                          │
│  (물리적 카메라 장치 표현)                                │
└───────────────────────┬─────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────┐
│               CameraCaptureSession                       │
│  (캡처 요청 처리를 위한 세션)                             │
└───────────────────────┬─────────────────────────────────┘
                        │
           ┌────────────┴────────────┐
           ▼                         ▼
┌──────────────────┐      ┌──────────────────┐
│  CaptureRequest  │      │   CaptureResult  │
│  (캡처 설정)      │      │   (캡처 메타데이터) │
└──────────────────┘      └──────────────────┘
```

## 3. 카메라 초기화

### 3.1 CameraManager 획득

```kotlin
// Kotlin
class CameraService(private val context: Context) {
    private val cameraManager: CameraManager by lazy {
        context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    }

    // 후면 카메라 ID 찾기
    fun findBackCamera(): String? {
        return cameraManager.cameraIdList.find { id ->
            val characteristics = cameraManager.getCameraCharacteristics(id)
            val facing = characteristics.get(CameraCharacteristics.LENS_FACING)
            facing == CameraCharacteristics.LENS_FACING_BACK
        }
    }
}
```

### 3.2 카메라 특성 확인

```kotlin
fun checkCameraCapabilities(cameraId: String) {
    val characteristics = cameraManager.getCameraCharacteristics(cameraId)

    // 하드웨어 지원 레벨 확인
    val hardwareLevel = characteristics.get(
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL
    )

    when (hardwareLevel) {
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY -> {
            // 기본 기능만 지원, 동기화 제한적
        }
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED -> {
            // 일부 고급 기능 지원
        }
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL -> {
            // 완전한 Camera2 기능 지원
        }
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3 -> {
            // YUV 재처리, RAW 캡처 등 최고 수준 지원
        }
    }

    // 타임스탬프 소스 확인 (VI-SLAM에 중요!)
    val timestampSource = characteristics.get(
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
    )

    when (timestampSource) {
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN -> {
            // 타임스탬프 소스 불명 - IMU 동기화 어려움
        }
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME -> {
            // SystemClock.elapsedRealtimeNanos()와 동일
            // IMU 타임스탬프와 동기화 가능!
        }
    }
}
```

### 3.3 카메라 열기

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

## 4. 캡처 세션 구성

### 4.1 Surface 설정

```kotlin
// 비디오 녹화용 Surface (MediaRecorder 또는 MediaCodec)
private lateinit var videoSurface: Surface

// 프레임 처리용 ImageReader
private lateinit var imageReader: ImageReader

fun setupSurfaces(width: Int, height: Int) {
    // ImageReader 설정 (YUV 포맷, 30fps 기준 버퍼 3개)
    imageReader = ImageReader.newInstance(
        width, height,
        ImageFormat.YUV_420_888,
        3  // maxImages
    )

    imageReader.setOnImageAvailableListener({ reader ->
        val image = reader.acquireLatestImage()
        image?.let {
            // 이미지 처리
            processImage(it)
            it.close()
        }
    }, backgroundHandler)
}
```

### 4.2 세션 생성

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

## 5. 캡처 요청 및 타임스탬프 획득

### 5.1 캡처 요청 설정

```kotlin
fun startPreview() {
    val captureRequestBuilder = cameraDevice?.createCaptureRequest(
        CameraDevice.TEMPLATE_RECORD
    ) ?: return

    // Surface 추가
    captureRequestBuilder.addTarget(imageReader.surface)
    captureRequestBuilder.addTarget(videoSurface)

    // VI-SLAM 최적화 설정

    // 1. 자동 노출 모드 (또는 수동 제어)
    captureRequestBuilder.set(
        CaptureRequest.CONTROL_AE_MODE,
        CaptureRequest.CONTROL_AE_MODE_ON
    )

    // 2. 자동 초점 모드
    captureRequestBuilder.set(
        CaptureRequest.CONTROL_AF_MODE,
        CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
    )

    // 3. OIS(광학 손떨림 보정) 비활성화 (VI-SLAM에서 권장)
    captureRequestBuilder.set(
        CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
        CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF
    )

    // 4. 비디오 안정화 비활성화
    captureRequestBuilder.set(
        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF
    )

    // 반복 캡처 요청
    captureSession?.setRepeatingRequest(
        captureRequestBuilder.build(),
        captureCallback,
        backgroundHandler
    )
}
```

### 5.2 캡처 콜백에서 타임스탬프 추출

```kotlin
private val captureCallback = object : CameraCaptureSession.CaptureCallback() {

    override fun onCaptureCompleted(
        session: CameraCaptureSession,
        request: CaptureRequest,
        result: TotalCaptureResult
    ) {
        // 센서 타임스탬프 (나노초, elapsedRealtimeNanos 기준)
        val sensorTimestamp = result.get(CaptureResult.SENSOR_TIMESTAMP)

        // 노출 시간 (나노초)
        val exposureTime = result.get(CaptureResult.SENSOR_EXPOSURE_TIME)

        // 프레임 지속 시간 (나노초)
        val frameDuration = result.get(CaptureResult.SENSOR_FRAME_DURATION)

        // Rolling Shutter Skew (나노초) - 첫 행과 마지막 행 사이 시간차
        val rollingShutterSkew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW)

        // 초점 거리 (mm)
        val focalLength = result.get(CaptureResult.LENS_FOCAL_LENGTH)

        // OIS 오프셋 (지원 기기만)
        val oisXShift = result.get(CaptureResult.OIS_DATA_X_SHIFT)
        val oisYShift = result.get(CaptureResult.OIS_DATA_Y_SHIFT)

        // 프레임 메타데이터 저장
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
        // 캡처 시작 시점 (프레임 노출 시작)
        // timestamp: 센서 타임스탬프 (나노초)
        // frameNumber: 프레임 시퀀스 번호
    }
}
```

## 6. 타임스탬프 동기화

### 6.1 카메라와 IMU 타임스탬프 비교

```kotlin
// 카메라 타임스탬프: CaptureResult.SENSOR_TIMESTAMP
// IMU 타임스탬프: SensorEvent.timestamp

// 둘 다 SystemClock.elapsedRealtimeNanos() 기준 (REALTIME 소스인 경우)

data class SynchronizedFrame(
    val frameTimestamp: Long,      // 나노초
    val imuReadings: List<ImuReading>,
    val imageData: ByteArray
)

data class ImuReading(
    val timestamp: Long,           // 나노초
    val accelerometer: FloatArray, // [ax, ay, az] m/s²
    val gyroscope: FloatArray      // [gx, gy, gz] rad/s
)

fun synchronizeData(
    frameTimestamp: Long,
    imuBuffer: List<ImuReading>
): SynchronizedFrame {
    // 프레임 시간 근처의 IMU 데이터 추출
    val relevantImu = imuBuffer.filter { imu ->
        val timeDiff = abs(imu.timestamp - frameTimestamp)
        timeDiff < 50_000_000L  // ±50ms 범위
    }

    return SynchronizedFrame(
        frameTimestamp = frameTimestamp,
        imuReadings = relevantImu,
        imageData = byteArrayOf()  // 실제 이미지 데이터
    )
}
```

### 6.2 타임스탬프 검증

```kotlin
fun verifyTimestampSource(cameraId: String): Boolean {
    val characteristics = cameraManager.getCameraCharacteristics(cameraId)
    val timestampSource = characteristics.get(
        CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
    )

    return timestampSource == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
}

// 기기별 동기화 지원 여부 확인
fun checkSynchronizationSupport(): SyncSupportLevel {
    val cameraSupport = verifyTimestampSource(findBackCamera() ?: "")

    return when {
        cameraSupport -> SyncSupportLevel.HARDWARE_SYNC
        else -> SyncSupportLevel.SOFTWARE_SYNC_ONLY
    }
}

enum class SyncSupportLevel {
    HARDWARE_SYNC,        // 하드웨어 동기화 지원
    SOFTWARE_SYNC_ONLY    // 소프트웨어 동기화만 가능
}
```

## 7. 비디오 인코딩

### 7.1 MediaCodec을 이용한 H.264 인코딩

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
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)  // 1초마다 키프레임
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
            // 인코딩된 데이터 처리
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

## 8. 주요 설정 값

### VI-SLAM 최적화 Camera2 설정

| 설정 | 권장값 | 이유 |
|------|-------|------|
| OIS | OFF | 캘리브레이션 유지 |
| Video Stabilization | OFF | 프레임 왜곡 방지 |
| Auto Focus | CONTINUOUS_VIDEO | 안정적 초점 |
| Auto Exposure | ON 또는 MANUAL | 상황에 따라 |
| Frame Rate | 30fps | 처리 속도와 균형 |
| Resolution | 720p-1080p | 처리 부하 고려 |

### 메타데이터 저장 항목

```kotlin
data class FrameMetadata(
    val frameNumber: Long,
    val timestamp: Long,           // 나노초
    val exposureTime: Long,        // 나노초
    val frameDuration: Long,       // 나노초
    val rollingShutterSkew: Long,  // 나노초
    val focalLength: Float,        // mm
    val focalLengthPixels: Float,  // 픽셀 (계산 필요)
    val sensorSensitivity: Int,    // ISO
    val oisXShift: Float?,         // OIS 보정값 (지원 시)
    val oisYShift: Float?
)
```

## 9. 참고 오픈소스 프로젝트

| 프로젝트 | 설명 | 링크 |
|---------|------|------|
| OpenCamera Sensors | 동기화 카메라/IMU 앱 | [GitHub](https://github.com/prime-slam/OpenCamera-Sensors) |
| VideoIMUCapture | OIS 데이터 포함 캡처 | [GitHub](https://github.com/DavidGillsjo/VideoIMUCapture-Android) |
| VIRec | 듀얼 카메라 + IMU + GPS | [GitHub](https://github.com/A3DV/VIRec) |

## References

- [Android Camera2 API Reference](https://developer.android.com/reference/android/hardware/camera2/package-summary)
- [MARS Logger Paper](https://arxiv.org/pdf/2001.00470)
- [Sensor Synchronization for Android Phone VI-SLAM](https://www.researchgate.net/publication/324929837_Sensor_Synchronization_for_Android_Phone_Tightly-Coupled_Visual-Inertial_SLAM)
