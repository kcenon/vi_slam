# VI-SLAM Data Collection App Development Guide

## 1. 개요

이 문서는 스마트폰에서 VI-SLAM용 카메라/IMU 데이터를 수집하는 앱 개발 가이드이다.

### 개발 목표

1. 카메라와 IMU 데이터의 동기화된 수집
2. 실시간 또는 준실시간 PC 스트리밍
3. 로컬 저장 옵션
4. 캘리브레이션 데이터 포함

## 2. 앱 아키텍처

### 2.1 전체 구조

```
┌─────────────────────────────────────────────────────────────┐
│                        Application                           │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │     UI      │  │  Settings   │  │    Calibration      │ │
│  │   Layer     │  │  Manager    │  │      Wizard         │ │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘ │
│         │                │                     │            │
├─────────┴────────────────┴─────────────────────┴────────────┤
│                      Core Services                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Camera    │  │    IMU      │  │   Synchronization   │ │
│  │   Service   │  │   Service   │  │      Service        │ │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘ │
│         │                │                     │            │
├─────────┴────────────────┴─────────────────────┴────────────┤
│                      Data Layer                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Local     │  │   Network   │  │      Buffer         │ │
│  │   Storage   │  │   Streamer  │  │      Manager        │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 모듈 구성

```
app/
├── src/main/
│   ├── java/com/vislam/
│   │   ├── MainActivity.kt
│   │   ├── ui/
│   │   │   ├── PreviewFragment.kt
│   │   │   ├── SettingsFragment.kt
│   │   │   └── CalibrationFragment.kt
│   │   ├── camera/
│   │   │   ├── CameraService.kt
│   │   │   ├── CameraConfig.kt
│   │   │   └── FrameProcessor.kt
│   │   ├── imu/
│   │   │   ├── ImuService.kt
│   │   │   ├── ImuConfig.kt
│   │   │   └── ImuProcessor.kt
│   │   ├── sync/
│   │   │   ├── TimestampSynchronizer.kt
│   │   │   ├── DataAligner.kt
│   │   │   └── SyncValidator.kt
│   │   ├── storage/
│   │   │   ├── LocalRecorder.kt
│   │   │   ├── FileManager.kt
│   │   │   └── DataExporter.kt
│   │   ├── network/
│   │   │   ├── StreamingService.kt
│   │   │   ├── UdpSender.kt
│   │   │   └── WebRtcClient.kt
│   │   ├── calibration/
│   │   │   ├── CameraCalibrator.kt
│   │   │   ├── ImuCalibrator.kt
│   │   │   └── ExtrinsicsEstimator.kt
│   │   └── util/
│   │       ├── PermissionManager.kt
│   │       ├── TimestampUtils.kt
│   │       └── Constants.kt
│   └── res/
│       ├── layout/
│       └── values/
└── build.gradle
```

## 3. 핵심 컴포넌트 구현

### 3.1 메인 서비스 (Foreground Service)

```kotlin
class DataCollectionService : Service() {

    private lateinit var cameraService: CameraService
    private lateinit var imuService: ImuService
    private lateinit var synchronizer: TimestampSynchronizer
    private lateinit var recorder: LocalRecorder
    private lateinit var streamer: StreamingService

    private val serviceScope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    private var isRecording = false
    private var isStreaming = false

    override fun onCreate() {
        super.onCreate()

        cameraService = CameraService(this)
        imuService = ImuService(this)
        synchronizer = TimestampSynchronizer()
        recorder = LocalRecorder(this)
        streamer = StreamingService()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START_RECORDING -> startRecording()
            ACTION_STOP_RECORDING -> stopRecording()
            ACTION_START_STREAMING -> startStreaming(intent)
            ACTION_STOP_STREAMING -> stopStreaming()
        }
        return START_STICKY
    }

    private fun startRecording() {
        if (isRecording) return

        startForeground(NOTIFICATION_ID, createNotification("Recording..."))

        // 초기화
        cameraService.initialize()
        imuService.initialize()

        // 데이터 콜백 설정
        setupDataCallbacks()

        // 시작
        imuService.start(targetFrequency = 200.0)
        cameraService.start()

        isRecording = true
    }

    private fun setupDataCallbacks() {
        // IMU 데이터 콜백
        imuService.onImuData = { reading ->
            if (isRecording) {
                recorder.writeImu(reading)
            }
            if (isStreaming) {
                streamer.sendImu(reading)
            }
        }

        // 카메라 프레임 콜백
        cameraService.onFrameCaptured = { frame, metadata ->
            // 동기화된 IMU 데이터 매칭
            val imuData = synchronizer.matchImuToFrame(
                metadata.timestamp,
                imuService.getRecentData()
            )

            if (isRecording) {
                recorder.writeFrame(frame, metadata, imuData)
            }
            if (isStreaming) {
                streamer.sendFrame(frame, metadata.timestamp)
            }
        }
    }

    private fun stopRecording() {
        cameraService.stop()
        imuService.stop()
        recorder.finalize()
        isRecording = false
        stopForeground(STOP_FOREGROUND_REMOVE)
    }

    private fun startStreaming(intent: Intent) {
        val serverAddress = intent.getStringExtra(EXTRA_SERVER_ADDRESS) ?: return
        val videoPort = intent.getIntExtra(EXTRA_VIDEO_PORT, 8080)
        val imuPort = intent.getIntExtra(EXTRA_IMU_PORT, 5005)

        streamer.configure(serverAddress, videoPort, imuPort)
        streamer.connect()
        isStreaming = true
    }

    private fun stopStreaming() {
        streamer.disconnect()
        isStreaming = false
    }

    override fun onDestroy() {
        serviceScope.cancel()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    companion object {
        const val ACTION_START_RECORDING = "start_recording"
        const val ACTION_STOP_RECORDING = "stop_recording"
        const val ACTION_START_STREAMING = "start_streaming"
        const val ACTION_STOP_STREAMING = "stop_streaming"
        const val EXTRA_SERVER_ADDRESS = "server_address"
        const val EXTRA_VIDEO_PORT = "video_port"
        const val EXTRA_IMU_PORT = "imu_port"
        private const val NOTIFICATION_ID = 1001
    }
}
```

### 3.2 설정 관리

```kotlin
data class AppConfig(
    // Camera settings
    val cameraResolution: Resolution = Resolution.HD_720P,
    val cameraFps: Int = 30,
    val disableOis: Boolean = true,
    val disableVideoStabilization: Boolean = true,
    val fixedExposure: Boolean = false,
    val exposureTimeNs: Long = 8_333_333,  // 1/120s

    // IMU settings
    val imuFrequencyHz: Double = 200.0,
    val useUncalibratedSensors: Boolean = true,
    val recordMagnetometer: Boolean = false,

    // Storage settings
    val saveRawFrames: Boolean = false,
    val videoCodec: VideoCodec = VideoCodec.H264,
    val videoBitrate: Int = 4_000_000,

    // Streaming settings
    val streamingEnabled: Boolean = false,
    val serverAddress: String = "",
    val videoStreamingProtocol: StreamingProtocol = StreamingProtocol.WEBRTC,
    val imuStreamingProtocol: StreamingProtocol = StreamingProtocol.UDP
)

enum class Resolution(val width: Int, val height: Int) {
    VGA_480P(640, 480),
    HD_720P(1280, 720),
    FHD_1080P(1920, 1080)
}

enum class VideoCodec { H264, H265 }

enum class StreamingProtocol { WEBRTC, RTSP, UDP, TCP }

class ConfigManager(private val context: Context) {
    private val prefs = context.getSharedPreferences("vislam_config", Context.MODE_PRIVATE)
    private val gson = Gson()

    fun saveConfig(config: AppConfig) {
        prefs.edit()
            .putString("config", gson.toJson(config))
            .apply()
    }

    fun loadConfig(): AppConfig {
        val json = prefs.getString("config", null)
        return if (json != null) {
            gson.fromJson(json, AppConfig::class.java)
        } else {
            AppConfig()
        }
    }
}
```

### 3.3 UI 미리보기

```kotlin
class PreviewFragment : Fragment() {

    private var _binding: FragmentPreviewBinding? = null
    private val binding get() = _binding!!

    private lateinit var cameraService: CameraService

    private var isRecording = false
    private var recordingStartTime: Long = 0

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentPreviewBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        setupPreview()
        setupControls()
        observeStats()
    }

    private fun setupPreview() {
        // Camera preview surface 설정
        binding.previewView.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                cameraService.setPreviewSurface(holder.surface)
            }

            override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {}
            override fun surfaceDestroyed(holder: SurfaceHolder) {}
        })
    }

    private fun setupControls() {
        binding.btnRecord.setOnClickListener {
            if (isRecording) {
                stopRecording()
            } else {
                startRecording()
            }
        }

        binding.btnSettings.setOnClickListener {
            findNavController().navigate(R.id.action_preview_to_settings)
        }
    }

    private fun startRecording() {
        val intent = Intent(requireContext(), DataCollectionService::class.java).apply {
            action = DataCollectionService.ACTION_START_RECORDING
        }
        requireContext().startForegroundService(intent)

        isRecording = true
        recordingStartTime = SystemClock.elapsedRealtime()
        updateRecordingUI()
    }

    private fun stopRecording() {
        val intent = Intent(requireContext(), DataCollectionService::class.java).apply {
            action = DataCollectionService.ACTION_STOP_RECORDING
        }
        requireContext().startService(intent)

        isRecording = false
        updateRecordingUI()
    }

    private fun updateRecordingUI() {
        binding.btnRecord.apply {
            setImageResource(
                if (isRecording) R.drawable.ic_stop else R.drawable.ic_record
            )
        }

        binding.recordingIndicator.visibility =
            if (isRecording) View.VISIBLE else View.GONE
    }

    private fun observeStats() {
        // 실시간 통계 표시
        viewLifecycleOwner.lifecycleScope.launch {
            while (true) {
                if (isRecording) {
                    val elapsed = SystemClock.elapsedRealtime() - recordingStartTime
                    binding.tvRecordingTime.text = formatDuration(elapsed)

                    // IMU/Camera 상태 표시
                    // binding.tvImuRate.text = "${imuService.getCurrentRate()} Hz"
                    // binding.tvFrameRate.text = "${cameraService.getCurrentFps()} fps"
                }
                delay(100)
            }
        }
    }

    private fun formatDuration(millis: Long): String {
        val seconds = millis / 1000
        val minutes = seconds / 60
        val hours = minutes / 60
        return String.format("%02d:%02d:%02d", hours, minutes % 60, seconds % 60)
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
```

## 4. 캘리브레이션 워크플로우

### 4.1 카메라 캘리브레이션

```kotlin
class CameraCalibrator(private val context: Context) {

    private val detector = ChessboardDetector(9, 6)  // 9x6 체커보드
    private val imagePoints = mutableListOf<Array<PointF>>()
    private val objectPoints = mutableListOf<Array<Point3F>>()

    private val squareSize = 0.025f  // 25mm

    data class CalibrationResult(
        val fx: Double,
        val fy: Double,
        val cx: Double,
        val cy: Double,
        val distortionCoeffs: DoubleArray,  // [k1, k2, p1, p2, k3]
        val reprojectionError: Double,
        val imageWidth: Int,
        val imageHeight: Int
    )

    fun processFrame(frame: Mat): Boolean {
        val corners = detector.findCorners(frame)
        if (corners != null) {
            imagePoints.add(corners)
            objectPoints.add(generateObjectPoints())
            return true
        }
        return false
    }

    fun getCollectedFrameCount(): Int = imagePoints.size

    fun calibrate(imageWidth: Int, imageHeight: Int): CalibrationResult? {
        if (imagePoints.size < 10) {
            return null  // 최소 10장 필요
        }

        // OpenCV calibrateCamera 호출
        val cameraMatrix = Mat.eye(3, 3, CvType.CV_64F)
        val distCoeffs = Mat.zeros(5, 1, CvType.CV_64F)
        val rvecs = mutableListOf<Mat>()
        val tvecs = mutableListOf<Mat>()

        val error = Calib3d.calibrateCamera(
            objectPoints.map { it.toMat() },
            imagePoints.map { it.toMat() },
            Size(imageWidth.toDouble(), imageHeight.toDouble()),
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs
        )

        return CalibrationResult(
            fx = cameraMatrix.get(0, 0)[0],
            fy = cameraMatrix.get(1, 1)[0],
            cx = cameraMatrix.get(0, 2)[0],
            cy = cameraMatrix.get(1, 2)[0],
            distortionCoeffs = DoubleArray(5) { distCoeffs.get(it, 0)[0] },
            reprojectionError = error,
            imageWidth = imageWidth,
            imageHeight = imageHeight
        )
    }

    private fun generateObjectPoints(): Array<Point3F> {
        // 체커보드 3D 점 생성
        return Array(9 * 6) { idx ->
            val row = idx / 9
            val col = idx % 9
            Point3F(col * squareSize, row * squareSize, 0f)
        }
    }

    fun reset() {
        imagePoints.clear()
        objectPoints.clear()
    }
}
```

### 4.2 캘리브레이션 결과 저장

```kotlin
data class FullCalibration(
    val camera: CameraCalibrator.CalibrationResult,
    val imuNoise: ImuNoiseParameters,
    val extrinsics: CameraImuExtrinsics,
    val timeOffset: Double,  // 초
    val timestamp: Long,
    val deviceModel: String
)

data class ImuNoiseParameters(
    val accelNoiseDensity: Double,    // m/s²/√Hz
    val accelRandomWalk: Double,      // m/s³/√Hz
    val gyroNoiseDensity: Double,     // rad/s/√Hz
    val gyroRandomWalk: Double        // rad/s²/√Hz
)

data class CameraImuExtrinsics(
    val rotation: DoubleArray,      // 쿼터니언 [w, x, y, z]
    val translation: DoubleArray    // [x, y, z] meters
)

class CalibrationStorage(private val context: Context) {

    private val calibDir = File(context.filesDir, "calibration")

    init {
        calibDir.mkdirs()
    }

    fun saveCalibration(calibration: FullCalibration) {
        val file = File(calibDir, "calibration.json")
        val json = Gson().toJson(calibration)
        file.writeText(json)
    }

    fun loadCalibration(): FullCalibration? {
        val file = File(calibDir, "calibration.json")
        return if (file.exists()) {
            Gson().fromJson(file.readText(), FullCalibration::class.java)
        } else {
            null
        }
    }

    fun exportToYaml(): String {
        val calib = loadCalibration() ?: return ""

        return buildString {
            appendLine("# VI-SLAM Camera-IMU Calibration")
            appendLine("# Generated: ${java.util.Date()}")
            appendLine()
            appendLine("camera:")
            appendLine("  model: pinhole")
            appendLine("  intrinsics:")
            appendLine("    fx: ${calib.camera.fx}")
            appendLine("    fy: ${calib.camera.fy}")
            appendLine("    cx: ${calib.camera.cx}")
            appendLine("    cy: ${calib.camera.cy}")
            appendLine("  distortion:")
            appendLine("    model: radtan")
            appendLine("    coeffs: [${calib.camera.distortionCoeffs.joinToString(", ")}]")
            appendLine("  resolution:")
            appendLine("    width: ${calib.camera.imageWidth}")
            appendLine("    height: ${calib.camera.imageHeight}")
            appendLine()
            appendLine("imu:")
            appendLine("  accelerometer:")
            appendLine("    noise_density: ${calib.imuNoise.accelNoiseDensity}")
            appendLine("    random_walk: ${calib.imuNoise.accelRandomWalk}")
            appendLine("  gyroscope:")
            appendLine("    noise_density: ${calib.imuNoise.gyroNoiseDensity}")
            appendLine("    random_walk: ${calib.imuNoise.gyroRandomWalk}")
            appendLine()
            appendLine("T_cam_imu:")
            appendLine("  quaternion: [${calib.extrinsics.rotation.joinToString(", ")}]")
            appendLine("  translation: [${calib.extrinsics.translation.joinToString(", ")}]")
            appendLine()
            appendLine("time_offset_cam_imu: ${calib.timeOffset}")
        }
    }
}
```

## 5. 테스트 및 검증

### 5.1 단위 테스트

```kotlin
class TimestampSynchronizerTest {

    @Test
    fun `test IMU interpolation accuracy`() {
        val interpolator = ImuInterpolator()

        val samples = listOf(
            ImuSample(1000000000L, doubleArrayOf(0.0, 0.0, 9.8), doubleArrayOf(0.0, 0.0, 0.0)),
            ImuSample(1010000000L, doubleArrayOf(0.0, 0.0, 9.8), doubleArrayOf(0.1, 0.0, 0.0))
        )

        val interpolated = interpolator.interpolate(samples, 1005000000L)

        assertNotNull(interpolated)
        assertEquals(1005000000L, interpolated!!.timestamp)
        assertEquals(0.05, interpolated.gyro[0], 0.001)
    }

    @Test
    fun `test timestamp continuity check`() {
        val validator = SynchronizationValidator()

        val timestamps = listOf(
            1000000000L,
            1033333333L,  // 33.3ms later (30fps)
            1066666666L,
            1133333333L   // Gap! Missing frame
        )

        val gaps = validator.checkContinuity(timestamps, 50_000_000L)
        assertEquals(1, gaps.size)
        assertEquals(1066666666L, gaps[0])
    }
}
```

### 5.2 통합 테스트 체크리스트

```markdown
## 기능 테스트
- [ ] 카메라 미리보기 정상 동작
- [ ] 녹화 시작/중지 정상 동작
- [ ] IMU 데이터 수집 확인 (200Hz)
- [ ] 프레임 타임스탬프 정확성 확인
- [ ] 스트리밍 연결 및 전송 확인
- [ ] 파일 저장 및 내보내기 확인

## 동기화 테스트
- [ ] 카메라-IMU 타임스탬프 기준 동일 확인
- [ ] 프레임 드롭 시 IMU 데이터 연속성 유지
- [ ] 장시간 녹화 시 드리프트 확인 (30분 이상)

## 성능 테스트
- [ ] CPU 사용률 모니터링 (< 50% 권장)
- [ ] 메모리 사용량 모니터링
- [ ] 배터리 소모율 측정
- [ ] 발열 테스트

## 호환성 테스트
- [ ] 다양한 Android 버전 테스트 (8.0+)
- [ ] 다양한 기기 테스트
- [ ] Camera2 API 지원 수준 확인
```

## 6. 배포 고려사항

### 6.1 권한 요청

```xml
<!-- AndroidManifest.xml -->
<uses-permission android:name="android.permission.CAMERA" />
<uses-permission android:name="android.permission.RECORD_AUDIO" />
<uses-permission android:name="android.permission.WRITE_EXTERNAL_STORAGE" />
<uses-permission android:name="android.permission.INTERNET" />
<uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
<uses-permission android:name="android.permission.FOREGROUND_SERVICE_CAMERA" />
<uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />

<uses-feature android:name="android.hardware.camera" android:required="true" />
<uses-feature android:name="android.hardware.camera.autofocus" android:required="false" />
<uses-feature android:name="android.hardware.sensor.accelerometer" android:required="true" />
<uses-feature android:name="android.hardware.sensor.gyroscope" android:required="true" />
```

### 6.2 ProGuard 규칙

```proguard
# Keep data classes
-keep class com.vislam.data.** { *; }
-keep class com.vislam.calibration.** { *; }

# Keep Gson serialization
-keepattributes Signature
-keepattributes *Annotation*
-keep class com.google.gson.** { *; }
```

## 7. 참고 오픈소스 프로젝트

| 프로젝트 | 설명 | 참고 포인트 |
|---------|------|------------|
| [OpenCamera Sensors](https://github.com/prime-slam/OpenCamera-Sensors) | 동기화 카메라/IMU 앱 | 전체 아키텍처 |
| [VideoIMUCapture](https://github.com/DavidGillsjo/VideoIMUCapture-Android) | OIS 데이터 수집 | Camera2 고급 기능 |
| [MARS Logger](https://github.com/OSUPCVLab/mobile-ar-sensor-logger) | Android/iOS 로거 | 크로스 플랫폼 구현 |
| [VIRec](https://github.com/A3DV/VIRec) | 듀얼 카메라 + GPS | 다중 센서 통합 |

## References

- [Android Camera2 API](https://developer.android.com/reference/android/hardware/camera2/package-summary)
- [Android Sensor Framework](https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview)
- [iOS AVFoundation](https://developer.apple.com/documentation/avfoundation/)
- [iOS CoreMotion](https://developer.apple.com/documentation/coremotion/)
- [GetStream WebRTC Android](https://getstream.io/resources/projects/webrtc/platforms/android-kotlin/)
