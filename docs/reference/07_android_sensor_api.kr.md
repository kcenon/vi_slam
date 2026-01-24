# Android Sensor API (IMU) Guide for VI-SLAM

## 1. 개요

Android SensorManager는 가속도계, 자이로스코프 등 모션 센서에 접근하는 표준 API를 제공한다.

### VI-SLAM에 필요한 센서

| 센서 | 타입 상수 | 출력 | 단위 |
|------|----------|------|------|
| 가속도계 | TYPE_ACCELEROMETER | ax, ay, az | m/s² |
| 자이로스코프 | TYPE_GYROSCOPE | gx, gy, gz | rad/s |
| 자력계 (선택) | TYPE_MAGNETIC_FIELD | mx, my, mz | μT |
| 가속도계 (비보정) | TYPE_ACCELEROMETER_UNCALIBRATED | + bias | m/s² |
| 자이로스코프 (비보정) | TYPE_GYROSCOPE_UNCALIBRATED | + drift | rad/s |

## 2. SensorManager 기본 사용법

### 2.1 센서 접근 및 등록

```kotlin
class ImuService(private val context: Context) {

    private val sensorManager: SensorManager by lazy {
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    }

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var accelerometerUncalibrated: Sensor? = null
    private var gyroscopeUncalibrated: Sensor? = null

    fun initialize(): Boolean {
        // 기본 센서
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        // 비보정 센서 (VI-SLAM에 권장)
        accelerometerUncalibrated = sensorManager.getDefaultSensor(
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED
        )
        gyroscopeUncalibrated = sensorManager.getDefaultSensor(
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED
        )

        return accelerometer != null && gyroscope != null
    }

    fun getSensorInfo(sensor: Sensor): SensorInfo {
        return SensorInfo(
            name = sensor.name,
            vendor = sensor.vendor,
            version = sensor.version,
            resolution = sensor.resolution,
            maxRange = sensor.maximumRange,
            minDelay = sensor.minDelay,  // 마이크로초
            maxDelay = sensor.maxDelay,
            power = sensor.power  // mA
        )
    }
}

data class SensorInfo(
    val name: String,
    val vendor: String,
    val version: Int,
    val resolution: Float,
    val maxRange: Float,
    val minDelay: Int,    // 최소 샘플링 간격 (μs)
    val maxDelay: Int,
    val power: Float
)
```

### 2.2 샘플링 주파수 설정

```kotlin
// 샘플링 지연 옵션
enum class SamplingDelay(val delayUs: Int, val description: String) {
    FASTEST(0, "가능한 최대 속도, ~200-500Hz"),
    GAME(20_000, "게임용, ~50Hz"),
    UI(66_667, "UI용, ~15Hz"),
    NORMAL(200_000, "일반, ~5Hz"),

    // 커스텀 주파수 (마이크로초 단위)
    HZ_100(10_000, "100Hz"),
    HZ_200(5_000, "200Hz"),
    HZ_400(2_500, "400Hz")
}

fun startSensorListening(targetFrequency: SamplingDelay = SamplingDelay.HZ_200) {
    accelerometer?.let { sensor ->
        sensorManager.registerListener(
            sensorEventListener,
            sensor,
            targetFrequency.delayUs,
            backgroundHandler  // 백그라운드 스레드에서 처리
        )
    }

    gyroscope?.let { sensor ->
        sensorManager.registerListener(
            sensorEventListener,
            sensor,
            targetFrequency.delayUs,
            backgroundHandler
        )
    }
}

fun stopSensorListening() {
    sensorManager.unregisterListener(sensorEventListener)
}
```

## 3. 센서 이벤트 처리

### 3.1 SensorEventListener 구현

```kotlin
private val imuBuffer = ConcurrentLinkedQueue<ImuReading>()
private val bufferMaxSize = 10000  // 약 50초 분량 (200Hz)

private val sensorEventListener = object : SensorEventListener {

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                handleAccelerometer(event)
            }
            Sensor.TYPE_GYROSCOPE -> {
                handleGyroscope(event)
            }
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED -> {
                handleAccelerometerUncalibrated(event)
            }
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED -> {
                handleGyroscopeUncalibrated(event)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
        val accuracyString = when (accuracy) {
            SensorManager.SENSOR_STATUS_ACCURACY_HIGH -> "HIGH"
            SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM -> "MEDIUM"
            SensorManager.SENSOR_STATUS_ACCURACY_LOW -> "LOW"
            SensorManager.SENSOR_STATUS_UNRELIABLE -> "UNRELIABLE"
            else -> "UNKNOWN"
        }
        Log.d(TAG, "Sensor ${sensor.name} accuracy: $accuracyString")
    }
}

private fun handleAccelerometer(event: SensorEvent) {
    val reading = ImuReading(
        timestamp = event.timestamp,  // 나노초
        sensorType = SensorType.ACCELEROMETER,
        values = event.values.copyOf(),  // [ax, ay, az]
        accuracy = event.accuracy
    )
    addToBuffer(reading)
}

private fun handleGyroscope(event: SensorEvent) {
    val reading = ImuReading(
        timestamp = event.timestamp,  // 나노초
        sensorType = SensorType.GYROSCOPE,
        values = event.values.copyOf(),  // [gx, gy, gz]
        accuracy = event.accuracy
    )
    addToBuffer(reading)
}

private fun handleAccelerometerUncalibrated(event: SensorEvent) {
    // values[0-2]: 측정값, values[3-5]: 추정 바이어스
    val reading = ImuReading(
        timestamp = event.timestamp,
        sensorType = SensorType.ACCELEROMETER_UNCALIBRATED,
        values = event.values.sliceArray(0..2),
        bias = event.values.sliceArray(3..5),
        accuracy = event.accuracy
    )
    addToBuffer(reading)
}

private fun handleGyroscopeUncalibrated(event: SensorEvent) {
    // values[0-2]: 측정값, values[3-5]: 추정 드리프트
    val reading = ImuReading(
        timestamp = event.timestamp,
        sensorType = SensorType.GYROSCOPE_UNCALIBRATED,
        values = event.values.sliceArray(0..2),
        bias = event.values.sliceArray(3..5),
        accuracy = event.accuracy
    )
    addToBuffer(reading)
}

private fun addToBuffer(reading: ImuReading) {
    imuBuffer.offer(reading)

    // 버퍼 크기 제한
    while (imuBuffer.size > bufferMaxSize) {
        imuBuffer.poll()
    }
}
```

### 3.2 데이터 구조

```kotlin
enum class SensorType {
    ACCELEROMETER,
    GYROSCOPE,
    ACCELEROMETER_UNCALIBRATED,
    GYROSCOPE_UNCALIBRATED,
    MAGNETIC_FIELD
}

data class ImuReading(
    val timestamp: Long,           // 나노초 (elapsedRealtimeNanos 기준)
    val sensorType: SensorType,
    val values: FloatArray,        // [x, y, z]
    val bias: FloatArray? = null,  // 비보정 센서의 경우 바이어스
    val accuracy: Int
) {
    // 가속도계: m/s²
    val ax: Float get() = values[0]
    val ay: Float get() = values[1]
    val az: Float get() = values[2]

    // 자이로스코프: rad/s
    val gx: Float get() = values[0]
    val gy: Float get() = values[1]
    val gz: Float get() = values[2]
}
```

## 4. 타임스탬프 이해

### 4.1 타임스탬프 소스

```kotlin
/*
 * SensorEvent.timestamp:
 * - 단위: 나노초 (ns)
 * - 기준: SystemClock.elapsedRealtimeNanos()
 * - 기기 부팅 이후 경과 시간 (딥슬립 포함)
 *
 * Camera2 SENSOR_TIMESTAMP과 동일한 시간 기준 사용
 * → 카메라-IMU 동기화 가능!
 */

fun getCurrentTimeNanos(): Long {
    return SystemClock.elapsedRealtimeNanos()
}

// 타임스탬프 검증
fun validateTimestamp(sensorTimestamp: Long): Boolean {
    val currentTime = SystemClock.elapsedRealtimeNanos()
    val diff = currentTime - sensorTimestamp

    // 센서 타임스탬프는 현재 시간보다 과거여야 함
    // 일반적으로 수~수십 ms 이내
    return diff > 0 && diff < 100_000_000L  // 100ms 이내
}
```

### 4.2 실제 샘플링 주파수 측정

```kotlin
class FrequencyMonitor {
    private var lastTimestamp: Long = 0
    private val intervals = mutableListOf<Long>()
    private val maxSamples = 1000

    fun recordTimestamp(timestamp: Long) {
        if (lastTimestamp > 0) {
            val interval = timestamp - lastTimestamp
            intervals.add(interval)

            if (intervals.size > maxSamples) {
                intervals.removeAt(0)
            }
        }
        lastTimestamp = timestamp
    }

    fun getAverageFrequency(): Double {
        if (intervals.isEmpty()) return 0.0

        val avgIntervalNs = intervals.average()
        return 1_000_000_000.0 / avgIntervalNs  // Hz
    }

    fun getStatistics(): FrequencyStats {
        if (intervals.isEmpty()) return FrequencyStats(0.0, 0.0, 0.0, 0.0)

        val freqs = intervals.map { 1_000_000_000.0 / it }
        return FrequencyStats(
            mean = freqs.average(),
            min = freqs.minOrNull() ?: 0.0,
            max = freqs.maxOrNull() ?: 0.0,
            stdDev = calculateStdDev(freqs)
        )
    }
}

data class FrequencyStats(
    val mean: Double,
    val min: Double,
    val max: Double,
    val stdDev: Double
)
```

## 5. 센서 좌표계

### 5.1 Android 센서 좌표계

```
        Y (위쪽)
        ^
        |
        |
        +-------> X (오른쪽)
       /
      /
     v
    Z (화면 바깥쪽)

- 기기가 세로 방향, 화면이 사용자를 향할 때 기준
- 오른손 좌표계 (Right-handed coordinate system)
```

### 5.2 좌표계 변환

```kotlin
// Android → 일반적인 로봇/카메라 좌표계 변환 예시
// Android: X-right, Y-up, Z-out
// Robot: X-forward, Y-left, Z-up

data class ImuReadingConverted(
    val timestamp: Long,
    val accel: FloatArray,  // [forward, left, up]
    val gyro: FloatArray    // [roll_rate, pitch_rate, yaw_rate]
)

fun convertToRobotFrame(reading: ImuReading): ImuReadingConverted {
    // 예시 변환 (기기가 수평으로 앞을 향할 때)
    // 실제 변환은 기기 마운팅 방향에 따라 달라짐

    return when (reading.sensorType) {
        SensorType.ACCELEROMETER -> {
            ImuReadingConverted(
                timestamp = reading.timestamp,
                accel = floatArrayOf(
                    -reading.ay,   // forward = -Y
                    -reading.ax,   // left = -X
                    reading.az     // up = Z
                ),
                gyro = floatArrayOf()
            )
        }
        SensorType.GYROSCOPE -> {
            ImuReadingConverted(
                timestamp = reading.timestamp,
                accel = floatArrayOf(),
                gyro = floatArrayOf(
                    -reading.gy,   // roll = -Y rotation
                    -reading.gx,   // pitch = -X rotation
                    reading.gz     // yaw = Z rotation
                )
            )
        }
        else -> throw IllegalArgumentException("Unsupported sensor type")
    }
}
```

## 6. Android 12+ 제한사항

### 6.1 Rate Limiting

```kotlin
/*
 * Android 12 (API 31) 이상에서:
 * - 앱이 포그라운드가 아닌 경우 센서 샘플링 제한
 * - 백그라운드에서 200Hz 이상 제한될 수 있음
 *
 * 해결책:
 * 1. Foreground Service 사용
 * 2. HIGH_SAMPLING_RATE_SENSORS 권한 선언
 */

// AndroidManifest.xml
// <uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />

fun checkHighSamplingPermission(): Boolean {
    return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.HIGH_SAMPLING_RATE_SENSORS
        ) == PackageManager.PERMISSION_GRANTED
    } else {
        true  // Android 11 이하에서는 항상 가능
    }
}
```

### 6.2 Foreground Service 설정

```kotlin
class ImuForegroundService : Service() {

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        startForeground(NOTIFICATION_ID, createNotification())
        startImuRecording()
        return START_STICKY
    }

    private fun createNotification(): Notification {
        val channelId = "imu_recording"

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                channelId,
                "IMU Recording",
                NotificationManager.IMPORTANCE_LOW
            )
            val notificationManager = getSystemService(NotificationManager::class.java)
            notificationManager.createNotificationChannel(channel)
        }

        return NotificationCompat.Builder(this, channelId)
            .setContentTitle("VI-SLAM Recording")
            .setContentText("Recording camera and IMU data...")
            .setSmallIcon(R.drawable.ic_recording)
            .build()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
```

## 7. 데이터 저장 포맷

### 7.1 CSV 포맷

```kotlin
class ImuCsvWriter(private val outputFile: File) {
    private val writer: BufferedWriter = outputFile.bufferedWriter()

    init {
        // 헤더 작성
        writer.write("timestamp_ns,sensor_type,x,y,z,bias_x,bias_y,bias_z,accuracy\n")
    }

    fun writeReading(reading: ImuReading) {
        val line = buildString {
            append(reading.timestamp)
            append(",")
            append(reading.sensorType.name)
            append(",")
            append(reading.values.joinToString(","))
            append(",")
            append(reading.bias?.joinToString(",") ?: ",,")
            append(",")
            append(reading.accuracy)
            append("\n")
        }
        writer.write(line)
    }

    fun close() {
        writer.flush()
        writer.close()
    }
}
```

### 7.2 Binary 포맷 (고성능)

```kotlin
class ImuBinaryWriter(private val outputFile: File) {
    private val outputStream: DataOutputStream =
        DataOutputStream(BufferedOutputStream(FileOutputStream(outputFile)))

    // 헤더 작성 (한 번만)
    fun writeHeader() {
        outputStream.writeInt(MAGIC_NUMBER)  // 0x494D5530 = "IMU0"
        outputStream.writeInt(VERSION)       // 형식 버전
        outputStream.writeLong(System.currentTimeMillis())  // 기록 시작 시간
    }

    // 각 센서 리딩 작성 (8 + 1 + 12 + 12 + 1 = 34 bytes per reading)
    fun writeReading(reading: ImuReading) {
        outputStream.writeLong(reading.timestamp)
        outputStream.writeByte(reading.sensorType.ordinal)
        reading.values.forEach { outputStream.writeFloat(it) }
        reading.bias?.forEach { outputStream.writeFloat(it) }
            ?: repeat(3) { outputStream.writeFloat(0f) }
        outputStream.writeByte(reading.accuracy)
    }

    fun close() {
        outputStream.flush()
        outputStream.close()
    }

    companion object {
        const val MAGIC_NUMBER = 0x494D5530
        const val VERSION = 1
    }
}
```

## 8. 성능 최적화 팁

### 8.1 버퍼링 전략

```kotlin
// 1. Ring Buffer 사용 (메모리 효율적)
class RingBuffer<T>(private val capacity: Int) {
    private val buffer = arrayOfNulls<Any>(capacity)
    private var head = 0
    private var tail = 0
    private var size = 0

    @Synchronized
    fun add(item: T) {
        buffer[tail] = item
        tail = (tail + 1) % capacity
        if (size < capacity) size++ else head = (head + 1) % capacity
    }

    @Synchronized
    fun toList(): List<T> {
        val result = mutableListOf<T>()
        var index = head
        repeat(size) {
            @Suppress("UNCHECKED_CAST")
            result.add(buffer[index] as T)
            index = (index + 1) % capacity
        }
        return result
    }
}

// 2. 배치 쓰기 (I/O 효율적)
class BatchWriter(private val batchSize: Int = 100) {
    private val batch = mutableListOf<ImuReading>()

    fun addReading(reading: ImuReading) {
        batch.add(reading)
        if (batch.size >= batchSize) {
            flush()
        }
    }

    private fun flush() {
        // 한 번에 여러 리딩 쓰기
        writeBatch(batch.toList())
        batch.clear()
    }
}
```

### 8.2 스레드 처리

```kotlin
// 별도 HandlerThread에서 센서 이벤트 처리
class ImuHandler {
    private val handlerThread = HandlerThread("ImuThread", Process.THREAD_PRIORITY_MORE_FAVORABLE)
    private lateinit var handler: Handler

    fun start() {
        handlerThread.start()
        handler = Handler(handlerThread.looper)
    }

    fun getHandler(): Handler = handler

    fun stop() {
        handlerThread.quitSafely()
    }
}
```

## References

- [Android Sensors Overview](https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview)
- [Android Motion Sensors](https://developer.android.com/develop/sensors-and-location/sensors/sensors_motion)
- [Sensor Logger App](https://www.tszheichoi.com/sensorlogger)
- [Sensors-Data-Logger GitHub](https://github.com/PyojinKim/Sensors-Data-Logger)
