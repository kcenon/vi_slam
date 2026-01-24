# Android Sensor API (IMU) Guide for VI-SLAM

## 1. Overview

Android SensorManager provides a standard API for accessing motion sensors such as accelerometers and gyroscopes.

### Sensors Required for VI-SLAM

| Sensor | Type Constant | Output | Unit |
|------|----------|------|------|
| Accelerometer | TYPE_ACCELEROMETER | ax, ay, az | m/s² |
| Gyroscope | TYPE_GYROSCOPE | gx, gy, gz | rad/s |
| Magnetometer (optional) | TYPE_MAGNETIC_FIELD | mx, my, mz | μT |
| Accelerometer (uncalibrated) | TYPE_ACCELEROMETER_UNCALIBRATED | + bias | m/s² |
| Gyroscope (uncalibrated) | TYPE_GYROSCOPE_UNCALIBRATED | + drift | rad/s |

## 2. SensorManager Basic Usage

### 2.1 Sensor Access and Registration

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
        // Basic sensors
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        // Uncalibrated sensors (recommended for VI-SLAM)
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
            minDelay = sensor.minDelay,  // microseconds
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
    val minDelay: Int,    // minimum sampling interval (μs)
    val maxDelay: Int,
    val power: Float
)
```

### 2.2 Sampling Frequency Configuration

```kotlin
// Sampling delay options
enum class SamplingDelay(val delayUs: Int, val description: String) {
    FASTEST(0, "Maximum possible speed, ~200-500Hz"),
    GAME(20_000, "For games, ~50Hz"),
    UI(66_667, "For UI, ~15Hz"),
    NORMAL(200_000, "Normal, ~5Hz"),

    // Custom frequency (in microseconds)
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
            backgroundHandler  // Process on background thread
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

## 3. Sensor Event Processing

### 3.1 SensorEventListener Implementation

```kotlin
private val imuBuffer = ConcurrentLinkedQueue<ImuReading>()
private val bufferMaxSize = 10000  // Approximately 50 seconds worth (200Hz)

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
        timestamp = event.timestamp,  // nanoseconds
        sensorType = SensorType.ACCELEROMETER,
        values = event.values.copyOf(),  // [ax, ay, az]
        accuracy = event.accuracy
    )
    addToBuffer(reading)
}

private fun handleGyroscope(event: SensorEvent) {
    val reading = ImuReading(
        timestamp = event.timestamp,  // nanoseconds
        sensorType = SensorType.GYROSCOPE,
        values = event.values.copyOf(),  // [gx, gy, gz]
        accuracy = event.accuracy
    )
    addToBuffer(reading)
}

private fun handleAccelerometerUncalibrated(event: SensorEvent) {
    // values[0-2]: measurements, values[3-5]: estimated bias
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
    // values[0-2]: measurements, values[3-5]: estimated drift
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

    // Limit buffer size
    while (imuBuffer.size > bufferMaxSize) {
        imuBuffer.poll()
    }
}
```

### 3.2 Data Structures

```kotlin
enum class SensorType {
    ACCELEROMETER,
    GYROSCOPE,
    ACCELEROMETER_UNCALIBRATED,
    GYROSCOPE_UNCALIBRATED,
    MAGNETIC_FIELD
}

data class ImuReading(
    val timestamp: Long,           // nanoseconds (elapsedRealtimeNanos basis)
    val sensorType: SensorType,
    val values: FloatArray,        // [x, y, z]
    val bias: FloatArray? = null,  // bias for uncalibrated sensors
    val accuracy: Int
) {
    // Accelerometer: m/s²
    val ax: Float get() = values[0]
    val ay: Float get() = values[1]
    val az: Float get() = values[2]

    // Gyroscope: rad/s
    val gx: Float get() = values[0]
    val gy: Float get() = values[1]
    val gz: Float get() = values[2]
}
```

## 4. Understanding Timestamps

### 4.1 Timestamp Source

```kotlin
/*
 * SensorEvent.timestamp:
 * - Unit: nanoseconds (ns)
 * - Basis: SystemClock.elapsedRealtimeNanos()
 * - Time elapsed since device boot (including deep sleep)
 *
 * Uses the same time basis as Camera2 SENSOR_TIMESTAMP
 * → Camera-IMU synchronization possible!
 */

fun getCurrentTimeNanos(): Long {
    return SystemClock.elapsedRealtimeNanos()
}

// Timestamp validation
fun validateTimestamp(sensorTimestamp: Long): Boolean {
    val currentTime = SystemClock.elapsedRealtimeNanos()
    val diff = currentTime - sensorTimestamp

    // Sensor timestamp should be in the past relative to current time
    // Typically within a few to tens of milliseconds
    return diff > 0 && diff < 100_000_000L  // Within 100ms
}
```

### 4.2 Measuring Actual Sampling Frequency

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

## 5. Sensor Coordinate System

### 5.1 Android Sensor Coordinate System

```
        Y (up)
        ^
        |
        |
        +-------> X (right)
       /
      /
     v
    Z (out of screen)

- Reference: device in portrait orientation, screen facing the user
- Right-handed coordinate system
```

### 5.2 Coordinate System Transformation

```kotlin
// Android → typical robot/camera coordinate system transformation example
// Android: X-right, Y-up, Z-out
// Robot: X-forward, Y-left, Z-up

data class ImuReadingConverted(
    val timestamp: Long,
    val accel: FloatArray,  // [forward, left, up]
    val gyro: FloatArray    // [roll_rate, pitch_rate, yaw_rate]
)

fun convertToRobotFrame(reading: ImuReading): ImuReadingConverted {
    // Example transformation (when device is horizontal facing forward)
    // Actual transformation depends on device mounting orientation

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

## 6. Android 12+ Restrictions

### 6.1 Rate Limiting

```kotlin
/*
 * On Android 12 (API 31) and above:
 * - Sensor sampling is limited when app is not in foreground
 * - May be limited to below 200Hz in background
 *
 * Solutions:
 * 1. Use Foreground Service
 * 2. Declare HIGH_SAMPLING_RATE_SENSORS permission
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
        true  // Always available on Android 11 and below
    }
}
```

### 6.2 Foreground Service Setup

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

## 7. Data Storage Formats

### 7.1 CSV Format

```kotlin
class ImuCsvWriter(private val outputFile: File) {
    private val writer: BufferedWriter = outputFile.bufferedWriter()

    init {
        // Write header
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

### 7.2 Binary Format (High Performance)

```kotlin
class ImuBinaryWriter(private val outputFile: File) {
    private val outputStream: DataOutputStream =
        DataOutputStream(BufferedOutputStream(FileOutputStream(outputFile)))

    // Write header (once)
    fun writeHeader() {
        outputStream.writeInt(MAGIC_NUMBER)  // 0x494D5530 = "IMU0"
        outputStream.writeInt(VERSION)       // format version
        outputStream.writeLong(System.currentTimeMillis())  // recording start time
    }

    // Write each sensor reading (8 + 1 + 12 + 12 + 1 = 34 bytes per reading)
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

## 8. Performance Optimization Tips

### 8.1 Buffering Strategy

```kotlin
// 1. Use Ring Buffer (memory efficient)
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

// 2. Batch writing (I/O efficient)
class BatchWriter(private val batchSize: Int = 100) {
    private val batch = mutableListOf<ImuReading>()

    fun addReading(reading: ImuReading) {
        batch.add(reading)
        if (batch.size >= batchSize) {
            flush()
        }
    }

    private fun flush() {
        // Write multiple readings at once
        writeBatch(batch.toList())
        batch.clear()
    }
}
```

### 8.2 Thread Processing

```kotlin
// Process sensor events on separate HandlerThread
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
