# Camera-IMU Timestamp Synchronization Implementation

## 1. Overview

Accurate time synchronization between camera and IMU data is critical to VI-SLAM system performance. This document covers software-based synchronization implementation methods.

### Importance of Synchronization

```
Impact of 1ms time offset (typical motion):
- Rotation 1 rad/s → 1 mrad (0.057°) error
- Translation 1 m/s → 1 mm error

Fast motion (10 rad/s rotation):
- 1ms offset → 10 mrad (0.57°) error
- VI-SLAM performance degrades significantly
```

## 2. Platform-Specific Timestamp Systems

### 2.1 Android

```kotlin
/*
 * Android Timestamp Sources:
 *
 * 1. SystemClock.elapsedRealtimeNanos()
 *    - Elapsed time since device boot
 *    - Includes deep sleep time
 *    - Guaranteed monotonic increase
 *
 * 2. System.nanoTime()
 *    - Elapsed time since JVM start
 *    - Does not include deep sleep time
 *    - May not match camera/sensor timestamps
 *
 * 3. System.currentTimeMillis()
 *    - Wall clock time
 *    - Can be changed by user/NTP
 *    - Not suitable for synchronization
 *
 * Camera2 & SensorManager:
 * - When SENSOR_TIMESTAMP_SOURCE_REALTIME,
 *   uses the same basis as elapsedRealtimeNanos()
 */

object TimestampUtils {

    // Current system time (nanoseconds)
    fun currentNanos(): Long = SystemClock.elapsedRealtimeNanos()

    // Nanoseconds → Seconds
    fun nanosToSeconds(nanos: Long): Double = nanos / 1_000_000_000.0

    // Seconds → Nanoseconds
    fun secondsToNanos(seconds: Double): Long = (seconds * 1_000_000_000).toLong()

    // Timestamp validity check
    fun isValidTimestamp(timestamp: Long): Boolean {
        val current = currentNanos()
        // Invalid if in the future or more than 1 hour in the past
        return timestamp <= current && (current - timestamp) < 3_600_000_000_000L
    }
}
```

### 2.2 iOS

```swift
/*
 * iOS Timestamp Sources:
 *
 * 1. mach_absolute_time()
 *    - Tick count since device boot
 *    - Requires conversion to nanoseconds via mach_timebase_info
 *
 * 2. CACurrentMediaTime()
 *    - Based on mach_absolute_time, returns seconds
 *    - CoreAnimation timing function
 *
 * 3. Date() / CFAbsoluteTimeGetCurrent()
 *    - Wall clock time
 *    - Not suitable for synchronization
 *
 * AVFoundation (Camera):
 * - CMSampleBufferGetPresentationTimeStamp
 * - Based on hostTime (mach_absolute_time conversion)
 *
 * CoreMotion (IMU):
 * - timestamp property
 * - Seconds since system boot
 * - Based on mach_absolute_time
 */

struct TimestampUtils {

    // mach_absolute_time → Nanoseconds
    static func machToNanos(_ machTime: UInt64) -> Int64 {
        var timebaseInfo = mach_timebase_info_data_t()
        mach_timebase_info(&timebaseInfo)

        let nanos = machTime * UInt64(timebaseInfo.numer) / UInt64(timebaseInfo.denom)
        return Int64(nanos)
    }

    // Current time (nanoseconds)
    static func currentNanos() -> Int64 {
        return machToNanos(mach_absolute_time())
    }

    // CoreMotion TimeInterval → Nanoseconds
    static func timeIntervalToNanos(_ interval: TimeInterval) -> Int64 {
        return Int64(interval * 1_000_000_000)
    }
}
```

## 3. Software Synchronization Strategies

### 3.1 Using Common Time Reference

```kotlin
// Android: Verify Camera2 and SensorManager use the same time reference

class SynchronizationChecker(private val context: Context) {

    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    data class SyncCapability(
        val cameraTimestampSource: String,
        val canHardwareSync: Boolean,
        val estimatedOffset: Long?  // nanoseconds
    )

    fun checkSyncCapability(cameraId: String): SyncCapability {
        val characteristics = cameraManager.getCameraCharacteristics(cameraId)

        val timestampSource = characteristics.get(
            CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
        )

        val canHardwareSync = timestampSource ==
            CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME

        return SyncCapability(
            cameraTimestampSource = when (timestampSource) {
                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME -> "REALTIME"
                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN -> "UNKNOWN"
                else -> "OTHER"
            },
            canHardwareSync = canHardwareSync,
            estimatedOffset = if (canHardwareSync) 0L else null
        )
    }
}
```

### 3.2 Time Offset Estimation (Cross-Correlation)

```kotlin
/**
 * Angular velocity-based time offset estimation
 *
 * Principle:
 * 1. Estimate angular velocity from camera frames using optical flow
 * 2. Compare with IMU gyroscope angular velocity
 * 3. Find optimal offset using cross-correlation
 */

class TimeOffsetEstimator {

    data class AngularVelocitySample(
        val timestamp: Long,  // nanoseconds
        val omega: Double     // rad/s (magnitude)
    )

    /**
     * Estimate time offset using cross-correlation
     *
     * @param cameraSamples Angular velocity estimated from camera
     * @param imuSamples IMU gyroscope angular velocity
     * @param searchRangeMs Search range (milliseconds)
     * @return Estimated offset (nanoseconds, positive if IMU leads camera)
     */
    fun estimateOffset(
        cameraSamples: List<AngularVelocitySample>,
        imuSamples: List<AngularVelocitySample>,
        searchRangeMs: Long = 100
    ): Long {
        val searchRangeNs = searchRangeMs * 1_000_000L
        val stepNs = 1_000_000L  // 1ms step

        var bestOffset = 0L
        var bestCorrelation = Double.NEGATIVE_INFINITY

        var offset = -searchRangeNs
        while (offset <= searchRangeNs) {
            val correlation = computeCorrelation(cameraSamples, imuSamples, offset)

            if (correlation > bestCorrelation) {
                bestCorrelation = correlation
                bestOffset = offset
            }

            offset += stepNs
        }

        // Sub-millisecond refinement using quadratic fitting
        return refineOffset(cameraSamples, imuSamples, bestOffset, stepNs)
    }

    private fun computeCorrelation(
        cameraSamples: List<AngularVelocitySample>,
        imuSamples: List<AngularVelocitySample>,
        offset: Long
    ): Double {
        // Apply offset to camera samples and match with IMU
        var sumProduct = 0.0
        var count = 0

        for (camSample in cameraSamples) {
            val adjustedTime = camSample.timestamp + offset

            // Find closest IMU sample
            val imuSample = findClosestSample(imuSamples, adjustedTime) ?: continue

            sumProduct += camSample.omega * imuSample.omega
            count++
        }

        return if (count > 0) sumProduct / count else 0.0
    }

    private fun findClosestSample(
        samples: List<AngularVelocitySample>,
        targetTime: Long
    ): AngularVelocitySample? {
        return samples.minByOrNull { abs(it.timestamp - targetTime) }
    }

    private fun refineOffset(
        cameraSamples: List<AngularVelocitySample>,
        imuSamples: List<AngularVelocitySample>,
        coarseOffset: Long,
        step: Long
    ): Long {
        // Quadratic fitting for sub-step accuracy
        val c0 = computeCorrelation(cameraSamples, imuSamples, coarseOffset - step)
        val c1 = computeCorrelation(cameraSamples, imuSamples, coarseOffset)
        val c2 = computeCorrelation(cameraSamples, imuSamples, coarseOffset + step)

        // Parabola vertex: offset_refined = coarse + step * (c0 - c2) / (2 * (c0 - 2*c1 + c2))
        val denominator = 2 * (c0 - 2 * c1 + c2)
        if (abs(denominator) < 1e-10) return coarseOffset

        val refinement = step * (c0 - c2) / denominator
        return coarseOffset + refinement.toLong()
    }
}
```

### 3.3 Real-Time Offset Correction

```kotlin
/**
 * Kalman Filter-based real-time time offset estimation
 */
class OnlineTimeOffsetEstimator {

    // State: [offset, offset_drift]
    private var state = doubleArrayOf(0.0, 0.0)

    // Covariance matrix
    private var P = arrayOf(
        doubleArrayOf(1e-3, 0.0),
        doubleArrayOf(0.0, 1e-6)
    )

    // Process noise
    private val Q = arrayOf(
        doubleArrayOf(1e-8, 0.0),
        doubleArrayOf(0.0, 1e-10)
    )

    // Measurement noise
    private var R = 1e-4

    /**
     * Update offset estimate with new measurement
     *
     * @param measuredOffset Offset estimated from single measurement (seconds)
     * @param dt Time since previous update (seconds)
     */
    fun update(measuredOffset: Double, dt: Double) {
        // Prediction step
        val F = arrayOf(
            doubleArrayOf(1.0, dt),
            doubleArrayOf(0.0, 1.0)
        )

        val predictedState = matVecMul(F, state)
        val predictedP = matAdd(matMul(matMul(F, P), transpose(F)), Q)

        // Update step
        val H = doubleArrayOf(1.0, 0.0)
        val y = measuredOffset - (H[0] * predictedState[0] + H[1] * predictedState[1])

        val S = H[0] * predictedP[0][0] + H[1] * predictedP[1][0] +
                H[0] * predictedP[0][1] + H[1] * predictedP[1][1] + R

        val K = doubleArrayOf(
            (predictedP[0][0] * H[0] + predictedP[0][1] * H[1]) / S,
            (predictedP[1][0] * H[0] + predictedP[1][1] * H[1]) / S
        )

        state[0] = predictedState[0] + K[0] * y
        state[1] = predictedState[1] + K[1] * y

        // Update covariance
        P[0][0] = (1 - K[0] * H[0]) * predictedP[0][0]
        P[0][1] = (1 - K[0] * H[0]) * predictedP[0][1]
        P[1][0] = -K[1] * H[0] * predictedP[0][0] + predictedP[1][0]
        P[1][1] = -K[1] * H[0] * predictedP[0][1] + predictedP[1][1]
    }

    fun getEstimatedOffset(): Double = state[0]  // seconds

    fun getEstimatedDrift(): Double = state[1]   // seconds/second

    fun getOffsetUncertainty(): Double = sqrt(P[0][0])

    // Matrix operation helper functions (omitted)
    private fun matVecMul(m: Array<DoubleArray>, v: DoubleArray): DoubleArray = TODO()
    private fun matMul(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> = TODO()
    private fun matAdd(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> = TODO()
    private fun transpose(m: Array<DoubleArray>): Array<DoubleArray> = TODO()
}
```

## 4. IMU Data Interpolation

### 4.1 Linear Interpolation

```kotlin
/**
 * IMU data interpolation aligned to camera frame times
 */
class ImuInterpolator {

    data class ImuSample(
        val timestamp: Long,    // nanoseconds
        val accel: DoubleArray, // [ax, ay, az] m/s²
        val gyro: DoubleArray   // [gx, gy, gz] rad/s
    )

    /**
     * Estimate IMU value at specific time using linear interpolation
     */
    fun interpolate(
        imuSamples: List<ImuSample>,
        targetTime: Long
    ): ImuSample? {
        // Find two samples that bracket targetTime
        val sortedSamples = imuSamples.sortedBy { it.timestamp }

        val beforeIdx = sortedSamples.indexOfLast { it.timestamp <= targetTime }
        if (beforeIdx < 0 || beforeIdx >= sortedSamples.size - 1) {
            return null  // Out of range
        }

        val before = sortedSamples[beforeIdx]
        val after = sortedSamples[beforeIdx + 1]

        // Calculate interpolation ratio
        val t = (targetTime - before.timestamp).toDouble() /
                (after.timestamp - before.timestamp).toDouble()

        // Linear interpolation
        return ImuSample(
            timestamp = targetTime,
            accel = interpolateArray(before.accel, after.accel, t),
            gyro = interpolateArray(before.gyro, after.gyro, t)
        )
    }

    /**
     * Get all IMU samples between camera frames + interpolated start/end times
     */
    fun getIntegratedImu(
        imuSamples: List<ImuSample>,
        startTime: Long,
        endTime: Long
    ): List<ImuSample> {
        val result = mutableListOf<ImuSample>()

        // Interpolated sample at start time
        interpolate(imuSamples, startTime)?.let { result.add(it) }

        // Original samples between start and end
        imuSamples.filter { it.timestamp in (startTime + 1) until endTime }
            .sortedBy { it.timestamp }
            .forEach { result.add(it) }

        // Interpolated sample at end time
        interpolate(imuSamples, endTime)?.let { result.add(it) }

        return result
    }

    private fun interpolateArray(
        a: DoubleArray,
        b: DoubleArray,
        t: Double
    ): DoubleArray {
        return DoubleArray(a.size) { i ->
            a[i] * (1 - t) + b[i] * t
        }
    }
}
```

### 4.2 SLERP (Spherical Linear Interpolation)

```kotlin
/**
 * For gyroscope data, SLERP is recommended for rotation interpolation
 */
class QuaternionInterpolator {

    data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {

        fun normalize(): Quaternion {
            val norm = sqrt(w * w + x * x + y * y + z * z)
            return Quaternion(w / norm, x / norm, y / norm, z / norm)
        }

        fun dot(other: Quaternion): Double {
            return w * other.w + x * other.x + y * other.y + z * other.z
        }

        operator fun times(scalar: Double): Quaternion {
            return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar)
        }

        operator fun plus(other: Quaternion): Quaternion {
            return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z)
        }
    }

    /**
     * SLERP (Spherical Linear Interpolation)
     */
    fun slerp(q0: Quaternion, q1: Quaternion, t: Double): Quaternion {
        var dot = q0.dot(q1)

        // Ensure shortest path
        val q1Adjusted = if (dot < 0) {
            dot = -dot
            Quaternion(-q1.w, -q1.x, -q1.y, -q1.z)
        } else {
            q1
        }

        // Use linear interpolation when very close
        if (dot > 0.9995) {
            return (q0 * (1 - t) + q1Adjusted * t).normalize()
        }

        val theta0 = acos(dot)
        val theta = theta0 * t
        val sinTheta = sin(theta)
        val sinTheta0 = sin(theta0)

        val s0 = cos(theta) - dot * sinTheta / sinTheta0
        val s1 = sinTheta / sinTheta0

        return (q0 * s0 + q1Adjusted * s1).normalize()
    }
}
```

## 5. Synchronization Quality Validation

### 5.1 Synchronization Metrics

```kotlin
class SynchronizationValidator {

    data class SyncMetrics(
        val meanOffset: Double,      // Mean offset (seconds)
        val stdOffset: Double,       // Offset standard deviation
        val maxJitter: Double,       // Maximum jitter
        val missingFrames: Int,      // Number of missing frames
        val outOfOrderSamples: Int   // Number of out-of-order samples
    )

    fun validateSynchronization(
        frameTimestamps: List<Long>,
        imuTimestamps: List<Long>,
        expectedFrameRate: Double,
        expectedImuRate: Double
    ): SyncMetrics {
        // 1. Frame interval analysis
        val frameIntervals = frameTimestamps.zipWithNext { a, b -> b - a }
        val expectedFrameInterval = (1_000_000_000.0 / expectedFrameRate).toLong()

        val frameMissing = frameIntervals.count {
            it > expectedFrameInterval * 1.5
        }

        // 2. IMU interval analysis
        val imuIntervals = imuTimestamps.zipWithNext { a, b -> b - a }
        val outOfOrder = imuTimestamps.zipWithNext().count { (a, b) -> b <= a }

        // 3. Jitter calculation
        val expectedImuInterval = 1_000_000_000.0 / expectedImuRate
        val jitters = imuIntervals.map { abs(it - expectedImuInterval) }

        return SyncMetrics(
            meanOffset = 0.0,  // Requires separate calculation
            stdOffset = 0.0,
            maxJitter = jitters.maxOrNull()?.let { it / 1_000_000.0 } ?: 0.0,  // ms
            missingFrames = frameMissing,
            outOfOrderSamples = outOfOrder
        )
    }

    fun checkContinuity(timestamps: List<Long>, maxGapNs: Long): List<Long> {
        // Return timestamps where continuity is broken
        return timestamps.zipWithNext()
            .filter { (a, b) -> b - a > maxGapNs }
            .map { it.first }
    }
}
```

### 5.2 Visual Validation

```kotlin
/**
 * For debugging: Generate visualization data for camera-IMU synchronization
 */
class SyncVisualizer {

    data class VisualizationData(
        val timeSeconds: List<Double>,
        val cameraAngularVel: List<Double>,
        val imuAngularVel: List<Double>,
        val correlationByOffset: Map<Double, Double>
    )

    fun generateVisualizationData(
        cameraData: List<Pair<Long, Double>>,  // (timestamp, omega)
        imuData: List<Pair<Long, Double>>,
        testOffsets: List<Long>
    ): VisualizationData {
        val baseTime = minOf(
            cameraData.minOfOrNull { it.first } ?: 0,
            imuData.minOfOrNull { it.first } ?: 0
        )

        val timeSeconds = cameraData.map { (it.first - baseTime) / 1e9 }
        val cameraVel = cameraData.map { it.second }
        val imuVel = imuData.map { it.second }

        val correlations = testOffsets.associate { offset ->
            val correlation = computeCorrelation(cameraData, imuData, offset)
            offset / 1e6 to correlation  // Convert to ms
        }

        return VisualizationData(
            timeSeconds = timeSeconds,
            cameraAngularVel = cameraVel,
            imuAngularVel = imuVel,
            correlationByOffset = correlations
        )
    }

    private fun computeCorrelation(
        cameraData: List<Pair<Long, Double>>,
        imuData: List<Pair<Long, Double>>,
        offset: Long
    ): Double {
        // Cross-correlation calculation (simplified)
        return 0.0  // Actual implementation required
    }
}
```

## 6. Implementation Checklist

### Data Collection Phase

- [ ] Verify camera timestamp source (REALTIME or not)
- [ ] Verify IMU sampling frequency (200Hz or higher recommended)
- [ ] Unify timestamp units (nanoseconds recommended)
- [ ] Verify timestamps are monotonically increasing

### Synchronization Processing Phase

- [ ] Initial time offset estimation (cross-correlation)
- [ ] Online offset tracking (Kalman Filter)
- [ ] IMU data interpolation (aligned to frame times)

### Validation Phase

- [ ] Check for frame drops
- [ ] Verify IMU data continuity
- [ ] Verify time offset stability
- [ ] Validate SLAM trajectory quality

## References

- [Online Temporal Calibration for Camera-IMU Systems](https://arxiv.org/pdf/1808.00692)
- [Kalibr Camera-IMU Calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [Sensor Synchronization for Android Phone VI-SLAM](https://www.researchgate.net/publication/324929837_Sensor_Synchronization_for_Android_Phone_Tightly-Coupled_Visual-Inertial_SLAM)
- [MARS Logger Paper](https://arxiv.org/pdf/2001.00470)
