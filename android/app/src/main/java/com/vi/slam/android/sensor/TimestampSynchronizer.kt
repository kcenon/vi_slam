package com.vi.slam.android.sensor

import android.util.Log
import kotlin.math.abs

/**
 * Synchronized frame data containing camera frame and associated IMU samples.
 *
 * @property frameTimestampNs Camera frame timestamp in nanoseconds
 * @property frameSequence Frame sequence number
 * @property imuSamplesBefore IMU samples captured before frame timestamp
 * @property imuSamplesAfter IMU samples captured after frame timestamp (up to next frame)
 * @property interpolatedImu IMU sample interpolated at exact frame timestamp
 */
data class SynchronizedData(
    val frameTimestampNs: Long,
    val frameSequence: Long,
    val imuSamplesBefore: List<IMUSample>,
    val imuSamplesAfter: List<IMUSample>,
    val interpolatedImu: InterpolatedIMU?
)

/**
 * Interpolated IMU measurement at a specific timestamp.
 *
 * Contains both accelerometer and gyroscope values interpolated
 * at the exact camera frame timestamp.
 *
 * @property timestampNs Target timestamp in nanoseconds
 * @property ax Interpolated accelerometer X (m/s²)
 * @property ay Interpolated accelerometer Y (m/s²)
 * @property az Interpolated accelerometer Z (m/s²)
 * @property gx Interpolated gyroscope X (rad/s)
 * @property gy Interpolated gyroscope Y (rad/s)
 * @property gz Interpolated gyroscope Z (rad/s)
 */
data class InterpolatedIMU(
    val timestampNs: Long,
    val ax: Float,
    val ay: Float,
    val az: Float,
    val gx: Float,
    val gy: Float,
    val gz: Float
)

/**
 * Synchronization status for monitoring.
 *
 * @property isHealthy True if synchronization is working correctly
 * @property timeDriftNs Estimated drift between camera and IMU clocks
 * @property lastFrameTimestampNs Last processed frame timestamp
 * @property imuSampleCount Number of IMU samples in buffer
 * @property averageImuRate Measured IMU sampling rate in Hz
 * @property warningMessage Warning message if any issue detected
 */
data class SyncStatus(
    val isHealthy: Boolean,
    val timeDriftNs: Long = 0,
    val lastFrameTimestampNs: Long = 0,
    val imuSampleCount: Int = 0,
    val averageImuRate: Double = 0.0,
    val warningMessage: String? = null
)

/**
 * Timestamp Synchronizer for camera-IMU data alignment.
 *
 * This class aligns camera and IMU timestamps to a unified timeline,
 * performs IMU interpolation at frame timestamps, and detects
 * synchronization issues.
 *
 * Features:
 * - Linear interpolation of IMU samples at camera frame timestamps
 * - Retrieval of IMU samples between consecutive frames
 * - Clock drift detection and monitoring
 * - Handles variable IMU sampling rates
 * - Thread-safe operations
 *
 * Usage:
 * ```kotlin
 * val synchronizer = TimestampSynchronizer(imuBuffer)
 * val syncData = synchronizer.associateIMUWithFrame(frameTimestamp, frameSeq)
 * val interpolated = syncData.interpolatedImu
 * ```
 *
 * @property imuBuffer IMU circular buffer containing recent samples
 * @property maxTimeDriftMs Maximum allowed time drift before warning (default: 50ms)
 */
class TimestampSynchronizer(
    private val imuBuffer: IMUCircularBuffer,
    private val maxTimeDriftMs: Long = 50
) {
    companion object {
        private const val TAG = "TimestampSynchronizer"

        /** Maximum time window to search for IMU samples (100ms) */
        private const val MAX_SEARCH_WINDOW_NS = 100_000_000L

        /** Minimum expected IMU rate for health check (Hz) */
        private const val MIN_EXPECTED_IMU_RATE = 100.0

        /** Expected IMU rate for drift calculation (Hz) */
        private const val EXPECTED_IMU_RATE = 200.0
    }

    private var lastFrameTimestampNs: Long = 0
    private var lastImuTimestampNs: Long = 0
    private var frameCount: Long = 0
    private var cumulativeDriftNs: Long = 0

    private val lock = Any()

    /**
     * Associate IMU samples with a camera frame.
     *
     * Retrieves IMU samples around the frame timestamp and performs
     * interpolation to get IMU values at the exact frame time.
     *
     * @param frameTimestampNs Camera frame timestamp in nanoseconds
     * @param frameSequence Frame sequence number for tracking
     * @param windowBeforeNs Time window before frame to collect IMU samples (default: 50ms)
     * @param windowAfterNs Time window after frame to collect IMU samples (default: 10ms)
     * @return SynchronizedData containing associated IMU samples and interpolated values
     */
    fun associateIMUWithFrame(
        frameTimestampNs: Long,
        frameSequence: Long,
        windowBeforeNs: Long = 50_000_000L,
        windowAfterNs: Long = 10_000_000L
    ): SynchronizedData = synchronized(lock) {
        val startTime = System.nanoTime()

        // Calculate time range for IMU sample retrieval
        val rangeStart = frameTimestampNs - windowBeforeNs
        val rangeEnd = frameTimestampNs + windowAfterNs

        // Get all IMU samples in the time range
        val allSamples = imuBuffer.getSamples(rangeStart, rangeEnd)

        // Separate accelerometer and gyroscope samples
        val accelSamples = allSamples.filter { it.type == SensorType.ACCELEROMETER }
        val gyroSamples = allSamples.filter { it.type == SensorType.GYROSCOPE }

        // Split samples into before and after frame timestamp
        val samplesBefore = allSamples.filter { it.timestampNs < frameTimestampNs }
        val samplesAfter = allSamples.filter { it.timestampNs >= frameTimestampNs }

        // Perform interpolation at frame timestamp
        val interpolated = interpolateAtTimestamp(
            frameTimestampNs,
            accelSamples,
            gyroSamples
        )

        // Update tracking state
        lastFrameTimestampNs = frameTimestampNs
        frameCount++

        // Log performance if slow
        val processingTimeMs = (System.nanoTime() - startTime) / 1_000_000.0
        if (processingTimeMs > 1.0) {
            Log.w(TAG, "Synchronization took ${processingTimeMs}ms (exceeds 1ms target)")
        }

        SynchronizedData(
            frameTimestampNs = frameTimestampNs,
            frameSequence = frameSequence,
            imuSamplesBefore = samplesBefore,
            imuSamplesAfter = samplesAfter,
            interpolatedImu = interpolated
        )
    }

    /**
     * Interpolate IMU values at a specific timestamp.
     *
     * Uses linear interpolation between the nearest samples before and after
     * the target timestamp. Returns null if insufficient samples are available.
     *
     * @param targetTimestampNs Target timestamp for interpolation
     * @param accelSamples Accelerometer samples (sorted by timestamp)
     * @param gyroSamples Gyroscope samples (sorted by timestamp)
     * @return InterpolatedIMU or null if interpolation is not possible
     */
    private fun interpolateAtTimestamp(
        targetTimestampNs: Long,
        accelSamples: List<IMUSample>,
        gyroSamples: List<IMUSample>
    ): InterpolatedIMU? {
        // Find accelerometer samples for interpolation
        val accelBefore = accelSamples.lastOrNull { it.timestampNs <= targetTimestampNs }
        val accelAfter = accelSamples.firstOrNull { it.timestampNs > targetTimestampNs }

        // Find gyroscope samples for interpolation
        val gyroBefore = gyroSamples.lastOrNull { it.timestampNs <= targetTimestampNs }
        val gyroAfter = gyroSamples.firstOrNull { it.timestampNs > targetTimestampNs }

        // Check if we have enough samples for interpolation
        if (accelBefore == null || accelAfter == null ||
            gyroBefore == null || gyroAfter == null) {
            Log.d(TAG, "Insufficient samples for interpolation at $targetTimestampNs")
            return null
        }

        // Calculate interpolation factors
        val accelAlpha = calculateInterpolationFactor(
            targetTimestampNs,
            accelBefore.timestampNs,
            accelAfter.timestampNs
        )

        val gyroAlpha = calculateInterpolationFactor(
            targetTimestampNs,
            gyroBefore.timestampNs,
            gyroAfter.timestampNs
        )

        // Perform linear interpolation
        return InterpolatedIMU(
            timestampNs = targetTimestampNs,
            ax = lerp(accelBefore.x, accelAfter.x, accelAlpha),
            ay = lerp(accelBefore.y, accelAfter.y, accelAlpha),
            az = lerp(accelBefore.z, accelAfter.z, accelAlpha),
            gx = lerp(gyroBefore.x, gyroAfter.x, gyroAlpha),
            gy = lerp(gyroBefore.y, gyroAfter.y, gyroAlpha),
            gz = lerp(gyroBefore.z, gyroAfter.z, gyroAlpha)
        )
    }

    /**
     * Interpolate single IMU sample between two samples.
     *
     * Public API for manual interpolation when needed.
     *
     * @param targetTimestampNs Target timestamp in nanoseconds
     * @param before Sample before target timestamp
     * @param after Sample after target timestamp
     * @return Interpolated IMUSample
     * @throws IllegalArgumentException if samples have different types or invalid timestamps
     */
    fun interpolateIMU(
        targetTimestampNs: Long,
        before: IMUSample,
        after: IMUSample
    ): IMUSample {
        require(before.type == after.type) {
            "Cannot interpolate samples of different types: ${before.type} vs ${after.type}"
        }
        require(before.timestampNs <= targetTimestampNs) {
            "Before sample timestamp (${before.timestampNs}) must be <= target ($targetTimestampNs)"
        }
        require(after.timestampNs >= targetTimestampNs) {
            "After sample timestamp (${after.timestampNs}) must be >= target ($targetTimestampNs)"
        }

        val alpha = calculateInterpolationFactor(
            targetTimestampNs,
            before.timestampNs,
            after.timestampNs
        )

        return IMUSample(
            timestampNs = targetTimestampNs,
            type = before.type,
            x = lerp(before.x, after.x, alpha),
            y = lerp(before.y, after.y, alpha),
            z = lerp(before.z, after.z, alpha)
        )
    }

    /**
     * Get IMU samples between two consecutive frame timestamps.
     *
     * Useful for SLAM algorithms that need all IMU samples between frames.
     *
     * @param prevFrameTimestampNs Previous frame timestamp
     * @param currFrameTimestampNs Current frame timestamp
     * @return List of IMU samples between the two frames (exclusive of prev, inclusive of curr)
     */
    fun getIMUSamplesBetweenFrames(
        prevFrameTimestampNs: Long,
        currFrameTimestampNs: Long
    ): List<IMUSample> {
        require(currFrameTimestampNs > prevFrameTimestampNs) {
            "Current frame timestamp must be > previous frame timestamp"
        }

        return imuBuffer.getSamples(prevFrameTimestampNs + 1, currFrameTimestampNs)
    }

    /**
     * Get current synchronization status.
     *
     * Monitors the health of camera-IMU synchronization and detects issues.
     *
     * @return SyncStatus with current synchronization state
     */
    fun getSyncStatus(): SyncStatus = synchronized(lock) {
        val allSamples = imuBuffer.getAllSamples()
        val sampleCount = allSamples.size

        // Calculate IMU rate from recent samples
        val imuRate = calculateIMURate(allSamples)

        // Detect issues
        val warnings = mutableListOf<String>()

        if (sampleCount < 10) {
            warnings.add("Low IMU sample count: $sampleCount")
        }

        if (imuRate < MIN_EXPECTED_IMU_RATE && sampleCount > 10) {
            warnings.add("Low IMU rate: ${String.format("%.1f", imuRate)} Hz (expected >= $MIN_EXPECTED_IMU_RATE Hz)")
        }

        // Check for timestamp discontinuities
        val discontinuity = detectTimestampDiscontinuity(allSamples)
        if (discontinuity != null) {
            warnings.add("Timestamp discontinuity detected: ${discontinuity / 1_000_000}ms gap")
        }

        // Estimate clock drift
        val driftNs = estimateClockDrift(allSamples)
        if (abs(driftNs) > maxTimeDriftMs * 1_000_000) {
            warnings.add("High clock drift: ${driftNs / 1_000_000}ms")
        }

        val isHealthy = warnings.isEmpty()
        val warningMessage = if (warnings.isNotEmpty()) {
            warnings.joinToString("; ")
        } else {
            null
        }

        SyncStatus(
            isHealthy = isHealthy,
            timeDriftNs = driftNs,
            lastFrameTimestampNs = lastFrameTimestampNs,
            imuSampleCount = sampleCount,
            averageImuRate = imuRate,
            warningMessage = warningMessage
        )
    }

    /**
     * Reset synchronizer state.
     *
     * Call this when starting a new recording session.
     */
    fun reset() = synchronized(lock) {
        lastFrameTimestampNs = 0
        lastImuTimestampNs = 0
        frameCount = 0
        cumulativeDriftNs = 0
        Log.d(TAG, "Synchronizer reset")
    }

    /**
     * Calculate interpolation factor (alpha) for linear interpolation.
     *
     * @return Value in range [0.0, 1.0] where 0.0 = before, 1.0 = after
     */
    private fun calculateInterpolationFactor(
        target: Long,
        before: Long,
        after: Long
    ): Float {
        if (after == before) return 0.5f
        return ((target - before).toFloat() / (after - before).toFloat()).coerceIn(0f, 1f)
    }

    /**
     * Linear interpolation between two values.
     */
    private fun lerp(a: Float, b: Float, alpha: Float): Float {
        return a + alpha * (b - a)
    }

    /**
     * Calculate average IMU sampling rate from samples.
     */
    private fun calculateIMURate(samples: List<IMUSample>): Double {
        if (samples.size < 2) return 0.0

        // Use accelerometer samples for rate calculation
        val accelSamples = samples.filter { it.type == SensorType.ACCELEROMETER }
        if (accelSamples.size < 2) return 0.0

        val firstTs = accelSamples.first().timestampNs
        val lastTs = accelSamples.last().timestampNs
        val durationNs = lastTs - firstTs

        if (durationNs <= 0) return 0.0

        val durationSec = durationNs / 1_000_000_000.0
        return (accelSamples.size - 1) / durationSec
    }

    /**
     * Detect timestamp discontinuities (clock jumps).
     *
     * @return Size of discontinuity in nanoseconds, or null if none detected
     */
    private fun detectTimestampDiscontinuity(samples: List<IMUSample>): Long? {
        if (samples.size < 2) return null

        val maxExpectedGapNs = (1_000_000_000.0 / EXPECTED_IMU_RATE * 3).toLong()

        for (i in 1 until samples.size) {
            val gap = samples[i].timestampNs - samples[i - 1].timestampNs
            if (gap > maxExpectedGapNs || gap < 0) {
                return gap
            }
        }
        return null
    }

    /**
     * Estimate clock drift between camera and IMU.
     *
     * This is a simplified estimation based on sample timing regularity.
     */
    private fun estimateClockDrift(samples: List<IMUSample>): Long {
        if (samples.size < 10) return 0L

        val accelSamples = samples.filter { it.type == SensorType.ACCELEROMETER }
        if (accelSamples.size < 10) return 0L

        // Calculate expected vs actual duration
        val expectedIntervalNs = (1_000_000_000.0 / EXPECTED_IMU_RATE).toLong()
        val actualDurationNs = accelSamples.last().timestampNs - accelSamples.first().timestampNs
        val expectedDurationNs = expectedIntervalNs * (accelSamples.size - 1)

        return actualDurationNs - expectedDurationNs
    }
}
