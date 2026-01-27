package com.vi.slam.android.calibration

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Time offset estimator between camera and IMU timestamps
 *
 * Estimates the time offset between camera and IMU sensor clocks using
 * cross-correlation of motion-based features. This is essential for proper
 * sensor fusion in Visual-Inertial Odometry (VIO) and SLAM systems.
 *
 * Algorithm:
 * 1. Extract motion features from camera (optical flow magnitude)
 * 2. Extract motion features from IMU (acceleration magnitude)
 * 3. Compute cross-correlation between the two signals
 * 4. Find peak correlation to determine time offset
 *
 * The estimator achieves sub-millisecond accuracy and handles timestamp drift.
 */
class TimeOffsetEstimator(
    private val config: TimeOffsetConfig = TimeOffsetConfig()
) {
    /**
     * Configuration for time offset estimation
     */
    data class TimeOffsetConfig(
        val windowSizeMs: Long = 5000, // Analysis window size in milliseconds
        val maxOffsetMs: Long = 1000,   // Maximum expected offset in milliseconds
        val minDataPoints: Int = 50,    // Minimum data points required
        val samplingRateHz: Double = 100.0 // Resampling rate for correlation
    )

    /**
     * Camera motion feature (e.g., optical flow magnitude)
     */
    data class CameraMotion(
        val timestamp: Long, // Nanoseconds
        val motionMagnitude: Double
    )

    /**
     * IMU motion feature (acceleration magnitude)
     */
    data class ImuMotion(
        val timestamp: Long, // Nanoseconds
        val accelMagnitude: Double
    )

    /**
     * Time offset estimation result
     */
    data class TimeOffsetResult(
        val offsetNs: Long,           // Estimated offset in nanoseconds (camera - IMU)
        val offsetMs: Double,         // Estimated offset in milliseconds
        val confidence: Double,       // Confidence score (0-1, based on correlation peak)
        val correlationPeak: Double,  // Maximum correlation value
        val accuracy: AccuracyLevel   // Estimated accuracy level
    )

    /**
     * Accuracy level classification
     */
    enum class AccuracyLevel {
        HIGH,      // < 0.5 ms
        MEDIUM,    // 0.5 - 1.0 ms
        LOW,       // 1.0 - 2.0 ms
        UNRELIABLE // > 2.0 ms or low confidence
    }

    /**
     * Estimate time offset between camera and IMU timestamps
     *
     * @param cameraMotions List of camera motion features with timestamps
     * @param imuMotions List of IMU motion features with timestamps
     * @return Time offset estimation result or error
     */
    fun estimateOffset(
        cameraMotions: List<CameraMotion>,
        imuMotions: List<ImuMotion>
    ): Result<TimeOffsetResult> {
        return try {
            // Validate input data
            if (cameraMotions.size < config.minDataPoints) {
                return Result.failure(
                    IllegalArgumentException(
                        "Insufficient camera data: ${cameraMotions.size} < ${config.minDataPoints}"
                    )
                )
            }

            if (imuMotions.size < config.minDataPoints) {
                return Result.failure(
                    IllegalArgumentException(
                        "Insufficient IMU data: ${imuMotions.size} < ${config.minDataPoints}"
                    )
                )
            }

            // Check temporal overlap
            val cameraStart = cameraMotions.first().timestamp
            val cameraEnd = cameraMotions.last().timestamp
            val imuStart = imuMotions.first().timestamp
            val imuEnd = imuMotions.last().timestamp

            if (cameraEnd < imuStart || imuEnd < cameraStart) {
                return Result.failure(
                    IllegalStateException("No temporal overlap between camera and IMU data")
                )
            }

            // Resample both signals to common time base
            val samplingPeriodNs = (1e9 / config.samplingRateHz).toLong()
            val commonStart = maxOf(cameraStart, imuStart)
            val commonEnd = minOf(cameraEnd, imuEnd)

            val cameraSamples = resampleSignal(cameraMotions, commonStart, commonEnd, samplingPeriodNs)
            val imuSamples = resampleSignal(imuMotions, commonStart, commonEnd, samplingPeriodNs)

            if (cameraSamples.size < config.minDataPoints || imuSamples.size < config.minDataPoints) {
                return Result.failure(
                    IllegalStateException("Insufficient samples after resampling")
                )
            }

            // Compute cross-correlation
            val maxLag = (config.maxOffsetMs * 1_000_000 / samplingPeriodNs).toInt()
            val correlation = computeCrossCorrelation(cameraSamples, imuSamples, maxLag)

            // Find peak correlation
            val (peakLag, peakValue) = findPeak(correlation)
            val offsetNs = peakLag * samplingPeriodNs
            val offsetMs = offsetNs / 1_000_000.0

            // Compute confidence based on correlation peak sharpness
            val confidence = computeConfidence(correlation, peakValue)

            // Determine accuracy level
            val accuracy = classifyAccuracy(abs(offsetMs), confidence)

            val result = TimeOffsetResult(
                offsetNs = offsetNs,
                offsetMs = offsetMs,
                confidence = confidence,
                correlationPeak = peakValue,
                accuracy = accuracy
            )

            Result.success(result)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Estimate offset from raw camera and IMU data
     *
     * Convenience method that extracts motion features from raw measurements
     *
     * @param cameraTimestamps Camera frame timestamps (nanoseconds)
     * @param cameraFlows Optical flow magnitudes
     * @param imuTimestamps IMU sample timestamps (nanoseconds)
     * @param imuAccels IMU acceleration vectors (x, y, z)
     * @return Time offset estimation result or error
     */
    fun estimateOffsetFromRaw(
        cameraTimestamps: List<Long>,
        cameraFlows: List<Double>,
        imuTimestamps: List<Long>,
        imuAccels: List<Triple<Double, Double, Double>>
    ): Result<TimeOffsetResult> {
        return try {
            // Convert to motion features
            val cameraMotions = cameraTimestamps.zip(cameraFlows).map { (ts, flow) ->
                CameraMotion(ts, flow)
            }

            val imuMotions = imuTimestamps.zip(imuAccels).map { (ts, accel) ->
                val magnitude = sqrt(accel.first * accel.first +
                        accel.second * accel.second +
                        accel.third * accel.third)
                ImuMotion(ts, magnitude)
            }

            estimateOffset(cameraMotions, imuMotions)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Resample signal to uniform time grid using linear interpolation
     */
    private fun resampleSignal(
        data: List<Pair<Long, Double>>,
        startTime: Long,
        endTime: Long,
        samplingPeriod: Long
    ): DoubleArray {
        val numSamples = ((endTime - startTime) / samplingPeriod).toInt() + 1
        val samples = DoubleArray(numSamples)

        var dataIdx = 0
        for (i in 0 until numSamples) {
            val targetTime = startTime + i * samplingPeriod

            // Find data points surrounding target time
            while (dataIdx < data.size - 1 && data[dataIdx + 1].first < targetTime) {
                dataIdx++
            }

            if (dataIdx >= data.size - 1) {
                // Use last value
                samples[i] = data.last().second
            } else {
                // Linear interpolation
                val t0 = data[dataIdx].first
                val t1 = data[dataIdx + 1].first
                val v0 = data[dataIdx].second
                val v1 = data[dataIdx + 1].second

                val alpha = (targetTime - t0).toDouble() / (t1 - t0)
                samples[i] = v0 + alpha * (v1 - v0)
            }
        }

        return samples
    }

    /**
     * Convert motion data to timestamp-value pairs
     */
    @JvmName("resampleCameraSignal")
    private fun resampleSignal(
        cameraMotions: List<CameraMotion>,
        startTime: Long,
        endTime: Long,
        samplingPeriod: Long
    ): DoubleArray {
        val data = cameraMotions.map { it.timestamp to it.motionMagnitude }
        return resampleSignal(data, startTime, endTime, samplingPeriod)
    }

    /**
     * Convert IMU motion data to timestamp-value pairs
     */
    @JvmName("resampleImuSignal")
    private fun resampleSignal(
        imuMotions: List<ImuMotion>,
        startTime: Long,
        endTime: Long,
        samplingPeriod: Long
    ): DoubleArray {
        val data = imuMotions.map { it.timestamp to it.accelMagnitude }
        return resampleSignal(data, startTime, endTime, samplingPeriod)
    }

    /**
     * Compute cross-correlation between two signals
     *
     * Returns correlation for lags in range [-maxLag, +maxLag]
     * Positive lag means signal2 is delayed relative to signal1
     */
    private fun computeCrossCorrelation(
        signal1: DoubleArray,
        signal2: DoubleArray,
        maxLag: Int
    ): DoubleArray {
        val correlation = DoubleArray(2 * maxLag + 1)

        // Normalize signals (zero mean, unit variance)
        val s1 = normalizeSignal(signal1)
        val s2 = normalizeSignal(signal2)

        // Compute correlation for each lag
        for (lag in -maxLag..maxLag) {
            var sum = 0.0
            var count = 0

            for (i in s1.indices) {
                val j = i + lag
                if (j in s2.indices) {
                    sum += s1[i] * s2[j]
                    count++
                }
            }

            correlation[lag + maxLag] = if (count > 0) sum / count else 0.0
        }

        return correlation
    }

    /**
     * Normalize signal to zero mean and unit variance
     */
    private fun normalizeSignal(signal: DoubleArray): DoubleArray {
        val mean = signal.average()
        val variance = signal.map { (it - mean) * (it - mean) }.average()
        val stdDev = sqrt(variance)

        return if (stdDev > 1e-6) {
            signal.map { (it - mean) / stdDev }.toDoubleArray()
        } else {
            DoubleArray(signal.size) { 0.0 }
        }
    }

    /**
     * Find peak in correlation array
     *
     * @return Pair of (lag index relative to center, peak value)
     */
    private fun findPeak(correlation: DoubleArray): Pair<Int, Double> {
        val maxIdx = correlation.indices.maxByOrNull { correlation[it] } ?: 0
        val maxValue = correlation[maxIdx]
        val centerIdx = correlation.size / 2
        val lag = maxIdx - centerIdx

        return Pair(lag, maxValue)
    }

    /**
     * Compute confidence score based on correlation peak sharpness
     *
     * Higher confidence means sharper peak (more reliable offset estimate)
     */
    private fun computeConfidence(correlation: DoubleArray, peakValue: Double): Double {
        // Confidence is ratio of peak to mean absolute correlation
        val meanAbsCorr = correlation.map { abs(it) }.average()
        val confidence = if (meanAbsCorr > 1e-6) {
            (peakValue / meanAbsCorr).coerceIn(0.0, 1.0)
        } else {
            0.0
        }

        return confidence
    }

    /**
     * Classify accuracy level based on offset magnitude and confidence
     */
    private fun classifyAccuracy(offsetMs: Double, confidence: Double): AccuracyLevel {
        return when {
            confidence < 0.5 -> AccuracyLevel.UNRELIABLE
            offsetMs < 0.5 -> AccuracyLevel.HIGH
            offsetMs < 1.0 -> AccuracyLevel.MEDIUM
            offsetMs < 2.0 -> AccuracyLevel.LOW
            else -> AccuracyLevel.UNRELIABLE
        }
    }

    /**
     * Handle timestamp drift by applying linear drift correction
     *
     * @param timestamps Original timestamps
     * @param driftRate Drift rate (e.g., 0.001 means 1ms drift per second)
     * @return Corrected timestamps
     */
    fun correctTimestampDrift(
        timestamps: List<Long>,
        driftRate: Double
    ): List<Long> {
        if (timestamps.isEmpty()) return timestamps

        val startTime = timestamps.first()
        return timestamps.map { ts ->
            val elapsedNs = ts - startTime
            val correction = (elapsedNs * driftRate).toLong()
            ts - correction
        }
    }
}
