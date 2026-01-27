package com.vi.slam.android.calibration

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

/**
 * Unit tests for TimeOffsetEstimator
 */
class TimeOffsetEstimatorTest {

    private lateinit var estimator: TimeOffsetEstimator

    @Before
    fun setUp() {
        estimator = TimeOffsetEstimator()
    }

    @Test
    fun testEstimateOffsetWithSyntheticData() {
        // Generate synthetic motion data with known offset
        val knownOffsetMs = 50.0 // 50ms offset
        val knownOffsetNs = (knownOffsetMs * 1_000_000).toLong()

        val cameraMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val imuMotions = generateSyntheticMotion(
            startTimeNs = knownOffsetNs, // IMU delayed by 50ms
            durationMs = 2000,
            frequencyHz = 1.0
        )

        // Estimate offset
        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isSuccess)
        val offset = result.getOrNull()
        assertNotNull(offset)

        // Verify offset is close to known value (within 5ms tolerance)
        val errorMs = kotlin.math.abs(offset!!.offsetMs - knownOffsetMs)
        assertTrue("Offset error $errorMs ms exceeds 5ms threshold", errorMs < 5.0)

        // Verify accuracy classification
        assertTrue(
            "Expected HIGH or MEDIUM accuracy, got ${offset.accuracy}",
            offset.accuracy == TimeOffsetEstimator.AccuracyLevel.HIGH ||
                    offset.accuracy == TimeOffsetEstimator.AccuracyLevel.MEDIUM
        )
    }

    @Test
    fun testEstimateOffsetWithZeroOffset() {
        // Generate synchronized motion data (zero offset)
        val cameraMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val imuMotions = generateSyntheticMotion(
            startTimeNs = 0L, // No offset
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isSuccess)
        val offset = result.getOrNull()
        assertNotNull(offset)

        // Verify offset is close to zero (within 5ms tolerance)
        assertTrue(
            "Offset ${offset!!.offsetMs} ms should be near zero",
            kotlin.math.abs(offset.offsetMs) < 5.0
        )
    }

    @Test
    fun testEstimateOffsetWithNegativeOffset() {
        // Generate motion data where camera is delayed relative to IMU
        val knownOffsetMs = -30.0 // Camera delayed by 30ms
        val knownOffsetNs = (knownOffsetMs * 1_000_000).toLong()

        val cameraMotions = generateSyntheticMotion(
            startTimeNs = -knownOffsetNs, // Camera delayed
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val imuMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isSuccess)
        val offset = result.getOrNull()
        assertNotNull(offset)

        // Verify offset is close to known value (within 5ms tolerance)
        val errorMs = kotlin.math.abs(offset!!.offsetMs - knownOffsetMs)
        assertTrue("Offset error $errorMs ms exceeds 5ms threshold", errorMs < 5.0)
    }

    @Test
    fun testEstimateOffsetInsufficientCameraData() {
        val cameraMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 100, // Too short
            frequencyHz = 1.0
        ).take(10) // Less than minimum required

        val imuMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalArgumentException)
        assertTrue(result.exceptionOrNull()?.message?.contains("camera data") == true)
    }

    @Test
    fun testEstimateOffsetInsufficientImuData() {
        val cameraMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 2000,
            frequencyHz = 1.0
        )

        val imuMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 100, // Too short
            frequencyHz = 1.0
        ).take(10) // Less than minimum required

        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalArgumentException)
        assertTrue(result.exceptionOrNull()?.message?.contains("IMU data") == true)
    }

    @Test
    fun testEstimateOffsetNoTemporalOverlap() {
        // Camera data from 0-1000ms
        val cameraMotions = generateSyntheticMotion(
            startTimeNs = 0L,
            durationMs = 1000,
            frequencyHz = 1.0
        )

        // IMU data from 2000-3000ms (no overlap)
        val imuMotions = generateSyntheticMotion(
            startTimeNs = 2_000_000_000L,
            durationMs = 1000,
            frequencyHz = 1.0
        )

        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
        assertTrue(result.exceptionOrNull()?.message?.contains("temporal overlap") == true)
    }

    @Test
    fun testEstimateOffsetFromRaw() {
        // Generate raw camera and IMU data
        val numSamples = 200
        val knownOffsetMs = 40.0
        val knownOffsetNs = (knownOffsetMs * 1_000_000).toLong()

        val cameraTimestamps = mutableListOf<Long>()
        val cameraFlows = mutableListOf<Double>()

        for (i in 0 until numSamples) {
            val t = i * 10_000_000L // 10ms intervals
            cameraTimestamps.add(t)
            cameraFlows.add(generateMotionValue(t, 1.0))
        }

        val imuTimestamps = mutableListOf<Long>()
        val imuAccels = mutableListOf<Triple<Double, Double, Double>>()

        for (i in 0 until numSamples * 10) { // Higher IMU rate
            val t = knownOffsetNs + i * 1_000_000L // 1ms intervals
            imuTimestamps.add(t)
            val mag = generateMotionValue(t - knownOffsetNs, 1.0)
            // Create acceleration vector with known magnitude
            imuAccels.add(Triple(mag / 1.732, mag / 1.732, mag / 1.732))
        }

        val result = estimator.estimateOffsetFromRaw(
            cameraTimestamps,
            cameraFlows,
            imuTimestamps,
            imuAccels
        )

        assertTrue(result.isSuccess)
        val offset = result.getOrNull()
        assertNotNull(offset)

        // Verify offset is close to known value (within 10ms tolerance for raw data)
        val errorMs = kotlin.math.abs(offset!!.offsetMs - knownOffsetMs)
        assertTrue("Offset error $errorMs ms exceeds 10ms threshold", errorMs < 10.0)
    }

    @Test
    fun testCorrectTimestampDrift() {
        val timestamps = listOf(
            0L,
            1_000_000_000L,  // 1 second
            2_000_000_000L,  // 2 seconds
            3_000_000_000L   // 3 seconds
        )

        val driftRate = 0.001 // 1ms per second

        val corrected = estimator.correctTimestampDrift(timestamps, driftRate)

        assertEquals(timestamps.size, corrected.size)

        // First timestamp should be unchanged
        assertEquals(timestamps[0], corrected[0])

        // Check drift correction
        for (i in 1 until timestamps.size) {
            val elapsed = timestamps[i] - timestamps[0]
            val expectedCorrection = (elapsed * driftRate).toLong()
            val expectedCorrected = timestamps[i] - expectedCorrection

            val error = kotlin.math.abs(corrected[i] - expectedCorrected)
            assertTrue("Drift correction error at index $i", error < 1000) // Within 1 microsecond
        }
    }

    @Test
    fun testCorrectTimestampDriftEmptyList() {
        val timestamps = emptyList<Long>()
        val corrected = estimator.correctTimestampDrift(timestamps, 0.001)

        assertTrue(corrected.isEmpty())
    }

    @Test
    fun testAccuracyClassification() {
        // Test with high accuracy data
        val highAccuracyMotions = generateSyntheticMotion(0L, 3000, 2.0)
        val result = estimator.estimateOffset(highAccuracyMotions, highAccuracyMotions)

        assertTrue(result.isSuccess)
        val offset = result.getOrNull()
        assertNotNull(offset)

        // Should have high confidence for perfectly synchronized data
        assertTrue("Expected high confidence, got ${offset!!.confidence}", offset.confidence > 0.5)
    }

    @Test
    fun testLargeOffset() {
        // Test with large offset near maximum
        val largeOffsetMs = 900.0 // 900ms
        val largeOffsetNs = (largeOffsetMs * 1_000_000).toLong()

        val cameraMotions = generateSyntheticMotion(0L, 3000, 1.0)
        val imuMotions = generateSyntheticMotion(largeOffsetNs, 3000, 1.0)

        val result = estimator.estimateOffset(cameraMotions, imuMotions)

        assertTrue(result.isSuccess)
        val offset = result.getOrNull()
        assertNotNull(offset)

        // Verify we can detect large offsets
        val errorMs = kotlin.math.abs(offset!!.offsetMs - largeOffsetMs)
        assertTrue("Large offset error $errorMs ms exceeds 50ms threshold", errorMs < 50.0)
    }

    /**
     * Generate synthetic motion data with sinusoidal pattern
     *
     * @param startTimeNs Start time in nanoseconds
     * @param durationMs Duration in milliseconds
     * @param frequencyHz Motion frequency in Hz
     * @return List of camera motion samples
     */
    private fun generateSyntheticMotion(
        startTimeNs: Long,
        durationMs: Int,
        frequencyHz: Double
    ): List<TimeOffsetEstimator.CameraMotion> {
        val samples = mutableListOf<TimeOffsetEstimator.CameraMotion>()
        val samplingPeriodNs = 10_000_000L // 10ms = 100 Hz

        val numSamples = (durationMs * 1_000_000 / samplingPeriodNs).toInt()

        for (i in 0 until numSamples) {
            val t = startTimeNs + i * samplingPeriodNs
            val magnitude = generateMotionValue(t, frequencyHz)

            samples.add(
                TimeOffsetEstimator.CameraMotion(
                    timestamp = t,
                    motionMagnitude = magnitude
                )
            )
        }

        return samples
    }

    /**
     * Generate synthetic motion value using sinusoidal pattern
     */
    private fun generateMotionValue(timeNs: Long, frequencyHz: Double): Double {
        val timeSeconds = timeNs / 1e9
        val omega = 2.0 * Math.PI * frequencyHz
        return kotlin.math.abs(sin(omega * timeSeconds))
    }
}
