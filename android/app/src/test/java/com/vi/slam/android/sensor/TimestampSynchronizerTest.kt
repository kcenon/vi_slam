package com.vi.slam.android.sensor

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import kotlin.math.abs

class TimestampSynchronizerTest {

    private lateinit var buffer: IMUCircularBuffer
    private lateinit var synchronizer: TimestampSynchronizer

    @Before
    fun setUp() {
        buffer = IMUCircularBuffer(capacitySamples = 1000)
        synchronizer = TimestampSynchronizer(buffer)
    }

    private fun createAccelSample(timestampNs: Long, x: Float = 0f, y: Float = 0f, z: Float = 9.8f): IMUSample {
        return IMUSample(
            timestampNs = timestampNs,
            type = SensorType.ACCELEROMETER,
            x = x,
            y = y,
            z = z
        )
    }

    private fun createGyroSample(timestampNs: Long, x: Float = 0f, y: Float = 0f, z: Float = 0f): IMUSample {
        return IMUSample(
            timestampNs = timestampNs,
            type = SensorType.GYROSCOPE,
            x = x,
            y = y,
            z = z
        )
    }

    /**
     * Populate buffer with IMU samples at specified rate.
     * Adds both accelerometer and gyroscope samples.
     */
    private fun populateBuffer(
        startNs: Long,
        endNs: Long,
        rateHz: Int = 200
    ) {
        val intervalNs = 1_000_000_000L / rateHz
        var ts = startNs

        while (ts <= endNs) {
            // Add accelerometer sample
            buffer.add(createAccelSample(ts, 0.1f, 0.2f, 9.8f))
            // Add gyroscope sample
            buffer.add(createGyroSample(ts, 0.01f, 0.02f, 0.03f))
            ts += intervalNs
        }
    }

    // ===========================================
    // associateIMUWithFrame Tests
    // ===========================================

    @Test
    fun `associateIMUWithFrame returns synchronized data`() {
        // Given: IMU samples from 0 to 100ms at 200Hz
        populateBuffer(0L, 100_000_000L)

        // When: Associate IMU with frame at 50ms
        val frameTimestamp = 50_000_000L
        val result = synchronizer.associateIMUWithFrame(frameTimestamp, 1)

        // Then: Result contains valid data
        assertEquals(frameTimestamp, result.frameTimestampNs)
        assertEquals(1L, result.frameSequence)
        assertTrue(result.imuSamplesBefore.isNotEmpty())
        assertTrue(result.imuSamplesAfter.isNotEmpty())
    }

    @Test
    fun `associateIMUWithFrame splits samples correctly before and after`() {
        // Given: IMU samples
        populateBuffer(0L, 100_000_000L)

        // When: Associate with frame at 50ms
        val frameTimestamp = 50_000_000L
        val result = synchronizer.associateIMUWithFrame(frameTimestamp, 1)

        // Then: Samples are correctly split
        assertTrue(result.imuSamplesBefore.all { it.timestampNs < frameTimestamp })
        assertTrue(result.imuSamplesAfter.all { it.timestampNs >= frameTimestamp })
    }

    @Test
    fun `associateIMUWithFrame returns interpolated IMU`() {
        // Given: IMU samples
        populateBuffer(0L, 100_000_000L)

        // When: Associate with frame at 50ms
        val frameTimestamp = 50_000_000L
        val result = synchronizer.associateIMUWithFrame(frameTimestamp, 1)

        // Then: Interpolated IMU is present
        assertNotNull(result.interpolatedImu)
        assertEquals(frameTimestamp, result.interpolatedImu!!.timestampNs)
    }

    @Test
    fun `associateIMUWithFrame handles empty buffer`() {
        // Given: Empty buffer

        // When: Associate with frame
        val result = synchronizer.associateIMUWithFrame(50_000_000L, 1)

        // Then: Result has empty lists and no interpolation
        assertTrue(result.imuSamplesBefore.isEmpty())
        assertTrue(result.imuSamplesAfter.isEmpty())
        assertNull(result.interpolatedImu)
    }

    @Test
    fun `associateIMUWithFrame respects time window parameters`() {
        // Given: IMU samples from 0 to 100ms
        populateBuffer(0L, 100_000_000L)

        // When: Use custom window (10ms before, 5ms after)
        val frameTimestamp = 50_000_000L
        val result = synchronizer.associateIMUWithFrame(
            frameTimestamp, 1,
            windowBeforeNs = 10_000_000L,
            windowAfterNs = 5_000_000L
        )

        // Then: Samples are within the window
        val minTimestamp = frameTimestamp - 10_000_000L
        val maxTimestamp = frameTimestamp + 5_000_000L
        assertTrue(result.imuSamplesBefore.all { it.timestampNs >= minTimestamp })
        assertTrue(result.imuSamplesAfter.all { it.timestampNs <= maxTimestamp })
    }

    // ===========================================
    // interpolateIMU Tests
    // ===========================================

    @Test
    fun `interpolateIMU performs linear interpolation correctly`() {
        // Given: Two accelerometer samples
        val before = createAccelSample(1000L, 0f, 0f, 10f)
        val after = createAccelSample(2000L, 10f, 10f, 20f)

        // When: Interpolate at midpoint
        val result = synchronizer.interpolateIMU(1500L, before, after)

        // Then: Values are linearly interpolated
        assertEquals(1500L, result.timestampNs)
        assertEquals(SensorType.ACCELEROMETER, result.type)
        assertEquals(5f, result.x, 0.001f)
        assertEquals(5f, result.y, 0.001f)
        assertEquals(15f, result.z, 0.001f)
    }

    @Test
    fun `interpolateIMU at before timestamp returns before values`() {
        // Given
        val before = createAccelSample(1000L, 1f, 2f, 3f)
        val after = createAccelSample(2000L, 4f, 5f, 6f)

        // When: Interpolate at exact before timestamp
        val result = synchronizer.interpolateIMU(1000L, before, after)

        // Then: Returns before values
        assertEquals(1f, result.x, 0.001f)
        assertEquals(2f, result.y, 0.001f)
        assertEquals(3f, result.z, 0.001f)
    }

    @Test
    fun `interpolateIMU at after timestamp returns after values`() {
        // Given
        val before = createAccelSample(1000L, 1f, 2f, 3f)
        val after = createAccelSample(2000L, 4f, 5f, 6f)

        // When: Interpolate at exact after timestamp
        val result = synchronizer.interpolateIMU(2000L, before, after)

        // Then: Returns after values
        assertEquals(4f, result.x, 0.001f)
        assertEquals(5f, result.y, 0.001f)
        assertEquals(6f, result.z, 0.001f)
    }

    @Test
    fun `interpolateIMU at 25 percent returns correct values`() {
        // Given
        val before = createAccelSample(1000L, 0f, 0f, 0f)
        val after = createAccelSample(2000L, 100f, 100f, 100f)

        // When: Interpolate at 25%
        val result = synchronizer.interpolateIMU(1250L, before, after)

        // Then
        assertEquals(25f, result.x, 0.001f)
        assertEquals(25f, result.y, 0.001f)
        assertEquals(25f, result.z, 0.001f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `interpolateIMU throws exception for different sensor types`() {
        // Given: Samples of different types
        val accel = createAccelSample(1000L)
        val gyro = createGyroSample(2000L)

        // When: Try to interpolate
        synchronizer.interpolateIMU(1500L, accel, gyro)

        // Then: Exception thrown
    }

    @Test(expected = IllegalArgumentException::class)
    fun `interpolateIMU throws exception when target before before-sample`() {
        // Given
        val before = createAccelSample(2000L)
        val after = createAccelSample(3000L)

        // When: Target is before both samples
        synchronizer.interpolateIMU(1000L, before, after)

        // Then: Exception thrown
    }

    @Test(expected = IllegalArgumentException::class)
    fun `interpolateIMU throws exception when target after after-sample`() {
        // Given
        val before = createAccelSample(1000L)
        val after = createAccelSample(2000L)

        // When: Target is after both samples
        synchronizer.interpolateIMU(3000L, before, after)

        // Then: Exception thrown
    }

    // ===========================================
    // getIMUSamplesBetweenFrames Tests
    // ===========================================

    @Test
    fun `getIMUSamplesBetweenFrames returns samples between frames`() {
        // Given: IMU samples at 200Hz from 0 to 100ms
        populateBuffer(0L, 100_000_000L, 200)

        // When: Get samples between 20ms and 40ms
        val samples = synchronizer.getIMUSamplesBetweenFrames(20_000_000L, 40_000_000L)

        // Then: All samples are in range (exclusive prev, inclusive curr)
        assertTrue(samples.isNotEmpty())
        assertTrue(samples.all { it.timestampNs > 20_000_000L })
        assertTrue(samples.all { it.timestampNs <= 40_000_000L })
    }

    @Test(expected = IllegalArgumentException::class)
    fun `getIMUSamplesBetweenFrames throws exception for invalid range`() {
        // When: Current timestamp <= previous timestamp
        synchronizer.getIMUSamplesBetweenFrames(40_000_000L, 20_000_000L)

        // Then: Exception thrown
    }

    @Test
    fun `getIMUSamplesBetweenFrames returns empty list when no samples`() {
        // Given: Empty buffer

        // When
        val samples = synchronizer.getIMUSamplesBetweenFrames(0L, 100_000_000L)

        // Then
        assertTrue(samples.isEmpty())
    }

    // ===========================================
    // getSyncStatus Tests
    // ===========================================

    @Test
    fun `getSyncStatus returns healthy status with good data`() {
        // Given: Sufficient IMU samples at good rate
        populateBuffer(0L, 500_000_000L, 200)

        // When
        val status = synchronizer.getSyncStatus()

        // Then
        assertTrue(status.isHealthy)
        assertNull(status.warningMessage)
        assertTrue(status.averageImuRate > 100.0)
    }

    @Test
    fun `getSyncStatus detects low sample count`() {
        // Given: Very few samples
        buffer.add(createAccelSample(1000L))
        buffer.add(createGyroSample(1000L))

        // When
        val status = synchronizer.getSyncStatus()

        // Then
        assertFalse(status.isHealthy)
        assertNotNull(status.warningMessage)
        assertTrue(status.warningMessage!!.contains("Low IMU sample count"))
    }

    @Test
    fun `getSyncStatus returns sample count`() {
        // Given
        for (i in 0 until 50) {
            buffer.add(createAccelSample(i * 5_000_000L))
            buffer.add(createGyroSample(i * 5_000_000L))
        }

        // When
        val status = synchronizer.getSyncStatus()

        // Then
        assertEquals(100, status.imuSampleCount)
    }

    // ===========================================
    // reset Tests
    // ===========================================

    @Test
    fun `reset clears internal state`() {
        // Given: Some processing has occurred
        populateBuffer(0L, 100_000_000L)
        synchronizer.associateIMUWithFrame(50_000_000L, 1)

        // When
        synchronizer.reset()

        // Then: Status reflects reset state (lastFrameTimestamp = 0)
        val status = synchronizer.getSyncStatus()
        assertEquals(0L, status.lastFrameTimestampNs)
    }

    // ===========================================
    // Performance Tests
    // ===========================================

    @Test
    fun `associateIMUWithFrame completes within 1ms`() {
        // Given: Typical buffer with 5 seconds of data
        populateBuffer(0L, 5_000_000_000L, 200)

        // When: Measure multiple associations
        val iterations = 100
        val startTime = System.nanoTime()

        for (i in 0 until iterations) {
            val frameTs = (i * 33_333_333L) // ~30fps
            synchronizer.associateIMUWithFrame(frameTs, i.toLong())
        }

        val totalTimeMs = (System.nanoTime() - startTime) / 1_000_000.0
        val avgTimeMs = totalTimeMs / iterations

        // Then: Average time is under 1ms
        assertTrue(
            "Average association time (${avgTimeMs}ms) exceeds 1ms target",
            avgTimeMs < 1.0
        )
    }

    @Test
    fun `interpolation is computationally efficient`() {
        // Given
        val before = createAccelSample(1000L, 0f, 0f, 0f)
        val after = createAccelSample(2000L, 10f, 10f, 10f)

        // When: Measure many interpolations
        val iterations = 10000
        val startTime = System.nanoTime()

        for (i in 0 until iterations) {
            val target = 1000L + (i % 1000)
            synchronizer.interpolateIMU(target, before, after)
        }

        val totalTimeMs = (System.nanoTime() - startTime) / 1_000_000.0
        val avgTimeMicros = (totalTimeMs * 1000) / iterations

        // Then: Very fast (< 10 microseconds per interpolation)
        assertTrue(
            "Average interpolation time (${avgTimeMicros}μs) too slow",
            avgTimeMicros < 10.0
        )
    }

    // ===========================================
    // Thread Safety Tests
    // ===========================================

    @Test
    fun `synchronizer is thread-safe with concurrent operations`() {
        // Given: Buffer with data
        populateBuffer(0L, 1_000_000_000L, 200)

        // When: Concurrent reads and associations
        val writeThread = Thread {
            for (i in 0 until 100) {
                buffer.add(createAccelSample(1_000_000_000L + i * 5_000_000L))
                buffer.add(createGyroSample(1_000_000_000L + i * 5_000_000L))
                Thread.sleep(1)
            }
        }

        val readThread = Thread {
            for (i in 0 until 100) {
                synchronizer.associateIMUWithFrame(i * 10_000_000L, i.toLong())
                synchronizer.getSyncStatus()
                Thread.sleep(1)
            }
        }

        writeThread.start()
        readThread.start()
        writeThread.join()
        readThread.join()

        // Then: No exceptions, status is retrievable
        val status = synchronizer.getSyncStatus()
        assertNotNull(status)
    }

    // ===========================================
    // Edge Cases
    // ===========================================

    @Test
    fun `handles identical timestamps for before and after`() {
        // Given: Same timestamp samples (edge case)
        val sample = createAccelSample(1000L, 5f, 5f, 5f)

        // When: Interpolate with same sample as before and after
        val result = synchronizer.interpolateIMU(1000L, sample, sample)

        // Then: Returns the value (midpoint of same = same)
        assertEquals(5f, result.x, 0.001f)
    }

    @Test
    fun `handles gyroscope interpolation`() {
        // Given: Gyroscope samples
        val before = createGyroSample(1000L, 0.1f, 0.2f, 0.3f)
        val after = createGyroSample(2000L, 0.5f, 0.6f, 0.7f)

        // When
        val result = synchronizer.interpolateIMU(1500L, before, after)

        // Then
        assertEquals(SensorType.GYROSCOPE, result.type)
        assertEquals(0.3f, result.x, 0.001f)
        assertEquals(0.4f, result.y, 0.001f)
        assertEquals(0.5f, result.z, 0.001f)
    }
}
