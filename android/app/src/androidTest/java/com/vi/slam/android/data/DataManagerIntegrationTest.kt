package com.vi.slam.android.data

import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.vi.slam.android.sensor.IMUCircularBuffer
import com.vi.slam.android.sensor.IMUSample
import com.vi.slam.android.sensor.SynchronizedData
import com.vi.slam.android.sensor.TimestampSynchronizer
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

/**
 * Integration tests for DataManager.
 *
 * These tests verify end-to-end functionality with real components
 * (TimestampSynchronizer, destinations) and realistic data flows.
 */
@RunWith(AndroidJUnit4::class)
class DataManagerIntegrationTest {

    private lateinit var dataManager: DataManager
    private lateinit var outputBaseDir: File
    private lateinit var timestampSynchronizer: TimestampSynchronizer
    private lateinit var imuBuffer: IMUCircularBuffer

    @Before
    fun setUp() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        outputBaseDir = File(context.cacheDir, "integration_test_output")
        if (outputBaseDir.exists()) {
            outputBaseDir.deleteRecursively()
        }
        outputBaseDir.mkdirs()

        imuBuffer = IMUCircularBuffer(capacitySamples = 1000)
        timestampSynchronizer = TimestampSynchronizer(imuBuffer)

        dataManager = DataManager(
            timestampSynchronizer = timestampSynchronizer,
            outputBaseDirectory = outputBaseDir
        )
    }

    @After
    fun tearDown() {
        if (dataManager.getSessionStatus() == SessionStatus.ACTIVE) {
            dataManager.stopSession()
        }
        if (outputBaseDir.exists()) {
            outputBaseDir.deleteRecursively()
        }
    }

    @Test
    fun testSessionLifecycle_recordOnly() {
        // Initialize
        val initResult = dataManager.initialize()
        assertTrue("Initialize should succeed", initResult.isSuccess)
        assertEquals("Initial status should be IDLE", SessionStatus.IDLE, dataManager.getSessionStatus())

        // Start session
        val startResult = dataManager.startSession(SessionMode.RECORD_ONLY)
        assertTrue("Start session should succeed", startResult.isSuccess)
        assertEquals("Status should be ACTIVE", SessionStatus.ACTIVE, dataManager.getSessionStatus())

        val sessionInfo = startResult.getOrThrow()
        assertNotNull("Session ID should not be null", sessionInfo.sessionId)
        assertEquals("Mode should be RECORD_ONLY", SessionMode.RECORD_ONLY, sessionInfo.mode)
        assertNotNull("Output directory should be set", sessionInfo.outputDirectory)

        // Verify session directory was created
        val sessionDir = File(sessionInfo.outputDirectory!!)
        assertTrue("Session directory should exist", sessionDir.exists())

        // Stop session
        val stopResult = dataManager.stopSession()
        assertTrue("Stop session should succeed", stopResult.isSuccess)
        assertEquals("Status should be IDLE", SessionStatus.IDLE, dataManager.getSessionStatus())

        val summary = stopResult.getOrThrow()
        assertEquals("Session ID should match", sessionInfo.sessionId, summary.sessionId)
        assertTrue("Session should be successful", summary.success)
    }

    @Test
    fun testSessionLifecycle_streamOnly() {
        dataManager.initialize()

        val startResult = dataManager.startSession(SessionMode.STREAM_ONLY)
        assertTrue("Start session should succeed", startResult.isSuccess)

        val sessionInfo = startResult.getOrThrow()
        assertEquals("Mode should be STREAM_ONLY", SessionMode.STREAM_ONLY, sessionInfo.mode)
        assertNull("Output directory should be null", sessionInfo.outputDirectory)

        val stopResult = dataManager.stopSession()
        assertTrue("Stop session should succeed", stopResult.isSuccess)
        assertEquals("Status should be IDLE", SessionStatus.IDLE, dataManager.getSessionStatus())
    }

    @Test
    fun testSessionLifecycle_recordAndStream() {
        dataManager.initialize()

        val startResult = dataManager.startSession(SessionMode.RECORD_AND_STREAM)
        assertTrue("Start session should succeed", startResult.isSuccess)

        val sessionInfo = startResult.getOrThrow()
        assertEquals("Mode should be RECORD_AND_STREAM", SessionMode.RECORD_AND_STREAM, sessionInfo.mode)
        assertNotNull("Output directory should be set", sessionInfo.outputDirectory)

        val stopResult = dataManager.stopSession()
        assertTrue("Stop session should succeed", stopResult.isSuccess)
    }

    @Test
    fun testDataRouting_singleDestination() = runBlocking {
        dataManager.initialize()

        val destination = TestDataDestination()
        dataManager.registerDestination(destination)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        // Add IMU samples to buffer
        val baseTime = System.nanoTime()
        for (i in 0 until 10) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.1f,
                    accelY = 0.2f,
                    accelZ = 9.8f,
                    gyroX = 0.01f,
                    gyroY = 0.02f,
                    gyroZ = 0.03f
                )
            )
        }

        // Simulate frame arrival
        dataManager.onFrameAvailable(baseTime + 25_000_000L, 1)

        // Wait briefly for processing
        delay(100)

        assertEquals("Destination should receive 1 frame", 1, destination.receivedCount.get())

        dataManager.stopSession()
    }

    @Test
    fun testDataRouting_multipleDestinations() = runBlocking {
        dataManager.initialize()

        val destination1 = TestDataDestination()
        val destination2 = TestDataDestination()
        val destination3 = TestDataDestination()

        dataManager.registerDestination(destination1)
        dataManager.registerDestination(destination2)
        dataManager.registerDestination(destination3)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        // Add IMU samples
        val baseTime = System.nanoTime()
        for (i in 0 until 10) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        // Simulate multiple frames
        for (frame in 1..5) {
            dataManager.onFrameAvailable(baseTime + frame * 33_000_000L, frame.toLong())
        }

        delay(200)

        assertEquals("Destination 1 should receive all frames", 5, destination1.receivedCount.get())
        assertEquals("Destination 2 should receive all frames", 5, destination2.receivedCount.get())
        assertEquals("Destination 3 should receive all frames", 5, destination3.receivedCount.get())

        dataManager.stopSession()
    }

    @Test
    fun testStatisticsAccuracy_frameCount() = runBlocking {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        // Add IMU samples
        val baseTime = System.nanoTime()
        for (i in 0 until 50) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        // Process 10 frames
        val frameCount = 10
        for (frame in 1..frameCount) {
            dataManager.onFrameAvailable(baseTime + frame * 33_000_000L, frame.toLong())
        }

        delay(100)

        val stats = dataManager.getStatistics()
        assertEquals("Frame count should match", frameCount.toLong(), stats.frameCount)
        assertTrue("Duration should be positive", stats.durationMs > 0)

        dataManager.stopSession()
    }

    @Test
    fun testStatisticsAccuracy_fps() = runBlocking {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        val baseTime = System.nanoTime()

        // Add IMU samples
        for (i in 0 until 100) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        // Simulate 30 FPS for 1 second
        val frameInterval = 33_000_000L // ~30 FPS
        for (frame in 1..30) {
            dataManager.onFrameAvailable(baseTime + frame * frameInterval, frame.toLong())
        }

        delay(1100) // Wait at least 1 second

        val stats = dataManager.getStatistics()
        assertTrue("Average FPS should be > 20", stats.averageFps > 20f)
        assertTrue("Average FPS should be < 40", stats.averageFps < 40f)

        dataManager.stopSession()
    }

    @Test
    fun testStatisticsAccuracy_imuRate() = runBlocking {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        val baseTime = System.nanoTime()

        // Add 200 Hz IMU samples for 1 second
        val imuInterval = 5_000_000L // 200 Hz
        for (i in 0 until 200) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * imuInterval,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        // Simulate frames to trigger IMU association
        for (frame in 1..10) {
            dataManager.onFrameAvailable(baseTime + frame * 100_000_000L, frame.toLong())
        }

        delay(1100)

        val stats = dataManager.getStatistics()
        assertTrue("IMU sample count should be > 0", stats.imuSampleCount > 0)
        assertTrue("Average IMU rate should be positive", stats.averageImuRate > 0f)

        dataManager.stopSession()
    }

    @Test
    fun testDynamicEnableDisable_destination() = runBlocking {
        dataManager.initialize()

        val destination = TestDataDestination()
        dataManager.registerDestination(destination)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        val baseTime = System.nanoTime()

        // Add IMU samples
        for (i in 0 until 20) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        // Send 3 frames with destination enabled
        for (frame in 1..3) {
            dataManager.onFrameAvailable(baseTime + frame * 33_000_000L, frame.toLong())
        }

        delay(100)
        val countBeforeDisable = destination.receivedCount.get()
        assertEquals("Should receive 3 frames", 3, countBeforeDisable)

        // Disable destination
        destination.setEnabled(false)

        // Send 2 more frames (should not be received)
        for (frame in 4..5) {
            dataManager.onFrameAvailable(baseTime + frame * 33_000_000L, frame.toLong())
        }

        delay(100)
        assertEquals("Should still have 3 frames (no new frames)", 3, destination.receivedCount.get())

        // Re-enable destination
        destination.setEnabled(true)

        // Send 2 more frames (should be received)
        for (frame in 6..7) {
            dataManager.onFrameAvailable(baseTime + frame * 33_000_000L, frame.toLong())
        }

        delay(100)
        assertEquals("Should have 5 frames total", 5, destination.receivedCount.get())

        dataManager.stopSession()
    }

    @Test
    fun testErrorHandling_invalidStateTransitions() {
        dataManager.initialize()

        // Try to stop without starting
        val stopResult = dataManager.stopSession()
        assertTrue("Stop should fail when not active", stopResult.isFailure)
        assertTrue("Should be IllegalStateException", stopResult.exceptionOrNull() is IllegalStateException)

        // Start session
        dataManager.startSession(SessionMode.RECORD_ONLY)

        // Try to start again
        val startAgainResult = dataManager.startSession(SessionMode.STREAM_ONLY)
        assertTrue("Start should fail when already active", startAgainResult.isFailure)
        assertTrue("Should be IllegalStateException", startAgainResult.exceptionOrNull() is IllegalStateException)

        dataManager.stopSession()
    }

    @Test
    fun testErrorHandling_destinationException() = runBlocking {
        dataManager.initialize()

        val goodDestination = TestDataDestination()
        val failingDestination = object : IDataDestination {
            override fun onData(data: SynchronizedData) {
                throw RuntimeException("Simulated destination failure")
            }

            override fun isEnabled(): Boolean = true
        }

        dataManager.registerDestination(failingDestination)
        dataManager.registerDestination(goodDestination)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        val baseTime = System.nanoTime()
        for (i in 0 until 10) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        // Should not throw despite failing destination
        dataManager.onFrameAvailable(baseTime + 25_000_000L, 1)

        delay(100)

        // Good destination should still receive data
        assertEquals("Good destination should receive frame despite other failure", 1, goodDestination.receivedCount.get())

        dataManager.stopSession()
    }

    @Test
    fun testPerformance_1000Frames() = runBlocking {
        dataManager.initialize()

        val destination = TestDataDestination()
        dataManager.registerDestination(destination)

        dataManager.startSession(SessionMode.RECORD_ONLY)

        val baseTime = System.nanoTime()

        // Add sufficient IMU samples
        for (i in 0 until 5000) {
            imuBuffer.add(
                IMUSample(
                    timestampNs = baseTime + i * 5_000_000L,
                    accelX = 0.0f,
                    accelY = 0.0f,
                    accelZ = 9.8f,
                    gyroX = 0.0f,
                    gyroY = 0.0f,
                    gyroZ = 0.0f
                )
            )
        }

        val startProcessing = System.currentTimeMillis()

        // Process 1000 frames
        for (frame in 1..1000) {
            dataManager.onFrameAvailable(baseTime + frame * 33_000_000L, frame.toLong())
        }

        val endProcessing = System.currentTimeMillis()
        val processingTime = endProcessing - startProcessing

        delay(500) // Allow processing to complete

        val stats = dataManager.getStatistics()
        assertEquals("Should process all 1000 frames", 1000L, stats.frameCount)
        assertTrue("Processing should complete in reasonable time", processingTime < 5000)

        val stopResult = dataManager.stopSession()
        assertTrue("Stop should succeed", stopResult.isSuccess)

        val summary = stopResult.getOrThrow()
        assertEquals("Summary should show 1000 frames", 1000L, summary.statistics.frameCount)
        assertTrue("Summary should indicate success", summary.success)
    }

    @Test
    fun testMultipleSessionsSequentially() {
        dataManager.initialize()

        // Session 1: RECORD_ONLY
        val result1 = dataManager.startSession(SessionMode.RECORD_ONLY)
        assertTrue("First session should start", result1.isSuccess)
        val session1 = result1.getOrThrow()

        val stop1 = dataManager.stopSession()
        assertTrue("First session should stop", stop1.isSuccess)

        // Session 2: STREAM_ONLY
        val result2 = dataManager.startSession(SessionMode.STREAM_ONLY)
        assertTrue("Second session should start", result2.isSuccess)
        val session2 = result2.getOrThrow()

        assertNotEquals("Session IDs should be different", session1.sessionId, session2.sessionId)

        val stop2 = dataManager.stopSession()
        assertTrue("Second session should stop", stop2.isSuccess)

        // Session 3: RECORD_AND_STREAM
        val result3 = dataManager.startSession(SessionMode.RECORD_AND_STREAM)
        assertTrue("Third session should start", result3.isSuccess)

        dataManager.stopSession()
    }

    @Test
    fun testRealisticDataRates_30FpsCamera_200HzIMU() = runBlocking {
        dataManager.initialize()

        val destination = TestDataDestination()
        dataManager.registerDestination(destination)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        val baseTime = System.nanoTime()
        val testDurationMs = 1000L // 1 second test

        // Start IMU generation
        val imuThread = Thread {
            val imuRate = 200 // 200 Hz
            val imuInterval = 1000L / imuRate // 5 ms
            val startTime = System.currentTimeMillis()

            while (System.currentTimeMillis() - startTime < testDurationMs) {
                val elapsed = System.currentTimeMillis() - startTime
                imuBuffer.add(
                    IMUSample(
                        timestampNs = baseTime + elapsed * 1_000_000L,
                        accelX = 0.0f,
                        accelY = 0.0f,
                        accelZ = 9.8f,
                        gyroX = 0.0f,
                        gyroY = 0.0f,
                        gyroZ = 0.0f
                    )
                )
                Thread.sleep(imuInterval)
            }
        }
        imuThread.start()

        // Generate camera frames at 30 FPS
        val frameRate = 30
        val frameInterval = 1000L / frameRate // ~33 ms
        val startTime = System.currentTimeMillis()
        var frameCount = 0

        while (System.currentTimeMillis() - startTime < testDurationMs) {
            val elapsed = System.currentTimeMillis() - startTime
            dataManager.onFrameAvailable(baseTime + elapsed * 1_000_000L, frameCount.toLong())
            frameCount++
            Thread.sleep(frameInterval)
        }

        imuThread.join()
        delay(200)

        val stats = dataManager.getStatistics()
        assertTrue("Frame count should be around 30", stats.frameCount in 25..35)
        assertTrue("IMU sample count should be around 200", stats.imuSampleCount in 150..250)
        assertTrue("Average FPS should be around 30", stats.averageFps in 20f..40f)

        dataManager.stopSession()
    }

    /**
     * Test data destination implementation.
     */
    private class TestDataDestination : IDataDestination {
        private val enabled = AtomicBoolean(true)
        val receivedCount = AtomicInteger(0)
        val receivedData = mutableListOf<SynchronizedData>()

        override fun onData(data: SynchronizedData) {
            receivedCount.incrementAndGet()
            synchronized(receivedData) {
                receivedData.add(data)
            }
        }

        override fun isEnabled(): Boolean = enabled.get()

        fun setEnabled(value: Boolean) {
            enabled.set(value)
        }
    }
}
