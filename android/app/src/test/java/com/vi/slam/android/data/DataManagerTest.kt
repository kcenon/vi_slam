package com.vi.slam.android.data

import com.vi.slam.android.sensor.IMUCircularBuffer
import com.vi.slam.android.sensor.SynchronizedData
import com.vi.slam.android.sensor.TimestampSynchronizer
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.File

class DataManagerTest {

    @get:Rule
    val tempFolder = TemporaryFolder()

    private lateinit var dataManager: DataManager
    private lateinit var outputBaseDir: File
    private lateinit var timestampSynchronizer: TimestampSynchronizer

    @Before
    fun setUp() {
        outputBaseDir = tempFolder.newFolder("output")

        val imuBuffer = IMUCircularBuffer(capacitySamples = 1000)
        timestampSynchronizer = TimestampSynchronizer(imuBuffer)

        dataManager = DataManager(
            timestampSynchronizer = timestampSynchronizer,
            outputBaseDirectory = outputBaseDir
        )
    }

    @Test
    fun testInitialize_success() {
        val result = dataManager.initialize()

        assertTrue("Initialize should succeed", result.isSuccess)
        assertEquals(
            "Initial status should be IDLE",
            SessionStatus.IDLE,
            dataManager.getSessionStatus()
        )
    }

    @Test
    fun testInitialize_createsOutputDirectory() {
        val newDir = File(tempFolder.root, "new_output")
        assertFalse("Directory should not exist initially", newDir.exists())

        val newDataManager = DataManager(timestampSynchronizer, newDir)
        val result = newDataManager.initialize()

        assertTrue("Initialize should succeed", result.isSuccess)
        assertTrue("Output directory should be created", newDir.exists())
    }

    @Test
    fun testStartSession_recordOnly() {
        dataManager.initialize()

        val result = dataManager.startSession(SessionMode.RECORD_ONLY)

        assertTrue("Start session should succeed", result.isSuccess)
        assertEquals(
            "Status should be ACTIVE",
            SessionStatus.ACTIVE,
            dataManager.getSessionStatus()
        )

        val sessionInfo = result.getOrNull()
        assertNotNull("Session info should not be null", sessionInfo)
        assertEquals("Mode should be RECORD_ONLY", SessionMode.RECORD_ONLY, sessionInfo!!.mode)
        assertNotNull("Output directory should be set for RECORD_ONLY", sessionInfo.outputDirectory)
    }

    @Test
    fun testStartSession_streamOnly() {
        dataManager.initialize()

        val result = dataManager.startSession(SessionMode.STREAM_ONLY)

        assertTrue("Start session should succeed", result.isSuccess)

        val sessionInfo = result.getOrNull()
        assertNotNull("Session info should not be null", sessionInfo)
        assertEquals("Mode should be STREAM_ONLY", SessionMode.STREAM_ONLY, sessionInfo!!.mode)
        assertNull("Output directory should be null for STREAM_ONLY", sessionInfo.outputDirectory)
    }

    @Test
    fun testStartSession_recordAndStream() {
        dataManager.initialize()

        val result = dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        assertTrue("Start session should succeed", result.isSuccess)

        val sessionInfo = result.getOrNull()
        assertNotNull("Session info should not be null", sessionInfo)
        assertEquals(
            "Mode should be RECORD_AND_STREAM",
            SessionMode.RECORD_AND_STREAM,
            sessionInfo!!.mode
        )
        assertNotNull("Output directory should be set for RECORD_AND_STREAM", sessionInfo.outputDirectory)
    }

    @Test
    fun testStartSession_failsWhenAlreadyActive() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        val result = dataManager.startSession(SessionMode.STREAM_ONLY)

        assertTrue("Second start should fail", result.isFailure)
        assertNotNull("Should have exception", result.exceptionOrNull())
        assertTrue(
            "Exception should be IllegalStateException",
            result.exceptionOrNull() is IllegalStateException
        )
    }

    @Test
    fun testStartSession_createsSessionDirectory() {
        dataManager.initialize()

        val result = dataManager.startSession(SessionMode.RECORD_ONLY)

        assertTrue("Start session should succeed", result.isSuccess)

        val sessionInfo = result.getOrNull()!!
        val sessionDir = File(sessionInfo.outputDirectory!!)

        assertTrue("Session directory should exist", sessionDir.exists())
        assertTrue("Session directory should be a directory", sessionDir.isDirectory)
    }

    @Test
    fun testStopSession_success() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        val result = dataManager.stopSession()

        assertTrue("Stop session should succeed", result.isSuccess)
        assertEquals(
            "Status should be IDLE after stop",
            SessionStatus.IDLE,
            dataManager.getSessionStatus()
        )

        val summary = result.getOrNull()
        assertNotNull("Session summary should not be null", summary)
        assertTrue("Session should be successful", summary!!.success)
        assertNull("Error message should be null on success", summary.errorMessage)
    }

    @Test
    fun testStopSession_failsWhenNotActive() {
        dataManager.initialize()

        val result = dataManager.stopSession()

        assertTrue("Stop should fail when not active", result.isFailure)
        assertTrue(
            "Exception should be IllegalStateException",
            result.exceptionOrNull() is IllegalStateException
        )
    }

    @Test
    fun testStopSession_returnsSummaryWithStatistics() {
        dataManager.initialize()
        val startResult = dataManager.startSession(SessionMode.RECORD_ONLY)
        val sessionInfo = startResult.getOrNull()!!

        val stopResult = dataManager.stopSession()

        assertTrue("Stop should succeed", stopResult.isSuccess)

        val summary = stopResult.getOrNull()!!
        assertEquals("Session ID should match", sessionInfo.sessionId, summary.sessionId)
        assertEquals("Mode should match", sessionInfo.mode, summary.mode)
        assertEquals(
            "Output directory should match",
            sessionInfo.outputDirectory,
            summary.outputDirectory
        )
        assertNotNull("Statistics should not be null", summary.statistics)
    }

    @Test
    fun testSessionLifecycle_fullCycle() {
        dataManager.initialize()

        assertEquals("Initial state should be IDLE", SessionStatus.IDLE, dataManager.getSessionStatus())

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)
        assertEquals("State should be ACTIVE after start", SessionStatus.ACTIVE, dataManager.getSessionStatus())

        dataManager.stopSession()
        assertEquals("State should be IDLE after stop", SessionStatus.IDLE, dataManager.getSessionStatus())
    }

    @Test
    fun testGetStatistics_initialState() {
        dataManager.initialize()

        val stats = dataManager.getStatistics()

        assertEquals("Initial frame count should be 0", 0, stats.frameCount)
        assertEquals("Initial IMU count should be 0", 0, stats.imuSampleCount)
        assertEquals("Initial duration should be 0", 0, stats.durationMs)
        assertEquals("Initial drop count should be 0", 0, stats.frameDropCount)
        assertEquals("Initial FPS should be 0", 0f, stats.averageFps, 0.001f)
        assertEquals("Initial IMU rate should be 0", 0f, stats.averageImuRate, 0.001f)
    }

    @Test
    fun testSetRecorderEnabled() {
        dataManager.initialize()

        dataManager.setRecorderEnabled(false)
        dataManager.setRecorderEnabled(true)
    }

    @Test
    fun testSetStreamerEnabled() {
        dataManager.initialize()

        dataManager.setStreamerEnabled(false)
        dataManager.setStreamerEnabled(true)
    }

    @Test
    fun testRegisterDestination() {
        dataManager.initialize()

        val mockDestination = MockDataDestination()
        dataManager.registerDestination(mockDestination)
    }

    @Test
    fun testUnregisterDestination() {
        dataManager.initialize()

        val mockDestination = MockDataDestination()
        dataManager.registerDestination(mockDestination)
        dataManager.unregisterDestination(mockDestination)
    }

    @Test
    fun testSessionIdFormat() {
        dataManager.initialize()

        val result = dataManager.startSession(SessionMode.RECORD_ONLY)
        val sessionInfo = result.getOrNull()!!

        assertTrue(
            "Session ID should match format: yyyyMMdd_HHmmss_uuid",
            sessionInfo.sessionId.matches(Regex("\\d{8}_\\d{6}_[a-f0-9]{8}"))
        )
    }

    @Test
    fun testMultipleSessionsSequentially() {
        dataManager.initialize()

        val result1 = dataManager.startSession(SessionMode.RECORD_ONLY)
        assertTrue("First session should start", result1.isSuccess)
        val session1Id = result1.getOrNull()!!.sessionId

        dataManager.stopSession()

        val result2 = dataManager.startSession(SessionMode.STREAM_ONLY)
        assertTrue("Second session should start", result2.isSuccess)
        val session2Id = result2.getOrNull()!!.sessionId

        assertNotEquals("Session IDs should be different", session1Id, session2Id)

        dataManager.stopSession()
    }

    @Test
    fun testOnFrameAvailable_routesToDestinations() {
        dataManager.initialize()

        val destination1 = MockDataDestination()
        val destination2 = MockDataDestination()
        dataManager.registerDestination(destination1)
        dataManager.registerDestination(destination2)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        // Simulate frame arrival
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)

        // Verify data was routed to both destinations
        assertEquals(
            "Destination 1 should receive 1 frame",
            1,
            destination1.receivedData.size
        )
        assertEquals(
            "Destination 2 should receive 1 frame",
            1,
            destination2.receivedData.size
        )

        dataManager.stopSession()
    }

    @Test
    fun testOnFrameAvailable_doesNotRouteWhenSessionNotActive() {
        dataManager.initialize()

        val destination = MockDataDestination()
        dataManager.registerDestination(destination)

        // Call onFrameAvailable without starting session
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)

        // Verify no data was routed
        assertEquals(
            "Destination should not receive data when session is not active",
            0,
            destination.receivedData.size
        )
    }

    @Test
    fun testOnFrameAvailable_skipsDisabledDestinations() {
        dataManager.initialize()

        val enabledDestination = MockDataDestination()
        val disabledDestination = MockDataDestination()
        disabledDestination.setEnabled(false)

        dataManager.registerDestination(enabledDestination)
        dataManager.registerDestination(disabledDestination)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)

        assertEquals(
            "Enabled destination should receive data",
            1,
            enabledDestination.receivedData.size
        )
        assertEquals(
            "Disabled destination should not receive data",
            0,
            disabledDestination.receivedData.size
        )

        dataManager.stopSession()
    }

    @Test
    fun testOnFrameAvailable_updatesStatistics() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        val initialStats = dataManager.getStatistics()
        assertEquals("Initial frame count should be 0", 0, initialStats.frameCount)

        // Simulate multiple frames
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)
        dataManager.onFrameAvailable(frameTimestampNs = 1050000000L, frameSequence = 2)
        dataManager.onFrameAvailable(frameTimestampNs = 1100000000L, frameSequence = 3)

        val finalStats = dataManager.getStatistics()
        assertEquals("Frame count should be 3", 3, finalStats.frameCount)
        assertTrue("Duration should be positive", finalStats.durationMs > 0)
        assertTrue("Average FPS should be positive", finalStats.averageFps > 0)

        dataManager.stopSession()
    }

    @Test
    fun testOnFrameAvailable_handlesExceptionInDestination() {
        dataManager.initialize()

        val goodDestination = MockDataDestination()
        val badDestination = object : IDataDestination {
            override fun onData(data: SynchronizedData) {
                throw RuntimeException("Test exception")
            }

            override fun isEnabled(): Boolean = true
        }

        dataManager.registerDestination(badDestination)
        dataManager.registerDestination(goodDestination)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        // Should not throw despite badDestination throwing
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)

        // goodDestination should still receive data
        assertEquals(
            "Good destination should receive data despite bad destination throwing",
            1,
            goodDestination.receivedData.size
        )

        dataManager.stopSession()
    }

    @Test
    fun testMultipleDestinations_receiveIdenticalData() {
        dataManager.initialize()

        val destination1 = MockDataDestination()
        val destination2 = MockDataDestination()
        dataManager.registerDestination(destination1)
        dataManager.registerDestination(destination2)

        dataManager.startSession(SessionMode.RECORD_AND_STREAM)

        dataManager.onFrameAvailable(frameTimestampNs = 1234567890L, frameSequence = 42)

        assertEquals(
            "Both destinations should receive data",
            1,
            destination1.receivedData.size
        )
        assertEquals(
            "Both destinations should receive data",
            1,
            destination2.receivedData.size
        )

        val data1 = destination1.receivedData[0]
        val data2 = destination2.receivedData[0]

        assertEquals(
            "Frame timestamps should match",
            data1.frameTimestampNs,
            data2.frameTimestampNs
        )
        assertEquals(
            "Frame sequences should match",
            data1.frameSequence,
            data2.frameSequence
        )

        dataManager.stopSession()
    }

    @Test
    fun testStatistics_frameDropDetection() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        // Send frames with discontinuous sequence numbers
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)
        dataManager.onFrameAvailable(frameTimestampNs = 1050000000L, frameSequence = 2)
        // Skip sequence 3, 4, 5 - simulating 3 dropped frames
        dataManager.onFrameAvailable(frameTimestampNs = 1100000000L, frameSequence = 6)
        dataManager.onFrameAvailable(frameTimestampNs = 1150000000L, frameSequence = 7)

        val stats = dataManager.getStatistics()

        assertEquals("Frame count should be 4", 4, stats.frameCount)
        assertEquals("Frame drop count should be 3", 3, stats.frameDropCount)

        dataManager.stopSession()
    }

    @Test
    fun testStatistics_noDropsWithContinuousSequence() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        // Send frames with continuous sequence numbers
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)
        dataManager.onFrameAvailable(frameTimestampNs = 1050000000L, frameSequence = 2)
        dataManager.onFrameAvailable(frameTimestampNs = 1100000000L, frameSequence = 3)
        dataManager.onFrameAvailable(frameTimestampNs = 1150000000L, frameSequence = 4)

        val stats = dataManager.getStatistics()

        assertEquals("Frame count should be 4", 4, stats.frameCount)
        assertEquals("Frame drop count should be 0", 0, stats.frameDropCount)

        dataManager.stopSession()
    }

    @Test
    fun testStatistics_averageFpsCalculation() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        // Wait a bit to accumulate some duration
        Thread.sleep(100)

        // Simulate multiple frames
        for (i in 1..10) {
            dataManager.onFrameAvailable(
                frameTimestampNs = 1000000000L + i * 33_000_000L,
                frameSequence = i.toLong()
            )
        }

        val stats = dataManager.getStatistics()

        assertEquals("Frame count should be 10", 10, stats.frameCount)
        assertTrue("Average FPS should be positive", stats.averageFps > 0f)
        assertTrue("Duration should be > 100ms", stats.durationMs >= 100)

        dataManager.stopSession()
    }

    @Test
    fun testStatistics_resetBetweenSessions() {
        dataManager.initialize()

        // First session
        dataManager.startSession(SessionMode.RECORD_ONLY)
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)
        dataManager.onFrameAvailable(frameTimestampNs = 1050000000L, frameSequence = 2)
        val stats1 = dataManager.getStatistics()
        dataManager.stopSession()

        // Second session - statistics should reset
        dataManager.startSession(SessionMode.STREAM_ONLY)
        val stats2 = dataManager.getStatistics()

        assertEquals("First session should have 2 frames", 2, stats1.frameCount)
        assertEquals("Second session should start with 0 frames", 0, stats2.frameCount)
        assertEquals("Second session should start with 0 drops", 0, stats2.frameDropCount)

        dataManager.stopSession()
    }

    @Test
    fun testStatistics_sessionSummaryContainsFinalStats() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        // Process some frames
        dataManager.onFrameAvailable(frameTimestampNs = 1000000000L, frameSequence = 1)
        dataManager.onFrameAvailable(frameTimestampNs = 1050000000L, frameSequence = 2)
        // Simulate frame drop
        dataManager.onFrameAvailable(frameTimestampNs = 1150000000L, frameSequence = 5)

        val summary = dataManager.stopSession().getOrNull()!!

        assertNotNull("Summary statistics should not be null", summary.statistics)
        assertEquals("Summary should have correct frame count", 3, summary.statistics.frameCount)
        assertEquals("Summary should have correct drop count", 2, summary.statistics.frameDropCount)
        assertTrue("Summary should have positive duration", summary.statistics.durationMs > 0)
    }

    @Test
    fun testStatistics_performanceOverhead() {
        dataManager.initialize()
        dataManager.startSession(SessionMode.RECORD_ONLY)

        val frameCount = 1000
        val startTime = System.nanoTime()

        // Process many frames
        for (i in 1..frameCount) {
            dataManager.onFrameAvailable(
                frameTimestampNs = 1000000000L + i * 33_000_000L,
                frameSequence = i.toLong()
            )
        }

        val endTime = System.nanoTime()
        val totalTimeMs = (endTime - startTime) / 1_000_000.0

        // Each frame should process in < 1ms on average (statistics overhead < 1%)
        val avgTimePerFrame = totalTimeMs / frameCount
        assertTrue(
            "Average time per frame ($avgTimePerFrame ms) should be < 1ms",
            avgTimePerFrame < 1.0
        )

        val stats = dataManager.getStatistics()
        assertEquals("All frames should be counted", frameCount.toLong(), stats.frameCount)

        dataManager.stopSession()
    }

    /**
     * Mock data destination for testing.
     */
    private class MockDataDestination : IDataDestination {
        private var enabled = true
        val receivedData = mutableListOf<SynchronizedData>()

        override fun onData(data: SynchronizedData) {
            if (enabled) {
                receivedData.add(data)
            }
        }

        override fun isEnabled(): Boolean = enabled

        fun setEnabled(value: Boolean) {
            enabled = value
        }
    }
}
