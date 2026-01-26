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
