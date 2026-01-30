package com.vi.slam.android.recorder

import android.content.Context
import android.content.SharedPreferences
import com.vi.slam.android.sensor.IMUSample
import com.vi.slam.android.sensor.SensorType
import com.vi.slam.android.sensor.SynchronizedData
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.Mockito.`when`
import org.mockito.junit.MockitoJUnitRunner
import java.io.File

@RunWith(MockitoJUnitRunner::class)
class LocalRecorderTest {

    @get:Rule
    val tempFolder = TemporaryFolder()

    @Mock
    private lateinit var mockContext: Context

    @Mock
    private lateinit var mockSharedPreferences: SharedPreferences

    @Mock
    private lateinit var mockEditor: SharedPreferences.Editor

    private lateinit var recorder: LocalRecorder
    private lateinit var outputDir: File
    private lateinit var config: RecorderConfig

    @Before
    fun setUp() {
        // Setup SharedPreferences mocking
        `when`(mockContext.getSharedPreferences("vi_slam_recorder_state", Context.MODE_PRIVATE))
            .thenReturn(mockSharedPreferences)
        `when`(mockSharedPreferences.edit()).thenReturn(mockEditor)
        `when`(mockEditor.putString(org.mockito.ArgumentMatchers.anyString(), org.mockito.ArgumentMatchers.anyString()))
            .thenReturn(mockEditor)
        `when`(mockSharedPreferences.getString(org.mockito.ArgumentMatchers.anyString(), org.mockito.ArgumentMatchers.anyString()))
            .thenReturn(null)

        outputDir = tempFolder.newFolder("recordings")
        config = RecorderConfig(
            outputDirectory = outputDir,
            maxDurationMs = 60_000,  // 60 seconds
            videoWidth = 640,
            videoHeight = 480,
            videoFps = 30
        )
        recorder = LocalRecorder(mockContext)
    }

    // ========== Initialization Tests ==========

    @Test
    fun testInitialize_success() {
        val result = recorder.initialize(config)

        assertTrue("Initialize should succeed", result.isSuccess)
        assertFalse("Recorder should not be enabled after initialize", recorder.isEnabled())
    }

    @Test
    fun testInitialize_invalidDirectory_doesNotExist() {
        val nonExistentDir = File(tempFolder.root, "nonexistent")
        val invalidConfig = config.copy(outputDirectory = nonExistentDir)

        val result = recorder.initialize(invalidConfig)

        assertTrue("Initialize should fail for non-existent directory", result.isFailure)
    }

    @Test
    fun testInitialize_invalidDirectory_notWritable() {
        val readOnlyDir = tempFolder.newFolder("readonly")
        readOnlyDir.setWritable(false)

        val invalidConfig = config.copy(outputDirectory = readOnlyDir)
        val result = recorder.initialize(invalidConfig)

        // Clean up
        readOnlyDir.setWritable(true)

        assertTrue("Initialize should fail for non-writable directory", result.isFailure)
    }

    @Test
    fun testInitialize_multipleCallsAreIdempotent() {
        val result1 = recorder.initialize(config)
        val result2 = recorder.initialize(config)

        assertTrue("First initialize should succeed", result1.isSuccess)
        assertTrue("Second initialize should succeed", result2.isSuccess)
    }

    @Test
    fun testInitialize_cannotReinitializeWhileRecording() {
        recorder.initialize(config)
        recorder.startRecording()

        val result = recorder.initialize(config)

        assertTrue("Initialize should fail while recording", result.isFailure)

        // Cleanup
        recorder.stopRecording()
    }

    // ========== Start Recording Tests ==========

    @Test
    fun testStartRecording_success() {
        recorder.initialize(config)

        val result = recorder.startRecording()

        assertTrue("Start recording should succeed", result.isSuccess)
        assertTrue("Recorder should be enabled after start", recorder.isEnabled())

        val recordingInfo = result.getOrNull()
        assertNotNull("Recording info should not be null", recordingInfo)
        assertNotNull("Recording ID should not be null", recordingInfo?.recordingId)
        assertTrue("Start time should be positive", (recordingInfo?.startTime ?: 0) > 0)
        assertNotNull("Output path should not be null", recordingInfo?.outputPath)

        // Verify session directory was created
        val sessionDir = File(recordingInfo?.outputPath)
        assertTrue("Session directory should exist", sessionDir.exists())
        assertTrue("Session directory should be a directory", sessionDir.isDirectory)

        // Cleanup
        recorder.stopRecording()
    }

    @Test
    fun testStartRecording_failsIfNotInitialized() {
        val result = recorder.startRecording()

        assertTrue("Start recording should fail if not initialized", result.isFailure)
        assertFalse("Recorder should not be enabled", recorder.isEnabled())
    }

    @Test
    fun testStartRecording_failsIfAlreadyRecording() {
        recorder.initialize(config)
        recorder.startRecording()

        val result = recorder.startRecording()

        assertTrue("Second start recording should fail", result.isFailure)

        // Cleanup
        recorder.stopRecording()
    }

    @Test
    fun testStartRecording_createsUniqueSessionDirectories() {
        recorder.initialize(config)

        val result1 = recorder.startRecording()
        assertTrue("First start should succeed", result1.isSuccess)
        val recordingId1 = result1.getOrNull()?.recordingId
        val path1 = result1.getOrNull()?.outputPath
        recorder.stopRecording()

        // Small delay to ensure different timestamps
        Thread.sleep(10)

        val result2 = recorder.startRecording()
        assertTrue("Second start should succeed", result2.isSuccess)
        val recordingId2 = result2.getOrNull()?.recordingId
        val path2 = result2.getOrNull()?.outputPath
        recorder.stopRecording()

        assertNotEquals("Recording IDs should be unique", recordingId1, recordingId2)
        assertNotEquals("Output paths should be unique", path1, path2)
    }

    // ========== Stop Recording Tests ==========

    @Test
    fun testStopRecording_success() {
        recorder.initialize(config)
        recorder.startRecording()

        val result = recorder.stopRecording()

        assertTrue("Stop recording should succeed", result.isSuccess)
        assertFalse("Recorder should not be enabled after stop", recorder.isEnabled())

        val summary = result.getOrNull()
        assertNotNull("Recording summary should not be null", summary)
        assertTrue("Recording should be marked as successful", summary?.success == true)
        assertNotNull("Recording ID should not be null", summary?.recordingId)
        assertTrue("Duration should be positive", (summary?.durationMs ?: 0) >= 0)
        assertNull("Error message should be null for successful recording", summary?.errorMessage)
    }

    @Test
    fun testStopRecording_failsIfNotRecording() {
        recorder.initialize(config)

        val result = recorder.stopRecording()

        assertTrue("Stop recording should fail if not recording", result.isFailure)
    }

    @Test
    fun testStopRecording_returnsToIdleState() {
        recorder.initialize(config)
        recorder.startRecording()
        recorder.stopRecording()

        // Should be able to start a new recording after stopping
        val result = recorder.startRecording()
        assertTrue("Should be able to start new recording after stop", result.isSuccess)

        // Cleanup
        recorder.stopRecording()
    }

    // ========== Data Recording Tests ==========

    @Test
    fun testOnData_countsFramesWhenRecording() {
        recorder.initialize(config)
        recorder.startRecording()

        // Send some mock data
        val mockData = createMockSynchronizedData()
        repeat(10) {
            recorder.onData(mockData)
        }

        val result = recorder.stopRecording()
        val summary = result.getOrNull()

        assertNotNull("Summary should not be null", summary)
        assertEquals("Frame count should be 10", 10L, summary?.frameCount)
    }

    @Test
    fun testOnData_countsIMUSamplesWhenRecording() {
        recorder.initialize(config)
        recorder.startRecording()

        // Send mock data with 5 IMU samples each
        val mockData = createMockSynchronizedData(imuSampleCount = 5)
        repeat(10) {
            recorder.onData(mockData)
        }

        val result = recorder.stopRecording()
        val summary = result.getOrNull()

        assertNotNull("Summary should not be null", summary)
        // 10 frames * 5 IMU samples per frame = 50 total IMU samples
        assertEquals("IMU sample count should be 50", 50L, summary?.imuSampleCount)
    }

    @Test
    fun testOnData_doesNothingWhenNotRecording() {
        recorder.initialize(config)

        // Send data before starting recording
        val mockData = createMockSynchronizedData()
        recorder.onData(mockData)

        // Start and immediately stop to get summary
        recorder.startRecording()
        val result = recorder.stopRecording()
        val summary = result.getOrNull()

        // Frame count should be 0 since data was sent before recording started
        assertEquals("Frame count should be 0", 0L, summary?.frameCount)
    }

    // ========== State Management Tests ==========

    @Test
    fun testIsEnabled_returnsFalseWhenUninitialized() {
        assertFalse("Should not be enabled when uninitialized", recorder.isEnabled())
    }

    @Test
    fun testIsEnabled_returnsFalseWhenIdle() {
        recorder.initialize(config)
        assertFalse("Should not be enabled when idle", recorder.isEnabled())
    }

    @Test
    fun testIsEnabled_returnsTrueWhenRecording() {
        recorder.initialize(config)
        recorder.startRecording()

        assertTrue("Should be enabled when recording", recorder.isEnabled())

        // Cleanup
        recorder.stopRecording()
    }

    @Test
    fun testIsEnabled_returnsFalseAfterStopping() {
        recorder.initialize(config)
        recorder.startRecording()
        recorder.stopRecording()

        assertFalse("Should not be enabled after stopping", recorder.isEnabled())
    }

    // ========== Configuration Tests ==========

    @Test
    fun testRecorderConfig_validation() {
        // These should be validated by RecorderConfig's init block
        try {
            RecorderConfig(
                outputDirectory = outputDir,
                maxDurationMs = -1  // Invalid: negative duration
            )
            fail("Should throw exception for negative max duration")
        } catch (e: IllegalArgumentException) {
            assertTrue("Exception message should mention duration",
                e.message?.contains("duration", ignoreCase = true) == true)
        }

        try {
            RecorderConfig(
                outputDirectory = outputDir,
                videoWidth = 0  // Invalid: zero width
            )
            fail("Should throw exception for zero video width")
        } catch (e: IllegalArgumentException) {
            assertTrue("Exception message should mention dimensions",
                e.message?.contains("dimensions", ignoreCase = true) == true)
        }

        try {
            RecorderConfig(
                outputDirectory = outputDir,
                videoFps = 0  // Invalid: zero FPS
            )
            fail("Should throw exception for zero FPS")
        } catch (e: IllegalArgumentException) {
            assertTrue("Exception message should mention FPS",
                e.message?.contains("FPS", ignoreCase = true) == true)
        }

        try {
            RecorderConfig(
                outputDirectory = outputDir,
                videoBitrate = -1  // Invalid: negative bitrate
            )
            fail("Should throw exception for negative bitrate")
        } catch (e: IllegalArgumentException) {
            assertTrue("Exception message should mention bitrate",
                e.message?.contains("bitrate", ignoreCase = true) == true)
        }
    }

    // ========== IMU CSV Writer Tests ==========

    @Test
    fun testImuCsvWriter_fileCreatedOnStart() {
        recorder.initialize(config)

        val result = recorder.startRecording()
        assertTrue("Start recording should succeed", result.isSuccess)

        val recordingInfo = result.getOrNull()
        assertNotNull("Recording info should not be null", recordingInfo)

        val sessionDir = File(recordingInfo!!.outputPath)
        val csvFile = File(sessionDir, "imu_data.csv")

        assertTrue("IMU CSV file should be created", csvFile.exists())
        assertTrue("IMU CSV file should be readable", csvFile.canRead())

        // Cleanup
        recorder.stopRecording()
    }

    @Test
    fun testImuCsvWriter_headerWrittenCorrectly() {
        recorder.initialize(config)
        val result = recorder.startRecording()

        val recordingInfo = result.getOrNull()
        assertNotNull(recordingInfo)

        // Stop to flush and close the file
        recorder.stopRecording()

        val sessionDir = File(recordingInfo!!.outputPath)
        val csvFile = File(sessionDir, "imu_data.csv")

        val lines = csvFile.readLines()
        assertTrue("CSV file should have at least header", lines.isNotEmpty())

        val header = lines[0]
        assertEquals(
            "CSV header should match expected format",
            "timestamp_ns,sensor_type,x,y,z",
            header
        )
    }

    @Test
    fun testImuCsvWriter_writesImuSamples() {
        recorder.initialize(config)
        val result = recorder.startRecording()

        val recordingInfo = result.getOrNull()
        assertNotNull(recordingInfo)

        // Send mock data with IMU samples
        val mockData = createMockSynchronizedData(imuSampleCount = 6)
        recorder.onData(mockData)
        recorder.onData(mockData)  // Send twice

        // Stop to flush and close the file
        val summary = recorder.stopRecording().getOrNull()
        assertNotNull(summary)

        val sessionDir = File(recordingInfo!!.outputPath)
        val csvFile = File(sessionDir, "imu_data.csv")

        val lines = csvFile.readLines()
        // Header + (6 samples per frame * 2 frames)
        assertTrue(
            "CSV should have header + 12 data rows",
            lines.size >= 13  // Allow for potential extra lines
        )

        // Check first data row format
        val firstDataRow = lines[1]
        val parts = firstDataRow.split(",")
        assertEquals("Row should have 5 columns", 5, parts.size)

        // Validate timestamp is numeric
        assertTrue("Timestamp should be numeric", parts[0].toLongOrNull() != null)

        // Validate sensor type
        assertTrue(
            "Sensor type should be 'accel' or 'gyro'",
            parts[1] == "accel" || parts[1] == "gyro"
        )

        // Validate x, y, z are numeric
        assertTrue("X value should be numeric", parts[2].toFloatOrNull() != null)
        assertTrue("Y value should be numeric", parts[3].toFloatOrNull() != null)
        assertTrue("Z value should be numeric", parts[4].toFloatOrNull() != null)
    }

    @Test
    fun testImuCsvWriter_filePathInSummary() {
        recorder.initialize(config)
        val result = recorder.startRecording()

        val recordingInfo = result.getOrNull()
        assertNotNull(recordingInfo)

        // Stop recording
        val summary = recorder.stopRecording().getOrNull()
        assertNotNull(summary)

        // Check that imuFile is set in summary
        assertNotNull("Summary should contain IMU file path", summary!!.imuFile)

        val imuFile = File(summary.imuFile!!)
        assertTrue("IMU file should exist", imuFile.exists())
        assertEquals("IMU filename should be correct", "imu_data.csv", imuFile.name)
    }

    @Test
    fun testImuCsvWriter_correctSampleCount() {
        recorder.initialize(config)
        recorder.startRecording()

        // Send mock data with known IMU sample count
        val mockData = createMockSynchronizedData(imuSampleCount = 10)
        recorder.onData(mockData)
        recorder.onData(mockData)
        recorder.onData(mockData)  // 3 frames * 10 samples = 30 samples

        val summary = recorder.stopRecording().getOrNull()
        assertNotNull(summary)

        assertEquals(
            "Summary should report correct IMU sample count",
            30L,
            summary!!.imuSampleCount
        )
    }

    @Test
    fun testImuCsvWriter_closedProperlyOnStop() {
        recorder.initialize(config)
        val result = recorder.startRecording()

        val recordingInfo = result.getOrNull()
        assertNotNull(recordingInfo)

        // Send some data
        recorder.onData(createMockSynchronizedData(imuSampleCount = 5))

        // Stop recording
        val summary = recorder.stopRecording().getOrNull()
        assertNotNull(summary)

        val csvFile = File(summary!!.imuFile!!)

        // Verify file can be read (meaning it was closed properly)
        val lines = csvFile.readLines()
        assertTrue("CSV file should be readable after stop", lines.isNotEmpty())
    }

    // ========== Helper Methods ==========

    private fun createMockSynchronizedData(imuSampleCount: Int = 3): SynchronizedData {
        val timestamp = System.nanoTime()
        val imuSamplesBefore = List(imuSampleCount / 2) { index ->
            IMUSample(
                timestampNs = timestamp + index * 1_000_000,
                type = if (index % 2 == 0) SensorType.ACCELEROMETER else SensorType.GYROSCOPE,
                x = 0.1f,
                y = 0.2f,
                z = 0.3f
            )
        }
        val imuSamplesAfter = List(imuSampleCount - imuSampleCount / 2) { index ->
            IMUSample(
                timestampNs = timestamp + (imuSampleCount / 2 + index) * 1_000_000,
                type = if (index % 2 == 0) SensorType.GYROSCOPE else SensorType.ACCELEROMETER,
                x = 0.4f,
                y = 0.5f,
                z = 0.6f
            )
        }

        return SynchronizedData(
            frameTimestampNs = timestamp,
            frameSequence = 0L,
            imuSamplesBefore = imuSamplesBefore,
            imuSamplesAfter = imuSamplesAfter,
            interpolatedImu = null
        )
    }
}
