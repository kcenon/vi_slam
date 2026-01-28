package com.vi.slam.android.recorder

import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.File

/**
 * Unit tests for data recovery functionality.
 *
 * Tests CSV validation, MP4 recovery, and session state management.
 */
class RecoveryTest {

    @get:Rule
    val tempFolder = TemporaryFolder()

    private lateinit var testDir: File

    @Before
    fun setUp() {
        testDir = tempFolder.newFolder("recovery_test")
    }

    // ========== CSV Recovery Tests ==========

    @Test
    fun testCsvRecovery_validFile() {
        // Create valid CSV file
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8
            1000100000,gyro,0.01,-0.02,0.00
            1000200000,accel,0.15,0.25,9.75
        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV validation should succeed", result.success)
        assertEquals("Should have 4 total lines (header + 3 data)", 4, result.totalLines)
        assertEquals("Should have 4 valid lines", 4, result.validLines)
        assertEquals("Should remove 0 lines", 0, result.removedLines)
    }

    @Test
    fun testCsvRecovery_missingHeader() {
        // Create CSV without header
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            1000000000,accel,0.1,0.2,9.8
            1000100000,gyro,0.01,-0.02,0.00
        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV repair should succeed", result.success)
        assertTrue("Should add header", result.validLines > 0)

        // Verify header was added
        val lines = csvFile.readLines()
        assertEquals("First line should be header", "timestamp_ns,sensor_type,x,y,z", lines[0])
    }

    @Test
    fun testCsvRecovery_truncatedLine() {
        // Create CSV with truncated line
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8
            1000100000,gyro,0.01
            1000200000,accel,0.15,0.25,9.75
        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV repair should succeed", result.success)
        assertEquals("Should have 4 total lines", 4, result.totalLines)
        assertEquals("Should have 3 valid lines (header + 2 data)", 3, result.validLines)
        assertEquals("Should remove 1 truncated line", 1, result.removedLines)
    }

    @Test
    fun testCsvRecovery_emptyLines() {
        // Create CSV with empty lines
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8

            1000100000,gyro,0.01,-0.02,0.00

        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV repair should succeed", result.success)
        assertEquals("Should remove 2 empty lines", 2, result.removedLines)
    }

    @Test
    fun testCsvRecovery_invalidTimestamp() {
        // Create CSV with invalid timestamp
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8
            invalid,gyro,0.01,-0.02,0.00
            1000200000,accel,0.15,0.25,9.75
        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV repair should succeed", result.success)
        assertEquals("Should remove 1 invalid line", 1, result.removedLines)
    }

    @Test
    fun testCsvRecovery_invalidSensorType() {
        // Create CSV with invalid sensor type
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8
            1000100000,invalid_type,0.01,-0.02,0.00
            1000200000,gyro,0.15,0.25,9.75
        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV repair should succeed", result.success)
        assertEquals("Should remove 1 line with invalid sensor type", 1, result.removedLines)
    }

    @Test
    fun testCsvRecovery_invalidNumericValues() {
        // Create CSV with non-numeric x, y, z values
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8
            1000100000,gyro,invalid,0.02,0.00
            1000200000,accel,0.15,0.25,9.75
        """.trimIndent())

        // Validate and repair
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertTrue("CSV repair should succeed", result.success)
        assertEquals("Should remove 1 line with invalid values", 1, result.removedLines)
    }

    @Test
    fun testCsvRecovery_nonExistentFile() {
        val csvFile = File(testDir, "nonexistent.csv")

        // Attempt to recover non-existent file
        val result = CsvRecovery.validateAndRepair(csvFile)

        assertFalse("CSV repair should fail for non-existent file", result.success)
        assertNotNull("Should have error message", result.errorMessage)
    }

    @Test
    fun testCsvCountValidLines() {
        // Create CSV file
        val csvFile = File(testDir, "test.csv")
        csvFile.writeText("""
            timestamp_ns,sensor_type,x,y,z
            1000000000,accel,0.1,0.2,9.8
            1000100000,gyro,0.01,-0.02,0.00
            1000200000,accel,0.15,0.25,9.75
        """.trimIndent())

        // Count valid lines
        val count = CsvRecovery.countValidLines(csvFile)

        assertEquals("Should count 3 data lines (excluding header)", 3, count)
    }

    // ========== MP4 Recovery Tests ==========

    @Test
    fun testMp4Recovery_nonExistentFile() {
        val videoFile = File(testDir, "nonexistent.mp4")

        // Attempt to recover non-existent file
        val result = Mp4Recovery.recoverMp4(videoFile)

        assertFalse("MP4 recovery should fail for non-existent file", result.success)
        assertFalse("File should not be playable", result.playable)
        assertNotNull("Should have error message", result.errorMessage)
    }

    @Test
    fun testMp4Recovery_emptyFile() {
        val videoFile = File(testDir, "empty.mp4")
        videoFile.createNewFile()

        // Attempt to recover empty file
        val result = Mp4Recovery.recoverMp4(videoFile)

        assertFalse("MP4 recovery should fail for empty file", result.success)
        assertFalse("File should not be playable", result.playable)
        assertEquals("Error message should indicate empty file", "Video file is empty", result.errorMessage)
    }

    @Test
    fun testMp4IsPlayable_nonExistentFile() {
        val videoFile = File(testDir, "nonexistent.mp4")

        // Check if non-existent file is playable
        val isPlayable = Mp4Recovery.isPlayable(videoFile)

        assertFalse("Non-existent file should not be playable", isPlayable)
    }

    @Test
    fun testMp4IsPlayable_emptyFile() {
        val videoFile = File(testDir, "empty.mp4")
        videoFile.createNewFile()

        // Check if empty file is playable
        val isPlayable = Mp4Recovery.isPlayable(videoFile)

        assertFalse("Empty file should not be playable", isPlayable)
    }

    // Note: Testing actual MP4 recovery requires a real MP4 file with MediaCodec support,
    // which is not available in unit test environment. Integration tests would be needed
    // to test full MP4 recovery with MediaExtractor.

    // ========== RecoverableSession Tests ==========

    @Test
    fun testRecoverableSession_dataClass() {
        val session = RecoverableSession(
            recordingId = "test_123",
            startTime = 1000000L,
            outputPath = "/path/to/output",
            estimatedFrameCount = 100L,
            estimatedImuCount = 500L,
            lastUpdateTime = 1001000L
        )

        assertEquals("Recording ID should match", "test_123", session.recordingId)
        assertEquals("Start time should match", 1000000L, session.startTime)
        assertEquals("Output path should match", "/path/to/output", session.outputPath)
        assertEquals("Frame count should match", 100L, session.estimatedFrameCount)
        assertEquals("IMU count should match", 500L, session.estimatedImuCount)
        assertEquals("Last update time should match", 1001000L, session.lastUpdateTime)
    }

    @Test
    fun testRecoveryResult_dataClass() {
        val result = RecoveryResult(
            recordingId = "test_123",
            success = true,
            recoveredFrameCount = 90L,
            recoveredImuCount = 480L,
            videoRecovered = true,
            imuRecovered = true,
            metadataGenerated = true,
            outputFiles = listOf("/path/to/video.mp4", "/path/to/imu.csv"),
            videoFile = "/path/to/video.mp4",
            imuFile = "/path/to/imu.csv",
            metadataFile = "/path/to/metadata.json",
            errorMessage = null
        )

        assertEquals("Recording ID should match", "test_123", result.recordingId)
        assertTrue("Recovery should be successful", result.success)
        assertEquals("Frame count should match", 90L, result.recoveredFrameCount)
        assertEquals("IMU count should match", 480L, result.recoveredImuCount)
        assertTrue("Video should be recovered", result.videoRecovered)
        assertTrue("IMU should be recovered", result.imuRecovered)
        assertTrue("Metadata should be generated", result.metadataGenerated)
        assertEquals("Should have 2 output files", 2, result.outputFiles.size)
        assertNull("Error message should be null", result.errorMessage)
    }

    @Test
    fun testRecoveryResult_partialFailure() {
        val result = RecoveryResult(
            recordingId = "test_456",
            success = true,
            recoveredFrameCount = 0L,
            recoveredImuCount = 480L,
            videoRecovered = false,
            imuRecovered = true,
            metadataGenerated = true,
            outputFiles = listOf("/path/to/imu.csv"),
            videoFile = null,
            imuFile = "/path/to/imu.csv",
            metadataFile = "/path/to/metadata.json",
            errorMessage = null
        )

        assertTrue("Recovery should be successful (partial)", result.success)
        assertFalse("Video should not be recovered", result.videoRecovered)
        assertTrue("IMU should be recovered", result.imuRecovered)
        assertNull("Video file path should be null", result.videoFile)
        assertNotNull("IMU file path should not be null", result.imuFile)
    }
}
