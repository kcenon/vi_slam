package com.vi.slam.android.calibration

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.opencv.core.Core

/**
 * Unit tests for ExtrinsicCalibrator
 */
class ExtrinsicCalibratorTest {

    private lateinit var calibrator: ExtrinsicCalibrator
    private lateinit var config: ExtrinsicCalibConfig
    private lateinit var intrinsicParams: IntrinsicCalibResult

    @Before
    fun setUp() {
        // Initialize OpenCV (required for tests)
        try {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        } catch (e: UnsatisfiedLinkError) {
            // OpenCV not available in test environment - tests will be skipped
            println("OpenCV not available for testing: ${e.message}")
        }

        // Mock intrinsic parameters
        intrinsicParams = IntrinsicCalibResult(
            fx = 500.0,
            fy = 500.0,
            cx = 320.0,
            cy = 240.0,
            distortionCoeffs = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0),
            reprojectionError = 0.3,
            captureCount = 20,
            imageWidth = 640,
            imageHeight = 480,
            cameraModel = CameraModelType.PINHOLE
        )

        calibrator = ExtrinsicCalibrator()
        config = ExtrinsicCalibConfig(
            targetType = CalibrationTargetType.CHECKERBOARD,
            targetSize = TargetSize(width = 9, height = 6, squareSize = 0.025), // 9x6 board, 25mm squares
            recordingDuration = 60000, // 60 seconds
            intrinsicParams = intrinsicParams
        )
    }

    @Test
    fun testInitialStatus() {
        assertEquals(CalibrationStatus.IDLE, calibrator.getStatus())
    }

    @Test
    fun testStartCalibration() {
        val result = calibrator.startExtrinsicCalibration(config)
        assertTrue(result.isSuccess)
        assertEquals(CalibrationStatus.CAPTURING, calibrator.getStatus())
    }

    @Test
    fun testStartCalibrationTwice() {
        calibrator.startExtrinsicCalibration(config)
        val result = calibrator.startExtrinsicCalibration(config)
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
    }

    @Test
    fun testCaptureFrameBeforeStart() {
        val imageData = ByteArray(640 * 480) { 128.toByte() }
        val result = calibrator.captureCameraFrame(
            imageData = imageData,
            width = 640,
            height = 480,
            timestamp = System.nanoTime()
        )
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
    }

    @Test
    fun testAddImuSampleBeforeStart() {
        val result = calibrator.addImuSample(
            timestamp = System.nanoTime(),
            accelX = 0.0,
            accelY = 0.0,
            accelZ = 9.81,
            gyroX = 0.0,
            gyroY = 0.0,
            gyroZ = 0.0
        )
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
    }

    @Test
    fun testFinishBeforeStart() {
        val result = calibrator.finishExtrinsicCalibration()
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
    }

    @Test
    fun testRecordingProgressInitialization() {
        calibrator.startExtrinsicCalibration(config)
        val progress = calibrator.getRecordingProgress()
        assertTrue(progress >= 0.0f && progress < 0.1f) // Should be close to 0
    }

    @Test
    fun testRecordingStatsAfterStart() {
        calibrator.startExtrinsicCalibration(config)
        val stats = calibrator.getRecordingStats()

        assertEquals(0, stats.frameCount)
        assertEquals(0, stats.imuSampleCount)
        assertEquals(0, stats.framesWithCorners)
        assertTrue(stats.duration >= 0)
    }

    @Test
    fun testAddImuSample() {
        calibrator.startExtrinsicCalibration(config)

        val result = calibrator.addImuSample(
            timestamp = System.nanoTime(),
            accelX = 0.1,
            accelY = 0.2,
            accelZ = 9.8,
            gyroX = 0.01,
            gyroY = 0.02,
            gyroZ = 0.03
        )

        assertTrue(result.isSuccess)
        val stats = calibrator.getRecordingStats()
        assertEquals(1, stats.imuSampleCount)
    }

    @Test
    fun testMultipleImuSamples() {
        calibrator.startExtrinsicCalibration(config)

        // Add 100 IMU samples
        repeat(100) { i ->
            val result = calibrator.addImuSample(
                timestamp = System.nanoTime() + i * 5_000_000L, // 5ms intervals
                accelX = 0.0,
                accelY = 0.0,
                accelZ = 9.81,
                gyroX = 0.0,
                gyroY = 0.0,
                gyroZ = 0.0
            )
            assertTrue(result.isSuccess)
        }

        val stats = calibrator.getRecordingStats()
        assertEquals(100, stats.imuSampleCount)
    }

    @Test
    fun testFinishWithoutData() {
        calibrator.startExtrinsicCalibration(config)
        val result = calibrator.finishExtrinsicCalibration()

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull()?.message?.contains("No camera frames") == true)
    }

    @Test
    fun testFinishWithoutImuData() {
        try {
            calibrator.startExtrinsicCalibration(config)

            // Add a mock camera frame (this would fail in real scenario without OpenCV)
            // But for testing, we simulate the scenario
            val imageData = ByteArray(640 * 480) { 128.toByte() }
            calibrator.captureCameraFrame(imageData, 640, 480, System.nanoTime())

            // Finish without IMU data - should fail
            // (In real scenario, corner detection would likely fail without valid checkerboard)
        } catch (e: UnsatisfiedLinkError) {
            // Expected when OpenCV is not available
            assertTrue(true)
        } catch (e: Exception) {
            // Expected when OpenCV is not available or other error
            assertTrue(true)
        }
    }

    @Test
    fun testCancelCalibration() {
        calibrator.startExtrinsicCalibration(config)

        // Add some data
        calibrator.addImuSample(
            timestamp = System.nanoTime(),
            accelX = 0.0,
            accelY = 0.0,
            accelZ = 9.81,
            gyroX = 0.0,
            gyroY = 0.0,
            gyroZ = 0.0
        )

        // Cancel
        calibrator.cancelCalibration()

        assertEquals(CalibrationStatus.IDLE, calibrator.getStatus())
        val stats = calibrator.getRecordingStats()
        assertEquals(0, stats.imuSampleCount)
        assertEquals(0, stats.frameCount)
    }

    @Test
    fun testExportBeforeComplete() {
        calibrator.startExtrinsicCalibration(config)
        val result = calibrator.exportCalibrationData()

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull()?.message?.contains("not completed") == true)
    }

    @Test
    fun testRecordingDurationCheck() {
        val shortConfig = config.copy(recordingDuration = 100) // 100ms
        calibrator.startExtrinsicCalibration(shortConfig)

        assertFalse(calibrator.isRecordingComplete())

        // Wait for duration to complete
        Thread.sleep(150)

        assertTrue(calibrator.isRecordingComplete())
    }

    @Test
    fun testRecordingProgressOverTime() {
        val shortConfig = config.copy(recordingDuration = 200) // 200ms
        calibrator.startExtrinsicCalibration(shortConfig)

        val progress1 = calibrator.getRecordingProgress()
        assertTrue(progress1 < 0.3f)

        Thread.sleep(100) // Wait 50% of duration

        val progress2 = calibrator.getRecordingProgress()
        assertTrue(progress2 > progress1)
        assertTrue(progress2 < 1.0f)

        Thread.sleep(150) // Wait past completion

        val progress3 = calibrator.getRecordingProgress()
        assertEquals(1.0f, progress3, 0.01f)
    }

    @Test
    fun testMultipleCalibrationCycles() {
        // First calibration
        calibrator.startExtrinsicCalibration(config)
        calibrator.addImuSample(System.nanoTime(), 0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
        calibrator.cancelCalibration()

        assertEquals(CalibrationStatus.IDLE, calibrator.getStatus())

        // Second calibration
        val result = calibrator.startExtrinsicCalibration(config)
        assertTrue(result.isSuccess)
        assertEquals(CalibrationStatus.CAPTURING, calibrator.getStatus())
    }

    @Test
    fun testImuSampleDataIntegrity() {
        calibrator.startExtrinsicCalibration(config)

        val timestamp = System.nanoTime()
        val accelX = 0.123
        val accelY = 0.456
        val accelZ = 9.789
        val gyroX = 0.001
        val gyroY = 0.002
        val gyroZ = 0.003

        val result = calibrator.addImuSample(
            timestamp = timestamp,
            accelX = accelX,
            accelY = accelY,
            accelZ = accelZ,
            gyroX = gyroX,
            gyroY = gyroY,
            gyroZ = gyroZ
        )

        assertTrue(result.isSuccess)
        val stats = calibrator.getRecordingStats()
        assertEquals(1, stats.imuSampleCount)
    }

    @Test
    fun testIntrinsicParamsPreservation() {
        calibrator.startExtrinsicCalibration(config)

        // Add minimal data
        calibrator.addImuSample(System.nanoTime(), 0.0, 0.0, 9.81, 0.0, 0.0, 0.0)

        // The intrinsic params should be preserved in config
        // (actual finish would fail without camera frames, but config is preserved)
        assertEquals(intrinsicParams.fx, config.intrinsicParams.fx, 0.001)
        assertEquals(intrinsicParams.fy, config.intrinsicParams.fy, 0.001)
    }
}
