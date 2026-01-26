package com.vi.slam.android.calibration

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.opencv.core.Core

/**
 * Unit tests for IntrinsicCalibrator
 */
class IntrinsicCalibratorTest {

    private lateinit var calibrator: IntrinsicCalibrator
    private lateinit var config: IntrinsicCalibConfig

    @Before
    fun setUp() {
        // Initialize OpenCV (required for tests)
        try {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        } catch (e: UnsatisfiedLinkError) {
            // OpenCV not available in test environment - tests will be skipped
            println("OpenCV not available for testing: ${e.message}")
        }

        calibrator = IntrinsicCalibrator()
        config = IntrinsicCalibConfig(
            targetType = CalibrationTargetType.CHECKERBOARD,
            targetSize = TargetSize(width = 9, height = 6, squareSize = 0.025), // 9x6 board, 25mm squares
            cameraModel = CameraModelType.PINHOLE,
            minCaptures = 10,
            coverageRegions = 9,
            maxReprojectionError = 0.5
        )
    }

    @Test
    fun testInitialStatus() {
        assertEquals(CalibrationStatus.IDLE, calibrator.getCalibrationStatus())
    }

    @Test
    fun testStartCalibration() {
        val result = calibrator.startIntrinsicCalibration(config)
        assertTrue(result.isSuccess)
        assertEquals(CalibrationStatus.CAPTURING, calibrator.getCalibrationStatus())
    }

    @Test
    fun testStartCalibrationTwice() {
        calibrator.startIntrinsicCalibration(config)
        val result = calibrator.startIntrinsicCalibration(config)
        assertTrue(result.isFailure)
    }

    @Test
    fun testCaptureBeforeStart() {
        val result = calibrator.captureCalibrationImage()
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
    }

    @Test
    fun testFinishBeforeStart() {
        val result = calibrator.finishIntrinsicCalibration()
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalStateException)
    }

    @Test
    fun testCoverageMapInitialization() {
        calibrator.startIntrinsicCalibration(config)
        val coverageMap = calibrator.getCoverageMap()

        assertEquals(9, coverageMap.regions.size)
        assertEquals(0, coverageMap.coveredRegions)
        assertEquals(0.0f, coverageMap.progress, 0.001f)
    }

    @Test
    fun testFinishWithoutEnoughCaptures() {
        calibrator.startIntrinsicCalibration(config)

        // Try to finish without capturing enough images
        val result = calibrator.finishIntrinsicCalibration()
        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull()?.message?.contains("Not enough captures") == true)
    }

    @Test
    fun testCaptureWithSyntheticData() {
        // Skip if OpenCV not available
        try {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        } catch (e: UnsatisfiedLinkError) {
            println("Skipping test - OpenCV not available")
            return
        }

        calibrator.startIntrinsicCalibration(config)

        // Create a simple grayscale image (640x480)
        val width = 640
        val height = 480
        val imageData = ByteArray(width * height) { 128.toByte() } // Gray image

        // Attempt to capture (will fail to detect corners in uniform gray image)
        val result = calibrator.captureCalibrationImage(imageData, width, height)

        assertTrue(result.isSuccess)
        val capture = result.getOrNull()
        assertNotNull(capture)
        assertFalse(capture!!.cornersDetected) // Should not detect corners in gray image
    }

    @Test
    fun testDataModelEquality() {
        val result1 = IntrinsicCalibResult(
            fx = 500.0,
            fy = 500.0,
            cx = 320.0,
            cy = 240.0,
            distortionCoeffs = doubleArrayOf(0.1, -0.2, 0.0, 0.0, 0.05),
            reprojectionError = 0.3,
            captureCount = 20,
            imageWidth = 640,
            imageHeight = 480,
            cameraModel = CameraModelType.PINHOLE
        )

        val result2 = IntrinsicCalibResult(
            fx = 500.0,
            fy = 500.0,
            cx = 320.0,
            cy = 240.0,
            distortionCoeffs = doubleArrayOf(0.1, -0.2, 0.0, 0.0, 0.05),
            reprojectionError = 0.3,
            captureCount = 20,
            imageWidth = 640,
            imageHeight = 480,
            cameraModel = CameraModelType.PINHOLE
        )

        assertEquals(result1, result2)
        assertEquals(result1.hashCode(), result2.hashCode())
    }

    @Test
    fun testCoverageMapProgress() {
        val coverageMap = CoverageMap(
            regions = arrayOf(true, true, false, true, false, false, false, false, false),
            requiredRegions = 9
        )

        assertEquals(3, coverageMap.coveredRegions)
        assertEquals(3.0f / 9.0f, coverageMap.progress, 0.001f)
    }

    @Test
    fun testCalibrationTargetTypes() {
        val types = CalibrationTargetType.values()
        assertTrue(types.contains(CalibrationTargetType.CHECKERBOARD))
        assertTrue(types.contains(CalibrationTargetType.APRILGRID))
        assertTrue(types.contains(CalibrationTargetType.CIRCLES_GRID))
    }

    @Test
    fun testCameraModelTypes() {
        val types = CameraModelType.values()
        assertTrue(types.contains(CameraModelType.PINHOLE))
        assertTrue(types.contains(CameraModelType.FISHEYE))
    }
}
