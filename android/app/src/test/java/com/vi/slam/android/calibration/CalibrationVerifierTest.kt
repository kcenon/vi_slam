package com.vi.slam.android.calibration

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.opencv.core.*

/**
 * Unit tests for CalibrationVerifier
 *
 * Note: These tests use mock OpenCV Mat objects and simulate
 * calibration data for testing verification logic.
 */
class CalibrationVerifierTest {

    private lateinit var verifier: CalibrationVerifier

    // Sample calibration result for testing
    private val goodCalibResult = IntrinsicCalibResult(
        fx = 500.0,
        fy = 500.0,
        cx = 320.0,
        cy = 240.0,
        distortionCoeffs = doubleArrayOf(-0.2, 0.05, 0.0, 0.0, 0.0),
        reprojectionError = 0.3,
        captureCount = 25,
        imageWidth = 640,
        imageHeight = 480,
        cameraModel = CameraModelType.PINHOLE
    )

    private val poorCalibResult = IntrinsicCalibResult(
        fx = 500.0,
        fy = 500.0,
        cx = 320.0,
        cy = 240.0,
        distortionCoeffs = doubleArrayOf(-0.2, 0.05, 0.0, 0.0, 0.0),
        reprojectionError = 1.5,
        captureCount = 10,
        imageWidth = 640,
        imageHeight = 480,
        cameraModel = CameraModelType.PINHOLE
    )

    @Before
    fun setUp() {
        verifier = CalibrationVerifier()

        // Initialize OpenCV if not already initialized
        try {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        } catch (e: UnsatisfiedLinkError) {
            // OpenCV not available in test environment - tests will be skipped
            println("OpenCV not available: ${e.message}")
        }
    }

    @Test
    fun testVerifyGoodCalibration() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val objectPoints = createMockObjectPoints(25, 6, 9, 0.025)
        val imagePoints = createMockImagePoints(25, 6, 9, 0.2)

        val result = verifier.verifyIntrinsicCalibration(
            goodCalibResult,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isSuccess)

        val verification = result.getOrNull()
        assertNotNull(verification)

        // Good calibration should meet quality threshold
        assertTrue(verification!!.meetsQualityThreshold)
        assertTrue(verification.qualityScore >= 60.0)

        // Should have GOOD or EXCELLENT quality
        assertTrue(
            verification.quality == CalibrationQuality.GOOD ||
            verification.quality == CalibrationQuality.EXCELLENT
        )

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    @Test
    fun testVerifyPoorCalibration() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val objectPoints = createMockObjectPoints(10, 6, 9, 0.025)
        val imagePoints = createMockImagePoints(10, 6, 9, 1.5)

        val result = verifier.verifyIntrinsicCalibration(
            poorCalibResult,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isSuccess)

        val verification = result.getOrNull()
        assertNotNull(verification)

        // Poor calibration should not meet quality threshold
        assertFalse(verification!!.meetsQualityThreshold)
        assertTrue(verification.qualityScore < 60.0)

        // Should have POOR quality
        assertEquals(CalibrationQuality.POOR, verification.quality)

        // Should have recommendations
        assertTrue(verification.recommendations.isNotEmpty())

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    @Test
    fun testReprojectionErrorStats() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val objectPoints = createMockObjectPoints(20, 6, 9, 0.025)
        val imagePoints = createMockImagePoints(20, 6, 9, 0.3)

        val result = verifier.verifyIntrinsicCalibration(
            goodCalibResult,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isSuccess)

        val verification = result.getOrNull()
        assertNotNull(verification)

        val stats = verification!!.reprojectionStats

        // Verify statistics are calculated
        assertTrue(stats.mean >= 0.0)
        assertTrue(stats.stdDev >= 0.0)
        assertTrue(stats.min >= 0.0)
        assertTrue(stats.max >= stats.min)
        assertTrue(stats.rms >= 0.0)
        assertTrue(stats.median >= 0.0)
        assertTrue(stats.percentile95 >= stats.median)

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    @Test
    fun testOutlierDetection() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val objectPoints = createMockObjectPoints(20, 6, 9, 0.025)
        // Create image points with some outliers (high error)
        val imagePoints = createMockImagePointsWithOutliers(20, 6, 9, 0.3)

        val result = verifier.verifyIntrinsicCalibration(
            goodCalibResult,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isSuccess)

        val verification = result.getOrNull()
        assertNotNull(verification)

        // Should detect some outliers
        assertTrue(verification!!.outliers.isNotEmpty())

        // Each outlier should have valid data
        verification.outliers.forEach { outlier ->
            assertTrue(outlier.captureIndex >= 0)
            assertTrue(outlier.captureIndex < 20)
            assertTrue(outlier.errorValue > 0.0)
            assertTrue(kotlin.math.abs(outlier.zScore) > 2.5)
        }

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    @Test
    fun testRecommendations() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val objectPoints = createMockObjectPoints(10, 6, 9, 0.025)
        val imagePoints = createMockImagePoints(10, 6, 9, 1.5)

        val result = verifier.verifyIntrinsicCalibration(
            poorCalibResult,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isSuccess)

        val verification = result.getOrNull()
        assertNotNull(verification)

        // Poor calibration should have recommendations
        assertTrue(verification!!.recommendations.isNotEmpty())

        // Should recommend more captures (only 10)
        val hasMoreCapturesRecommendation = verification.recommendations.any {
            it.contains("capture", ignoreCase = true)
        }
        assertTrue(hasMoreCapturesRecommendation)

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    @Test
    fun testEmptyDataValidation() {
        val result = verifier.verifyIntrinsicCalibration(
            goodCalibResult,
            emptyList(),
            emptyList(),
            Size(640.0, 480.0)
        )

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalArgumentException)
    }

    @Test
    fun testMismatchedDataValidation() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val objectPoints = createMockObjectPoints(20, 6, 9, 0.025)
        val imagePoints = createMockImagePoints(15, 6, 9, 0.3) // Different count

        val result = verifier.verifyIntrinsicCalibration(
            goodCalibResult,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isFailure)
        assertTrue(result.exceptionOrNull() is IllegalArgumentException)

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    @Test
    fun testExcellentCalibrationScore() {
        // Skip if OpenCV not available
        if (!isOpenCVAvailable()) {
            println("Skipping test: OpenCV not available")
            return
        }

        val excellentCalib = IntrinsicCalibResult(
            fx = 500.0,
            fy = 500.0,
            cx = 320.0,
            cy = 240.0,
            distortionCoeffs = doubleArrayOf(-0.2, 0.05, 0.0, 0.0, 0.0),
            reprojectionError = 0.15, // Excellent error
            captureCount = 50, // Lots of captures
            imageWidth = 640,
            imageHeight = 480,
            cameraModel = CameraModelType.PINHOLE
        )

        val objectPoints = createMockObjectPoints(50, 6, 9, 0.025)
        val imagePoints = createMockImagePoints(50, 6, 9, 0.15)

        val result = verifier.verifyIntrinsicCalibration(
            excellentCalib,
            objectPoints,
            imagePoints,
            Size(640.0, 480.0)
        )

        assertTrue(result.isSuccess)

        val verification = result.getOrNull()
        assertNotNull(verification)

        // Should have EXCELLENT quality
        assertEquals(CalibrationQuality.EXCELLENT, verification!!.quality)
        assertTrue(verification.qualityScore >= 90.0)
        assertTrue(verification.meetsQualityThreshold)

        // Clean up
        objectPoints.forEach { it.release() }
        imagePoints.forEach { it.release() }
    }

    // Helper functions

    private fun isOpenCVAvailable(): Boolean {
        return try {
            Core.VERSION
            true
        } catch (e: UnsatisfiedLinkError) {
            false
        }
    }

    /**
     * Create mock 3D object points for calibration pattern
     */
    private fun createMockObjectPoints(
        numCaptures: Int,
        rows: Int,
        cols: Int,
        squareSize: Double
    ): List<Mat> {
        return (0 until numCaptures).map {
            val mat = Mat(rows * cols, 1, CvType.CV_32FC3)
            var idx = 0
            for (i in 0 until rows) {
                for (j in 0 until cols) {
                    mat.put(
                        idx++, 0,
                        j * squareSize,
                        i * squareSize,
                        0.0
                    )
                }
            }
            mat
        }
    }

    /**
     * Create mock 2D image points (detected corners)
     */
    private fun createMockImagePoints(
        numCaptures: Int,
        rows: Int,
        cols: Int,
        noise: Double
    ): List<Mat> {
        val random = java.util.Random(42) // Fixed seed for reproducibility

        return (0 until numCaptures).map {
            val mat = Mat(rows * cols, 1, CvType.CV_32FC2)
            var idx = 0
            for (i in 0 until rows) {
                for (j in 0 until cols) {
                    // Simulate detected corner with some noise
                    val x = 320.0 + j * 30.0 + random.nextGaussian() * noise
                    val y = 240.0 + i * 30.0 + random.nextGaussian() * noise
                    mat.put(idx++, 0, x, y)
                }
            }
            mat
        }
    }

    /**
     * Create mock image points with outliers
     */
    private fun createMockImagePointsWithOutliers(
        numCaptures: Int,
        rows: Int,
        cols: Int,
        noise: Double
    ): List<Mat> {
        val random = java.util.Random(42)

        return (0 until numCaptures).map { captureIdx ->
            val mat = Mat(rows * cols, 1, CvType.CV_32FC2)
            var idx = 0

            // Add outliers to first 2 captures
            val isOutlier = captureIdx < 2
            val outlierNoise = if (isOutlier) noise * 10.0 else noise

            for (i in 0 until rows) {
                for (j in 0 until cols) {
                    val x = 320.0 + j * 30.0 + random.nextGaussian() * outlierNoise
                    val y = 240.0 + i * 30.0 + random.nextGaussian() * outlierNoise
                    mat.put(idx++, 0, x, y)
                }
            }
            mat
        }
    }
}
