package com.vi.slam.android.calibration

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Calibration quality verification and metrics
 *
 * Analyzes calibration results to provide quality metrics including:
 * - Reprojection error statistics
 * - Outlier detection
 * - Quality scoring (0-100)
 * - Recommendations for improvement
 *
 * Quality scoring factors:
 * - Mean reprojection error (40%)
 * - Error consistency (stdDev) (30%)
 * - Outlier count (20%)
 * - Capture count (10%)
 */
class CalibrationVerifier {

    companion object {
        // Quality thresholds
        private const val EXCELLENT_SCORE = 90.0
        private const val GOOD_SCORE = 75.0
        private const val ACCEPTABLE_SCORE = 60.0

        // Error thresholds (in pixels)
        private const val EXCELLENT_ERROR = 0.3
        private const val GOOD_ERROR = 0.5
        private const val ACCEPTABLE_ERROR = 1.0

        // Outlier detection threshold (Z-score)
        private const val OUTLIER_Z_SCORE = 2.5

        // Minimum captures for reliable calibration
        private const val MIN_RECOMMENDED_CAPTURES = 20
    }

    /**
     * Verify intrinsic calibration quality
     *
     * @param calibResult Calibration result to verify
     * @param objectPoints 3D calibration pattern points (one per capture)
     * @param imagePoints 2D detected corner points (one per capture)
     * @param imageSize Size of calibration images
     * @return Verification result with quality metrics
     */
    fun verifyIntrinsicCalibration(
        calibResult: IntrinsicCalibResult,
        objectPoints: List<Mat>,
        imagePoints: List<Mat>,
        imageSize: Size
    ): Result<CalibrationVerificationResult> {
        return try {
            require(objectPoints.size == imagePoints.size) {
                "Object points and image points must have same size"
            }
            require(objectPoints.isNotEmpty()) {
                "No calibration data provided"
            }

            // Calculate per-capture reprojection errors
            val perCaptureErrors = calculatePerCaptureErrors(
                calibResult,
                objectPoints,
                imagePoints,
                imageSize
            )

            // Calculate error statistics
            val stats = calculateErrorStatistics(perCaptureErrors)

            // Detect outliers
            val outliers = detectOutliers(perCaptureErrors, stats)

            // Calculate quality score
            val qualityScore = calculateQualityScore(
                stats,
                outliers.size,
                calibResult.captureCount
            )

            // Determine quality level
            val quality = when {
                qualityScore >= EXCELLENT_SCORE -> CalibrationQuality.EXCELLENT
                qualityScore >= GOOD_SCORE -> CalibrationQuality.GOOD
                qualityScore >= ACCEPTABLE_SCORE -> CalibrationQuality.ACCEPTABLE
                else -> CalibrationQuality.POOR
            }

            // Generate recommendations
            val recommendations = generateRecommendations(
                stats,
                outliers,
                calibResult.captureCount
            )

            // Check if meets quality threshold
            val meetsQualityThreshold = qualityScore >= ACCEPTABLE_SCORE &&
                    stats.mean <= ACCEPTABLE_ERROR

            Result.success(
                CalibrationVerificationResult(
                    qualityScore = qualityScore,
                    quality = quality,
                    reprojectionStats = stats,
                    outliers = outliers,
                    meetsQualityThreshold = meetsQualityThreshold,
                    recommendations = recommendations
                )
            )
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Calculate reprojection error for each capture
     */
    private fun calculatePerCaptureErrors(
        calibResult: IntrinsicCalibResult,
        objectPoints: List<Mat>,
        imagePoints: List<Mat>,
        imageSize: Size
    ): List<Double> {
        // Construct camera matrix
        val cameraMatrix = Mat.eye(3, 3, CvType.CV_64F)
        cameraMatrix.put(0, 0, calibResult.fx)
        cameraMatrix.put(1, 1, calibResult.fy)
        cameraMatrix.put(0, 2, calibResult.cx)
        cameraMatrix.put(1, 2, calibResult.cy)

        // Construct distortion coefficients
        val distCoeffs = Mat(calibResult.distortionCoeffs.size, 1, CvType.CV_64F)
        for (i in calibResult.distortionCoeffs.indices) {
            distCoeffs.put(i, 0, calibResult.distortionCoeffs[i])
        }

        val errors = mutableListOf<Double>()

        try {
            for (i in objectPoints.indices) {
                // Calculate rotation and translation for this view
                val rvec = Mat()
                val tvec = Mat()

                when (calibResult.cameraModel) {
                    CameraModelType.PINHOLE -> {
                        Calib3d.solvePnP(
                            objectPoints[i],
                            imagePoints[i],
                            cameraMatrix,
                            distCoeffs,
                            rvec,
                            tvec
                        )
                    }
                    CameraModelType.FISHEYE -> {
                        // For fisheye, use regular PnP as approximation
                        Calib3d.solvePnP(
                            objectPoints[i],
                            imagePoints[i],
                            cameraMatrix,
                            distCoeffs,
                            rvec,
                            tvec
                        )
                    }
                }

                // Project points back to image
                val projectedPoints = Mat()
                when (calibResult.cameraModel) {
                    CameraModelType.PINHOLE -> {
                        Calib3d.projectPoints(
                            objectPoints[i],
                            rvec,
                            tvec,
                            cameraMatrix,
                            distCoeffs,
                            projectedPoints
                        )
                    }
                    CameraModelType.FISHEYE -> {
                        Calib3d.fisheye_projectPoints(
                            objectPoints[i],
                            projectedPoints,
                            rvec,
                            tvec,
                            cameraMatrix,
                            distCoeffs
                        )
                    }
                }

                // Calculate error for this capture
                val error = calculateReprojectionError(
                    imagePoints[i],
                    projectedPoints
                )
                errors.add(error)

                // Clean up
                rvec.release()
                tvec.release()
                projectedPoints.release()
            }
        } finally {
            cameraMatrix.release()
            distCoeffs.release()
        }

        return errors
    }

    /**
     * Calculate RMS reprojection error between observed and projected points
     */
    private fun calculateReprojectionError(
        imagePoints: Mat,
        projectedPoints: Mat
    ): Double {
        var sumSquaredError = 0.0
        var pointCount = 0

        for (i in 0 until imagePoints.rows()) {
            val observed = imagePoints.get(i, 0)
            val projected = projectedPoints.get(i, 0)

            val dx = observed[0] - projected[0]
            val dy = observed[1] - projected[1]
            sumSquaredError += dx * dx + dy * dy
            pointCount++
        }

        return if (pointCount > 0) {
            sqrt(sumSquaredError / pointCount)
        } else {
            0.0
        }
    }

    /**
     * Calculate comprehensive error statistics
     */
    private fun calculateErrorStatistics(errors: List<Double>): ReprojectionErrorStats {
        require(errors.isNotEmpty()) { "No errors to calculate statistics" }

        val sortedErrors = errors.sorted()
        val mean = errors.average()

        // Calculate standard deviation
        val variance = errors.map { (it - mean).pow(2) }.average()
        val stdDev = sqrt(variance)

        val min = sortedErrors.first()
        val max = sortedErrors.last()

        // Calculate RMS
        val rms = sqrt(errors.map { it * it }.average())

        // Calculate median
        val median = if (sortedErrors.size % 2 == 0) {
            (sortedErrors[sortedErrors.size / 2 - 1] + sortedErrors[sortedErrors.size / 2]) / 2.0
        } else {
            sortedErrors[sortedErrors.size / 2]
        }

        // Calculate 95th percentile
        val percentile95Index = (sortedErrors.size * 0.95).toInt().coerceAtMost(sortedErrors.size - 1)
        val percentile95 = sortedErrors[percentile95Index]

        return ReprojectionErrorStats(
            mean = mean,
            stdDev = stdDev,
            min = min,
            max = max,
            rms = rms,
            median = median,
            percentile95 = percentile95
        )
    }

    /**
     * Detect outlier captures using Z-score method
     */
    private fun detectOutliers(
        errors: List<Double>,
        stats: ReprojectionErrorStats
    ): List<OutlierInfo> {
        if (stats.stdDev == 0.0) {
            return emptyList()
        }

        return errors.mapIndexedNotNull { index, error ->
            val zScore = (error - stats.mean) / stats.stdDev
            if (kotlin.math.abs(zScore) > OUTLIER_Z_SCORE) {
                OutlierInfo(
                    captureIndex = index,
                    errorValue = error,
                    zScore = zScore
                )
            } else {
                null
            }
        }
    }

    /**
     * Calculate overall quality score (0-100)
     *
     * Scoring factors:
     * - Mean error (40%): Lower is better
     * - Error consistency (30%): Lower stdDev is better
     * - Outlier count (20%): Fewer outliers is better
     * - Capture count (10%): More captures is better
     */
    private fun calculateQualityScore(
        stats: ReprojectionErrorStats,
        outlierCount: Int,
        captureCount: Int
    ): Double {
        // Score for mean error (40%)
        val errorScore = when {
            stats.mean <= EXCELLENT_ERROR -> 100.0
            stats.mean <= GOOD_ERROR -> {
                // Linear interpolation between excellent and good
                100.0 - ((stats.mean - EXCELLENT_ERROR) / (GOOD_ERROR - EXCELLENT_ERROR)) * 10.0
            }
            stats.mean <= ACCEPTABLE_ERROR -> {
                // Linear interpolation between good and acceptable
                90.0 - ((stats.mean - GOOD_ERROR) / (ACCEPTABLE_ERROR - GOOD_ERROR)) * 30.0
            }
            else -> {
                // Rapidly decrease score for poor errors
                60.0 * kotlin.math.exp(-(stats.mean - ACCEPTABLE_ERROR))
            }
        } * 0.4

        // Score for error consistency (30%)
        val consistencyScore = when {
            stats.stdDev <= 0.1 -> 100.0
            stats.stdDev <= 0.3 -> 90.0 - ((stats.stdDev - 0.1) / 0.2) * 20.0
            stats.stdDev <= 0.5 -> 70.0 - ((stats.stdDev - 0.3) / 0.2) * 30.0
            else -> 40.0 * kotlin.math.exp(-(stats.stdDev - 0.5))
        } * 0.3

        // Score for outliers (20%)
        val outlierRatio = outlierCount.toDouble() / captureCount
        val outlierScore = when {
            outlierRatio == 0.0 -> 100.0
            outlierRatio <= 0.05 -> 90.0
            outlierRatio <= 0.10 -> 75.0
            outlierRatio <= 0.20 -> 50.0
            else -> 25.0
        } * 0.2

        // Score for capture count (10%)
        val captureScore = when {
            captureCount >= MIN_RECOMMENDED_CAPTURES * 2 -> 100.0
            captureCount >= MIN_RECOMMENDED_CAPTURES -> {
                80.0 + ((captureCount - MIN_RECOMMENDED_CAPTURES).toDouble() /
                        MIN_RECOMMENDED_CAPTURES) * 20.0
            }
            captureCount >= MIN_RECOMMENDED_CAPTURES / 2 -> {
                60.0 + ((captureCount - MIN_RECOMMENDED_CAPTURES / 2).toDouble() /
                        (MIN_RECOMMENDED_CAPTURES / 2)) * 20.0
            }
            else -> 40.0
        } * 0.1

        return (errorScore + consistencyScore + outlierScore + captureScore)
            .coerceIn(0.0, 100.0)
    }

    /**
     * Generate recommendations for improving calibration quality
     */
    private fun generateRecommendations(
        stats: ReprojectionErrorStats,
        outliers: List<OutlierInfo>,
        captureCount: Int
    ): List<String> {
        val recommendations = mutableListOf<String>()

        // Check mean error
        when {
            stats.mean > ACCEPTABLE_ERROR -> {
                recommendations.add(
                    "High mean reprojection error (${String.format("%.3f", stats.mean)}px). " +
                    "Consider recalibrating with better pattern detection or more stable captures."
                )
            }
            stats.mean > GOOD_ERROR -> {
                recommendations.add(
                    "Moderate reprojection error (${String.format("%.3f", stats.mean)}px). " +
                    "Quality is acceptable but could be improved with more careful captures."
                )
            }
        }

        // Check error consistency
        if (stats.stdDev > 0.3) {
            recommendations.add(
                "High error variance (stdDev: ${String.format("%.3f", stats.stdDev)}px). " +
                "Some captures have significantly higher errors. Consider removing outliers."
            )
        }

        // Check outliers
        if (outliers.isNotEmpty()) {
            val outlierRatio = outliers.size.toDouble() / captureCount
            if (outlierRatio > 0.1) {
                recommendations.add(
                    "High number of outliers (${outliers.size}/${captureCount} = " +
                    "${String.format("%.1f", outlierRatio * 100)}%). " +
                    "Consider recalibrating and ensuring stable capture conditions."
                )
            } else {
                recommendations.add(
                    "${outliers.size} outlier capture(s) detected. " +
                    "Consider removing these captures and recalibrating for better results."
                )
            }
        }

        // Check capture count
        when {
            captureCount < MIN_RECOMMENDED_CAPTURES / 2 -> {
                recommendations.add(
                    "Very few captures (${captureCount}). " +
                    "Recommend at least ${MIN_RECOMMENDED_CAPTURES} captures for reliable calibration."
                )
            }
            captureCount < MIN_RECOMMENDED_CAPTURES -> {
                recommendations.add(
                    "Capture count (${captureCount}) is below recommended (${MIN_RECOMMENDED_CAPTURES}). " +
                    "More captures will improve calibration reliability."
                )
            }
        }

        // Check max error
        if (stats.max > 2.0) {
            recommendations.add(
                "Maximum error is very high (${String.format("%.3f", stats.max)}px). " +
                "This may indicate a problematic capture that should be removed."
            )
        }

        // If no recommendations, add positive feedback
        if (recommendations.isEmpty()) {
            recommendations.add("Calibration quality is excellent. No improvements needed.")
        }

        return recommendations
    }
}
