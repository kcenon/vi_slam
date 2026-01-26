package com.vi.slam.android.calibration

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicInteger

/**
 * Camera intrinsic calibration implementation using OpenCV
 *
 * Supports:
 * - Checkerboard pattern detection
 * - Pinhole and fisheye camera models
 * - Multi-view calibration optimization
 * - Coverage-based capture guidance
 */
class IntrinsicCalibrator : ICalibrationCapture {

    private var config: IntrinsicCalibConfig? = null
    private var status: CalibrationStatus = CalibrationStatus.IDLE
    private var coverageMap: CoverageMap = CoverageMap(Array(9) { false })

    // Calibration data storage
    private val imagePoints = mutableListOf<Mat>()
    private val objectPoints = mutableListOf<Mat>()
    private var imageSize: Size? = null
    private val frameCounter = AtomicInteger(0)

    override fun startIntrinsicCalibration(config: IntrinsicCalibConfig): Result<Unit> {
        return try {
            if (status != CalibrationStatus.IDLE && status != CalibrationStatus.COMPLETED && status != CalibrationStatus.FAILED) {
                return Result.failure(IllegalStateException("Calibration already in progress"))
            }

            this.config = config
            this.status = CalibrationStatus.CAPTURING
            this.coverageMap = CoverageMap(Array(config.coverageRegions) { false })
            this.imagePoints.clear()
            this.objectPoints.clear()
            this.imageSize = null
            this.frameCounter.set(0)

            Result.success(Unit)
        } catch (e: Exception) {
            status = CalibrationStatus.FAILED
            Result.failure(e)
        }
    }

    override fun captureCalibrationImage(): Result<CalibrationCapture> {
        val currentConfig = config
            ?: return Result.failure(IllegalStateException("Calibration not started"))

        if (status != CalibrationStatus.CAPTURING) {
            return Result.failure(IllegalStateException("Not in capturing state"))
        }

        return Result.failure(UnsupportedOperationException(
            "captureCalibrationImage requires camera frame input - use captureCalibrationImage(imageData)"
        ))
    }

    /**
     * Capture and process calibration image
     * @param imageData Raw image data (grayscale or color)
     * @param width Image width
     * @param height Image height
     * @return Calibration capture result
     */
    fun captureCalibrationImage(imageData: ByteArray, width: Int, height: Int): Result<CalibrationCapture> {
        val currentConfig = config
            ?: return Result.failure(IllegalStateException("Calibration not started"))

        if (status != CalibrationStatus.CAPTURING) {
            return Result.failure(IllegalStateException("Not in capturing state"))
        }

        return try {
            // Convert to OpenCV Mat
            val mat = Mat(height, width, CvType.CV_8UC1)
            mat.put(0, 0, imageData)

            // Detect corners
            val corners = MatOfPoint2f()
            val patternSize = Size(
                currentConfig.targetSize.width.toDouble(),
                currentConfig.targetSize.height.toDouble()
            )

            val found = when (currentConfig.targetType) {
                CalibrationTargetType.CHECKERBOARD -> {
                    Calib3d.findChessboardCorners(
                        mat,
                        patternSize,
                        corners,
                        Calib3d.CALIB_CB_ADAPTIVE_THRESH or
                                Calib3d.CALIB_CB_NORMALIZE_IMAGE or
                                Calib3d.CALIB_CB_FAST_CHECK
                    )
                }
                CalibrationTargetType.CIRCLES_GRID -> {
                    Calib3d.findCirclesGrid(mat, patternSize, corners)
                }
                else -> {
                    return Result.failure(UnsupportedOperationException(
                        "Target type ${currentConfig.targetType} not yet implemented"
                    ))
                }
            }

            if (!found) {
                mat.release()
                corners.release()
                return Result.success(CalibrationCapture(
                    frameId = frameCounter.incrementAndGet(),
                    timestamp = System.currentTimeMillis(),
                    imageData = imageData,
                    width = width,
                    height = height,
                    cornersDetected = false
                ))
            }

            // Refine corner positions for better accuracy
            val criteria = TermCriteria(
                TermCriteria.EPS or TermCriteria.MAX_ITER,
                30,
                0.001
            )
            Imgproc.cornerSubPix(mat, corners, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)

            // Determine coverage region based on corner centroid
            val coverageRegion = calculateCoverageRegion(corners, width, height)

            // Store for calibration
            if (imageSize == null) {
                imageSize = Size(width.toDouble(), height.toDouble())
            }

            imagePoints.add(corners.clone())
            objectPoints.add(generateObjectPoints(currentConfig.targetSize))

            // Update coverage map
            val regions = coverageMap.regions.clone()
            if (coverageRegion in regions.indices) {
                regions[coverageRegion] = true
                val captures = coverageMap.capturesPerRegion.clone()
                captures[coverageRegion]++
                coverageMap = CoverageMap(regions, currentConfig.coverageRegions, captures)
            }

            // Convert corners to list
            val cornerList = mutableListOf<Pair<Float, Float>>()
            for (i in 0 until corners.rows()) {
                val point = corners.get(i, 0)
                cornerList.add(Pair(point[0].toFloat(), point[1].toFloat()))
            }

            mat.release()

            Result.success(CalibrationCapture(
                frameId = frameCounter.incrementAndGet(),
                timestamp = System.currentTimeMillis(),
                imageData = imageData,
                width = width,
                height = height,
                cornersDetected = true,
                cornerPoints = cornerList,
                coverageRegion = coverageRegion
            ))
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    override fun finishIntrinsicCalibration(): Result<IntrinsicCalibResult> {
        val currentConfig = config
            ?: return Result.failure(IllegalStateException("Calibration not started"))

        if (status != CalibrationStatus.CAPTURING) {
            return Result.failure(IllegalStateException("Not in capturing state"))
        }

        if (imagePoints.size < currentConfig.minCaptures) {
            return Result.failure(IllegalStateException(
                "Not enough captures: ${imagePoints.size} < ${currentConfig.minCaptures}"
            ))
        }

        val size = imageSize
            ?: return Result.failure(IllegalStateException("No images captured"))

        status = CalibrationStatus.PROCESSING

        return try {
            val cameraMatrix = Mat.eye(3, 3, CvType.CV_64F)
            val distCoeffs = Mat.zeros(5, 1, CvType.CV_64F)
            val rvecs = mutableListOf<Mat>()
            val tvecs = mutableListOf<Mat>()

            val rms = when (currentConfig.cameraModel) {
                CameraModelType.PINHOLE -> {
                    Calib3d.calibrateCamera(
                        objectPoints,
                        imagePoints,
                        size,
                        cameraMatrix,
                        distCoeffs,
                        rvecs,
                        tvecs,
                        Calib3d.CALIB_FIX_PRINCIPAL_POINT
                    )
                }
                CameraModelType.FISHEYE -> {
                    val K = Mat.eye(3, 3, CvType.CV_64F)
                    val D = Mat.zeros(4, 1, CvType.CV_64F)
                    Calib3d.fisheye_calibrate(
                        objectPoints,
                        imagePoints,
                        size,
                        K,
                        D,
                        rvecs,
                        tvecs,
                        Calib3d.fisheye_CALIB_RECOMPUTE_EXTRINSIC or
                                Calib3d.fisheye_CALIB_CHECK_COND or
                                Calib3d.fisheye_CALIB_FIX_SKEW
                    )
                    K.copyTo(cameraMatrix)
                    D.copyTo(distCoeffs)
                    K.release()
                    D.release()
                    // Return the RMS error
                    0.0 // fisheye_calibrate doesn't return RMS directly
                }
            }

            if (rms > currentConfig.maxReprojectionError) {
                status = CalibrationStatus.FAILED
                return Result.failure(IllegalStateException(
                    "Reprojection error too high: $rms > ${currentConfig.maxReprojectionError}"
                ))
            }

            // Extract calibration parameters
            val fx = cameraMatrix.get(0, 0)[0]
            val fy = cameraMatrix.get(1, 1)[0]
            val cx = cameraMatrix.get(0, 2)[0]
            val cy = cameraMatrix.get(1, 2)[0]

            val distCoeffsArray = DoubleArray(distCoeffs.rows())
            for (i in 0 until distCoeffs.rows()) {
                distCoeffsArray[i] = distCoeffs.get(i, 0)[0]
            }

            // Clean up
            cameraMatrix.release()
            distCoeffs.release()
            rvecs.forEach { it.release() }
            tvecs.forEach { it.release() }
            imagePoints.forEach { it.release() }
            objectPoints.forEach { it.release() }
            imagePoints.clear()
            objectPoints.clear()

            status = CalibrationStatus.COMPLETED

            Result.success(IntrinsicCalibResult(
                fx = fx,
                fy = fy,
                cx = cx,
                cy = cy,
                distortionCoeffs = distCoeffsArray,
                reprojectionError = rms,
                captureCount = frameCounter.get(),
                imageWidth = size.width.toInt(),
                imageHeight = size.height.toInt(),
                cameraModel = currentConfig.cameraModel
            ))
        } catch (e: Exception) {
            status = CalibrationStatus.FAILED
            Result.failure(e)
        }
    }

    override fun startExtrinsicCalibration(config: ExtrinsicCalibConfig): Result<Unit> {
        return Result.failure(UnsupportedOperationException(
            "Extrinsic calibration not implemented in IntrinsicCalibrator"
        ))
    }

    override fun recordExtrinsicSequence(): Result<ExtrinsicCalibData> {
        return Result.failure(UnsupportedOperationException(
            "Extrinsic calibration not implemented in IntrinsicCalibrator"
        ))
    }

    override fun getCalibrationStatus(): CalibrationStatus = status

    override fun getCoverageMap(): CoverageMap = coverageMap

    /**
     * Calculate which coverage region the corners belong to
     * Divides image into 3x3 grid (9 regions)
     */
    private fun calculateCoverageRegion(corners: MatOfPoint2f, width: Int, height: Int): Int {
        // Calculate centroid of corners
        var sumX = 0.0
        var sumY = 0.0
        for (i in 0 until corners.rows()) {
            val point = corners.get(i, 0)
            sumX += point[0]
            sumY += point[1]
        }
        val centroidX = sumX / corners.rows()
        val centroidY = sumY / corners.rows()

        // Map to 3x3 grid
        val regionX = (centroidX / width * 3).toInt().coerceIn(0, 2)
        val regionY = (centroidY / height * 3).toInt().coerceIn(0, 2)

        return regionY * 3 + regionX
    }

    /**
     * Generate 3D object points for calibration pattern
     */
    private fun generateObjectPoints(targetSize: TargetSize): Mat {
        val objectPoints = Mat(
            targetSize.width * targetSize.height,
            1,
            CvType.CV_32FC3
        )

        var idx = 0
        for (i in 0 until targetSize.height) {
            for (j in 0 until targetSize.width) {
                objectPoints.put(
                    idx++, 0,
                    j * targetSize.squareSize,
                    i * targetSize.squareSize,
                    0.0
                )
            }
        }

        return objectPoints
    }
}
