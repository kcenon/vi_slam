package com.vi.slam.android.calibration

/**
 * Camera calibration data models and configurations
 */

/**
 * Type of calibration target pattern
 */
enum class CalibrationTargetType {
    CHECKERBOARD,
    APRILGRID,
    CIRCLES_GRID
}

/**
 * Camera model type
 */
enum class CameraModelType {
    PINHOLE,
    FISHEYE
}

/**
 * Size of calibration target
 */
data class TargetSize(
    val width: Int,  // Number of inner corners (width)
    val height: Int, // Number of inner corners (height)
    val squareSize: Double // Size of each square in meters
)

/**
 * Configuration for intrinsic calibration
 */
data class IntrinsicCalibConfig(
    val targetType: CalibrationTargetType = CalibrationTargetType.CHECKERBOARD,
    val targetSize: TargetSize,
    val cameraModel: CameraModelType = CameraModelType.PINHOLE,
    val minCaptures: Int = 20,
    val coverageRegions: Int = 9, // 3x3 grid
    val maxReprojectionError: Double = 0.5 // pixels
)

/**
 * Result of intrinsic calibration
 */
data class IntrinsicCalibResult(
    val fx: Double,              // Focal length x
    val fy: Double,              // Focal length y
    val cx: Double,              // Principal point x
    val cy: Double,              // Principal point y
    val distortionCoeffs: DoubleArray, // Distortion coefficients
    val reprojectionError: Double,     // RMS reprojection error
    val captureCount: Int,              // Number of captures used
    val imageWidth: Int,                // Calibrated image width
    val imageHeight: Int,               // Calibrated image height
    val cameraModel: CameraModelType    // Camera model type
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as IntrinsicCalibResult

        if (fx != other.fx) return false
        if (fy != other.fy) return false
        if (cx != other.cx) return false
        if (cy != other.cy) return false
        if (!distortionCoeffs.contentEquals(other.distortionCoeffs)) return false
        if (reprojectionError != other.reprojectionError) return false
        if (captureCount != other.captureCount) return false
        if (imageWidth != other.imageWidth) return false
        if (imageHeight != other.imageHeight) return false
        if (cameraModel != other.cameraModel) return false

        return true
    }

    override fun hashCode(): Int {
        var result = fx.hashCode()
        result = 31 * result + fy.hashCode()
        result = 31 * result + cx.hashCode()
        result = 31 * result + cy.hashCode()
        result = 31 * result + distortionCoeffs.contentHashCode()
        result = 31 * result + reprojectionError.hashCode()
        result = 31 * result + captureCount
        result = 31 * result + imageWidth
        result = 31 * result + imageHeight
        result = 31 * result + cameraModel.hashCode()
        return result
    }
}

/**
 * Single captured frame for calibration
 */
data class CalibrationCapture(
    val frameId: Int,
    val timestamp: Long,
    val imageData: ByteArray,
    val width: Int,
    val height: Int,
    val cornersDetected: Boolean,
    val cornerPoints: List<Pair<Float, Float>>? = null,
    val coverageRegion: Int? = null // Which coverage region this belongs to (0-8 for 3x3)
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as CalibrationCapture

        if (frameId != other.frameId) return false
        if (timestamp != other.timestamp) return false
        if (!imageData.contentEquals(other.imageData)) return false
        if (width != other.width) return false
        if (height != other.height) return false
        if (cornersDetected != other.cornersDetected) return false
        if (cornerPoints != other.cornerPoints) return false
        if (coverageRegion != other.coverageRegion) return false

        return true
    }

    override fun hashCode(): Int {
        var result = frameId
        result = 31 * result + timestamp.hashCode()
        result = 31 * result + imageData.contentHashCode()
        result = 31 * result + width
        result = 31 * result + height
        result = 31 * result + cornersDetected.hashCode()
        result = 31 * result + (cornerPoints?.hashCode() ?: 0)
        result = 31 * result + (coverageRegion ?: 0)
        return result
    }
}

/**
 * Coverage map for calibration progress tracking
 */
data class CoverageMap(
    val regions: Array<Boolean>, // 3x3 grid = 9 regions
    val requiredRegions: Int = 9,
    val capturesPerRegion: IntArray = IntArray(9)
) {
    val coveredRegions: Int
        get() = regions.count { it }

    val progress: Float
        get() = coveredRegions.toFloat() / requiredRegions

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as CoverageMap

        if (!regions.contentEquals(other.regions)) return false
        if (requiredRegions != other.requiredRegions) return false
        if (!capturesPerRegion.contentEquals(other.capturesPerRegion)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = regions.contentHashCode()
        result = 31 * result + requiredRegions
        result = 31 * result + capturesPerRegion.contentHashCode()
        return result
    }
}

/**
 * Calibration status
 */
enum class CalibrationStatus {
    IDLE,
    CAPTURING,
    PROCESSING,
    COMPLETED,
    FAILED
}

/**
 * Configuration for extrinsic calibration
 */
data class ExtrinsicCalibConfig(
    val targetType: CalibrationTargetType = CalibrationTargetType.CHECKERBOARD,
    val targetSize: TargetSize,
    val recordingDuration: Long = 60000, // milliseconds
    val intrinsicParams: IntrinsicCalibResult // Require intrinsic calibration first
)

/**
 * Extrinsic calibration data (for PC-side processing)
 */
data class ExtrinsicCalibData(
    val recordingId: String,
    val duration: Long,
    val frameCount: Int,
    val imuSampleCount: Int,
    val intrinsicParams: IntrinsicCalibResult
)

/**
 * Result of camera-IMU extrinsic calibration
 *
 * Contains the 6-DOF transformation (rotation and translation)
 * from IMU coordinate frame to camera coordinate frame.
 */
data class ExtrinsicCalibResult(
    val rotationMatrix: Array<DoubleArray>, // 3x3 rotation matrix (R_cam_imu)
    val translation: DoubleArray,           // 3x1 translation vector (t_cam_imu) in meters
    val reprojectionError: Double,          // RMS reprojection error in pixels
    val frameCount: Int,                    // Number of frames used for calibration
    val timestamp: Long                     // Calibration timestamp
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as ExtrinsicCalibResult

        if (!rotationMatrix.contentDeepEquals(other.rotationMatrix)) return false
        if (!translation.contentEquals(other.translation)) return false
        if (reprojectionError != other.reprojectionError) return false
        if (frameCount != other.frameCount) return false
        if (timestamp != other.timestamp) return false

        return true
    }

    override fun hashCode(): Int {
        var result = rotationMatrix.contentDeepHashCode()
        result = 31 * result + translation.contentHashCode()
        result = 31 * result + reprojectionError.hashCode()
        result = 31 * result + frameCount
        result = 31 * result + timestamp.hashCode()
        return result
    }
}
