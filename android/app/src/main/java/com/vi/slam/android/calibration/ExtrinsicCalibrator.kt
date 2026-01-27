package com.vi.slam.android.calibration

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicInteger

/**
 * Camera-IMU extrinsic calibration data collection
 *
 * Collects synchronized camera-IMU measurements for offline calibration
 * using Kalibr or similar tools. This class handles:
 * - Checkerboard corner detection in camera frames
 * - Synchronization of camera and IMU timestamps
 * - Data packaging for export
 *
 * The actual hand-eye calibration (R_cam_imu, t_cam_imu estimation)
 * is performed offline using collected data.
 */
class ExtrinsicCalibrator {

    private var config: ExtrinsicCalibConfig? = null
    private var status: CalibrationStatus = CalibrationStatus.IDLE
    private var startTime: Long = 0L
    private val frameCounter = AtomicInteger(0)
    private val imuSampleCounter = AtomicInteger(0)

    // Calibration data storage
    private val cameraFrames = mutableListOf<CameraFrame>()
    private val imuSamples = mutableListOf<ImuSample>()

    /**
     * Camera frame with detected corners
     */
    data class CameraFrame(
        val frameId: Int,
        val timestamp: Long,
        val imageData: ByteArray,
        val width: Int,
        val height: Int,
        val cornersDetected: Boolean,
        val cornerPoints: MatOfPoint2f? = null
    ) {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (javaClass != other?.javaClass) return false

            other as CameraFrame

            if (frameId != other.frameId) return false
            if (timestamp != other.timestamp) return false
            if (!imageData.contentEquals(other.imageData)) return false
            if (width != other.width) return false
            if (height != other.height) return false
            if (cornersDetected != other.cornersDetected) return false

            return true
        }

        override fun hashCode(): Int {
            var result = frameId
            result = 31 * result + timestamp.hashCode()
            result = 31 * result + imageData.contentHashCode()
            result = 31 * result + width
            result = 31 * result + height
            result = 31 * result + cornersDetected.hashCode()
            return result
        }
    }

    /**
     * IMU measurement sample
     */
    data class ImuSample(
        val timestamp: Long,
        val accelX: Double,
        val accelY: Double,
        val accelZ: Double,
        val gyroX: Double,
        val gyroY: Double,
        val gyroZ: Double
    )

    /**
     * Start extrinsic calibration data collection
     *
     * @param config Calibration configuration
     * @return Success or failure result
     */
    fun startExtrinsicCalibration(config: ExtrinsicCalibConfig): Result<Unit> {
        return try {
            if (status != CalibrationStatus.IDLE &&
                status != CalibrationStatus.COMPLETED &&
                status != CalibrationStatus.FAILED
            ) {
                return Result.failure(IllegalStateException("Calibration already in progress"))
            }

            this.config = config
            this.status = CalibrationStatus.CAPTURING
            this.startTime = System.currentTimeMillis()
            this.cameraFrames.clear()
            this.imuSamples.clear()
            this.frameCounter.set(0)
            this.imuSampleCounter.set(0)

            Result.success(Unit)
        } catch (e: Exception) {
            status = CalibrationStatus.FAILED
            Result.failure(e)
        }
    }

    /**
     * Capture camera frame with corner detection
     *
     * @param imageData Raw grayscale image data
     * @param width Image width
     * @param height Image height
     * @param timestamp Frame timestamp in nanoseconds
     * @return Capture result with corner detection status
     */
    fun captureCameraFrame(
        imageData: ByteArray,
        width: Int,
        height: Int,
        timestamp: Long
    ): Result<CameraFrame> {
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
                else -> false // Only checkerboard supported for now
            }

            // Refine corners if detected
            if (found) {
                val termCriteria = TermCriteria(
                    TermCriteria.EPS + TermCriteria.MAX_ITER,
                    30,
                    0.001
                )
                Imgproc.cornerSubPix(
                    mat,
                    corners,
                    Size(11.0, 11.0),
                    Size(-1.0, -1.0),
                    termCriteria
                )
            }

            val frameId = frameCounter.incrementAndGet()
            val frame = CameraFrame(
                frameId = frameId,
                timestamp = timestamp,
                imageData = imageData.clone(),
                width = width,
                height = height,
                cornersDetected = found,
                cornerPoints = if (found) corners.clone() as MatOfPoint2f? else null
            )

            // Store frame if corners detected
            if (found) {
                cameraFrames.add(frame)
            }

            mat.release()
            Result.success(frame)
        } catch (e: Exception) {
            status = CalibrationStatus.FAILED
            Result.failure(e)
        }
    }

    /**
     * Add IMU sample
     *
     * @param timestamp Sample timestamp in nanoseconds
     * @param accelX Acceleration X (m/s²)
     * @param accelY Acceleration Y (m/s²)
     * @param accelZ Acceleration Z (m/s²)
     * @param gyroX Gyroscope X (rad/s)
     * @param gyroY Gyroscope Y (rad/s)
     * @param gyroZ Gyroscope Z (rad/s)
     * @return Success or failure result
     */
    fun addImuSample(
        timestamp: Long,
        accelX: Double,
        accelY: Double,
        accelZ: Double,
        gyroX: Double,
        gyroY: Double,
        gyroZ: Double
    ): Result<Unit> {
        if (status != CalibrationStatus.CAPTURING) {
            return Result.failure(IllegalStateException("Not in capturing state"))
        }

        return try {
            val sample = ImuSample(
                timestamp = timestamp,
                accelX = accelX,
                accelY = accelY,
                accelZ = accelZ,
                gyroX = gyroX,
                gyroY = gyroY,
                gyroZ = gyroZ
            )
            imuSamples.add(sample)
            imuSampleCounter.incrementAndGet()
            Result.success(Unit)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Check if recording duration is complete
     */
    fun isRecordingComplete(): Boolean {
        val currentConfig = config ?: return false
        val elapsedTime = System.currentTimeMillis() - startTime
        return elapsedTime >= currentConfig.recordingDuration
    }

    /**
     * Get current recording progress (0.0 to 1.0)
     */
    fun getRecordingProgress(): Float {
        val currentConfig = config ?: return 0f
        val elapsedTime = System.currentTimeMillis() - startTime
        return (elapsedTime.toFloat() / currentConfig.recordingDuration).coerceIn(0f, 1f)
    }

    /**
     * Get recording statistics
     */
    fun getRecordingStats(): RecordingStats {
        return RecordingStats(
            frameCount = cameraFrames.size,
            imuSampleCount = imuSamples.size,
            duration = if (status == CalibrationStatus.CAPTURING) {
                System.currentTimeMillis() - startTime
            } else 0L,
            framesWithCorners = cameraFrames.count { it.cornersDetected }
        )
    }

    data class RecordingStats(
        val frameCount: Int,
        val imuSampleCount: Int,
        val duration: Long,
        val framesWithCorners: Int
    )

    /**
     * Finish extrinsic calibration and prepare data for export
     *
     * @return Calibration data package for offline processing
     */
    fun finishExtrinsicCalibration(): Result<ExtrinsicCalibData> {
        val currentConfig = config
            ?: return Result.failure(IllegalStateException("Calibration not started"))

        if (status != CalibrationStatus.CAPTURING) {
            return Result.failure(IllegalStateException("Not in capturing state"))
        }

        return try {
            // Validate minimum data requirements
            if (cameraFrames.isEmpty()) {
                return Result.failure(
                    IllegalStateException("No camera frames with detected corners")
                )
            }

            if (imuSamples.isEmpty()) {
                return Result.failure(
                    IllegalStateException("No IMU samples collected")
                )
            }

            status = CalibrationStatus.COMPLETED
            val duration = System.currentTimeMillis() - startTime

            val recordingId = "extrinsic_${System.currentTimeMillis()}"
            val calibData = ExtrinsicCalibData(
                recordingId = recordingId,
                duration = duration,
                frameCount = cameraFrames.size,
                imuSampleCount = imuSamples.size,
                intrinsicParams = currentConfig.intrinsicParams
            )

            Result.success(calibData)
        } catch (e: Exception) {
            status = CalibrationStatus.FAILED
            Result.failure(e)
        }
    }

    /**
     * Export collected data for Kalibr processing
     *
     * Format compatible with Kalibr's expected input:
     * - Camera images with timestamps
     * - IMU measurements CSV
     * - Camera intrinsic parameters (from config)
     *
     * @return Map of file paths to data
     */
    fun exportCalibrationData(): Result<Map<String, Any>> {
        if (status != CalibrationStatus.COMPLETED) {
            return Result.failure(
                IllegalStateException("Calibration not completed")
            )
        }

        return try {
            val currentConfig = config
                ?: return Result.failure(IllegalStateException("Configuration not available"))

            val data: Map<String, Any> = mapOf(
                "camera_frames" to cameraFrames,
                "imu_samples" to imuSamples,
                "intrinsic_params" to currentConfig.intrinsicParams
            )
            Result.success(data)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Cancel ongoing calibration
     */
    fun cancelCalibration() {
        status = CalibrationStatus.IDLE
        cameraFrames.clear()
        imuSamples.clear()
        frameCounter.set(0)
        imuSampleCounter.set(0)
    }

    /**
     * Get current calibration status
     */
    fun getStatus(): CalibrationStatus = status
}
