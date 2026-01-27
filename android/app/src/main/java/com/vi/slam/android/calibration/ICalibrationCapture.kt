package com.vi.slam.android.calibration

/**
 * Interface for camera calibration capture operations
 * Defines operations for both intrinsic and extrinsic calibration
 */
interface ICalibrationCapture {
    /**
     * Start intrinsic calibration session
     * @param config Calibration configuration
     * @return Success or error
     */
    fun startIntrinsicCalibration(config: IntrinsicCalibConfig): Result<Unit>

    /**
     * Capture a single calibration image
     * @return Captured frame with corner detection results
     */
    fun captureCalibrationImage(): Result<CalibrationCapture>

    /**
     * Finish intrinsic calibration and compute camera parameters
     * @return Computed intrinsic parameters
     */
    fun finishIntrinsicCalibration(): Result<IntrinsicCalibResult>

    /**
     * Start extrinsic calibration session
     * @param config Extrinsic calibration configuration
     * @return Success or error
     */
    fun startExtrinsicCalibration(config: ExtrinsicCalibConfig): Result<Unit>

    /**
     * Record extrinsic calibration sequence (camera + IMU)
     * @return Recorded calibration data
     */
    fun recordExtrinsicSequence(): Result<ExtrinsicCalibData>

    /**
     * Get current calibration status
     * @return Current status
     */
    fun getCalibrationStatus(): CalibrationStatus

    /**
     * Get coverage map for calibration progress
     * @return Coverage map showing which regions are covered
     */
    fun getCoverageMap(): CoverageMap
}
