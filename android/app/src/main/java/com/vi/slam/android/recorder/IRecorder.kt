package com.vi.slam.android.recorder

import com.vi.slam.android.data.IDataDestination
import com.vi.slam.android.sensor.SynchronizedData
import java.io.File

/**
 * Interface for recording synchronized camera and IMU data to local storage.
 *
 * IRecorder extends IDataDestination to receive data from the Data Manager and
 * saves it in formats suitable for SLAM processing and post-processing analysis.
 *
 * Supported output formats:
 * - Video: H.264 encoded MP4 files
 * - IMU: CSV files with nanosecond-precision timestamps
 * - Metadata: JSON files with session info, calibration, and device metadata
 *
 * Thread Safety: Implementations must be thread-safe as onData() may be called
 * from high-frequency camera callback threads.
 *
 * Lifecycle:
 * ```
 * recorder.initialize(config)
 * recorder.startRecording()    // Begin recording
 * ... data flows via onData() ...
 * recorder.stopRecording()     // End recording, finalize files
 * ```
 *
 * Usage Example:
 * ```kotlin
 * val config = RecorderConfig(
 *     outputDirectory = File("/path/to/output"),
 *     videoFormat = VideoFormat.H264_MP4,
 *     imuFormat = ImuFormat.CSV,
 *     maxDurationMs = 60 * 60 * 1000  // 60 minutes
 * )
 *
 * recorder.initialize(config).onSuccess {
 *     recorder.startRecording().onSuccess { info ->
 *         println("Recording started: ${info.recordingId}")
 *     }
 * }
 *
 * // Later...
 * recorder.stopRecording().onSuccess { summary ->
 *     println("Recorded ${summary.frameCount} frames")
 *     println("Output files: ${summary.outputFiles}")
 * }
 * ```
 */
interface IRecorder : IDataDestination {
    /**
     * Initialize the recorder with configuration.
     *
     * Prepares the recorder for recording sessions. Must be called before startRecording().
     * Validates configuration, checks output directory permissions, and prepares
     * necessary resources (codecs, file writers, etc.).
     *
     * @param config Recorder configuration
     * @return Success or failure with error details
     */
    fun initialize(config: RecorderConfig): Result<Unit>

    /**
     * Start a new recording session.
     *
     * Creates output directory, opens video encoder and IMU file writer,
     * and begins accepting data via onData().
     *
     * Prerequisites:
     * - initialize() must have been called successfully
     * - No recording session currently active
     *
     * @return RecordingInfo with session metadata, or failure if cannot start
     */
    fun startRecording(): Result<RecordingInfo>

    /**
     * Stop the current recording session.
     *
     * Finalizes video file, closes IMU writer, generates metadata JSON,
     * and returns summary statistics.
     *
     * This method ensures all buffered data is flushed to disk and files
     * are properly closed. After this call, the recorder returns to idle
     * state and can start a new recording session.
     *
     * @return RecordingSummary with statistics and output file paths,
     *         or failure if stop operation fails
     */
    fun stopRecording(): Result<RecordingSummary>

    /**
     * Handle incoming synchronized data.
     *
     * Called by Data Manager for each synchronized frame. Implementations should:
     * 1. Encode video frame to H.264
     * 2. Write IMU samples to CSV
     * 3. Update internal statistics (frame count, etc.)
     *
     * This method must be fast (<5ms) to avoid blocking the data pipeline.
     * Consider using a background thread pool for encoding if needed.
     *
     * Thread Safety: This method may be called from multiple threads concurrently.
     *
     * @param data Synchronized camera frame and IMU data
     */
    override fun onData(data: SynchronizedData)

    /**
     * Check if recorder is currently enabled and accepting data.
     *
     * Returns true only when:
     * - Recorder is initialized
     * - Recording session is active
     * - No fatal errors occurred
     *
     * @return True if recorder can accept data, false otherwise
     */
    override fun isEnabled(): Boolean
}
