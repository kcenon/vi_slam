package com.vi.slam.android.recorder

import android.util.Log
import com.vi.slam.android.sensor.SynchronizedData
import java.io.File
import java.util.UUID
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference

/**
 * Local implementation of IRecorder for recording synchronized camera and IMU data to local storage.
 *
 * This class manages the lifecycle of recording sessions, handles data persistence,
 * and maintains recording statistics. It implements thread-safe state management
 * to ensure safe operation from multiple threads.
 *
 * Thread Safety: This class is thread-safe and can be accessed from multiple threads.
 * All state transitions and data recording operations are synchronized.
 *
 * Lifecycle:
 * ```
 * UNINITIALIZED -> initialize() -> IDLE
 * IDLE -> startRecording() -> RECORDING
 * RECORDING -> stopRecording() -> IDLE
 * ANY -> ERROR (on fatal error)
 * ```
 *
 * Usage Example:
 * ```kotlin
 * val recorder = LocalRecorder()
 * val config = RecorderConfig(
 *     outputDirectory = File("/path/to/output"),
 *     maxDurationMs = 60 * 60 * 1000
 * )
 *
 * recorder.initialize(config).onSuccess {
 *     recorder.startRecording().onSuccess { info ->
 *         println("Recording started: ${info.recordingId}")
 *         // Data flows via onData()...
 *         recorder.stopRecording().onSuccess { summary ->
 *             println("Recorded ${summary.frameCount} frames")
 *         }
 *     }
 * }
 * ```
 */
class LocalRecorder : IRecorder {

    companion object {
        private const val TAG = "LocalRecorder"
        private const val SESSION_FOLDER_PREFIX = "recording_"
    }

    // Current recorder state (thread-safe atomic reference)
    private val state = AtomicReference<RecorderState>(RecorderState.UNINITIALIZED)

    // Configuration
    private var config: RecorderConfig? = null

    // Current recording session info
    private var currentRecording: RecordingInfo? = null
    private var recordingStartTime: Long = 0

    // Session output directory
    private var sessionDirectory: File? = null

    // Statistics tracking - using atomic counters for thread-safe updates
    private val frameCount = AtomicLong(0)
    private val imuSampleCount = AtomicLong(0)

    // Lock for state transitions
    private val stateLock = Any()

    override fun initialize(config: RecorderConfig): Result<Unit> {
        return try {
            Log.d(TAG, "Initializing LocalRecorder")

            synchronized(stateLock) {
                val currentState = state.get()

                // Allow re-initialization from IDLE, ERROR, or UNINITIALIZED states
                if (currentState == RecorderState.RECORDING ||
                    currentState == RecorderState.STARTING ||
                    currentState == RecorderState.STOPPING
                ) {
                    return Result.failure(
                        IllegalStateException(
                            "Cannot initialize while in $currentState state. " +
                                    "Stop recording first."
                        )
                    )
                }

                // Validate configuration
                validateConfig(config).onFailure { error ->
                    return Result.failure(error)
                }

                // Store configuration
                this.config = config

                // Reset state
                currentRecording = null
                sessionDirectory = null
                frameCount.set(0)
                imuSampleCount.set(0)

                // Transition to IDLE
                state.set(RecorderState.IDLE)

                Log.i(TAG, "LocalRecorder initialized successfully")
                Result.success(Unit)
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize LocalRecorder", e)
            state.set(RecorderState.ERROR)
            Result.failure(e)
        }
    }

    override fun startRecording(): Result<RecordingInfo> {
        return try {
            Log.d(TAG, "Starting recording session")

            synchronized(stateLock) {
                val currentState = state.get()

                // Check if we can start recording
                if (currentState != RecorderState.IDLE) {
                    return Result.failure(
                        IllegalStateException(
                            "Cannot start recording from $currentState state. " +
                                    "Recorder must be in IDLE state."
                        )
                    )
                }

                // Check if initialized
                val currentConfig = config
                    ?: return Result.failure(
                        IllegalStateException("Recorder not initialized. Call initialize() first.")
                    )

                // Transition to STARTING
                state.set(RecorderState.STARTING)

                try {
                    // Generate unique recording ID
                    val recordingId = generateRecordingId()

                    // Create session directory
                    val sessionDir = File(
                        currentConfig.outputDirectory,
                        "$SESSION_FOLDER_PREFIX$recordingId"
                    )

                    if (!sessionDir.exists() && !sessionDir.mkdirs()) {
                        state.set(RecorderState.ERROR)
                        return Result.failure(
                            IllegalStateException(
                                "Failed to create session directory: ${sessionDir.absolutePath}"
                            )
                        )
                    }

                    sessionDirectory = sessionDir

                    // Create recording info
                    recordingStartTime = System.currentTimeMillis()
                    val recordingInfo = RecordingInfo(
                        recordingId = recordingId,
                        startTime = recordingStartTime,
                        outputPath = sessionDir.absolutePath,
                        config = currentConfig
                    )
                    currentRecording = recordingInfo

                    // Reset statistics
                    frameCount.set(0)
                    imuSampleCount.set(0)

                    // Transition to RECORDING
                    state.set(RecorderState.RECORDING)

                    Log.i(TAG, "Recording started: $recordingId at ${sessionDir.absolutePath}")
                    Result.success(recordingInfo)
                } catch (e: Exception) {
                    Log.e(TAG, "Failed during recording start", e)
                    state.set(RecorderState.ERROR)
                    Result.failure(e)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start recording", e)
            state.set(RecorderState.ERROR)
            Result.failure(e)
        }
    }

    override fun stopRecording(): Result<RecordingSummary> {
        return try {
            Log.d(TAG, "Stopping recording session")

            synchronized(stateLock) {
                val currentState = state.get()

                // Check if we can stop recording
                if (currentState != RecorderState.RECORDING) {
                    return Result.failure(
                        IllegalStateException(
                            "Cannot stop recording from $currentState state. " +
                                    "Recorder must be in RECORDING state."
                        )
                    )
                }

                // Transition to STOPPING
                state.set(RecorderState.STOPPING)

                try {
                    val recording = currentRecording
                        ?: return Result.failure(
                            IllegalStateException("No active recording session")
                        )

                    val sessionDir = sessionDirectory
                        ?: return Result.failure(
                            IllegalStateException("Session directory not found")
                        )

                    // Calculate recording duration
                    val durationMs = System.currentTimeMillis() - recordingStartTime

                    // Get final statistics
                    val finalFrameCount = frameCount.get()
                    val finalImuSampleCount = imuSampleCount.get()

                    // TODO: Finalize video file (will be implemented in video encoding task)
                    // TODO: Close IMU CSV writer (will be implemented in IMU writer task)
                    // TODO: Generate metadata.json (will be implemented in metadata task)

                    // Collect output files (will be populated as features are implemented)
                    val outputFiles = mutableListOf<String>()

                    // Create recording summary
                    val summary = RecordingSummary(
                        recordingId = recording.recordingId,
                        success = true,
                        frameCount = finalFrameCount,
                        imuSampleCount = finalImuSampleCount,
                        durationMs = durationMs,
                        outputFiles = outputFiles,
                        videoFile = null,  // TODO: Set after video encoding is implemented
                        imuFile = null,    // TODO: Set after IMU writer is implemented
                        metadataFile = null,  // TODO: Set after metadata generation is implemented
                        errorMessage = null
                    )

                    // Clean up session state
                    currentRecording = null
                    sessionDirectory = null

                    // Transition back to IDLE
                    state.set(RecorderState.IDLE)

                    Log.i(
                        TAG,
                        "Recording stopped: ${recording.recordingId}, " +
                                "Duration: ${durationMs}ms, " +
                                "Frames: $finalFrameCount, " +
                                "IMU samples: $finalImuSampleCount"
                    )
                    Result.success(summary)
                } catch (e: Exception) {
                    Log.e(TAG, "Failed during recording stop", e)
                    state.set(RecorderState.ERROR)
                    Result.failure(e)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to stop recording", e)
            state.set(RecorderState.ERROR)
            Result.failure(e)
        }
    }

    override fun onData(data: SynchronizedData) {
        // Check if we're currently recording
        if (!isEnabled()) {
            return
        }

        try {
            // TODO: Encode video frame to H.264 (will be implemented in video encoding task)
            // For now, just count frames
            frameCount.incrementAndGet()

            // TODO: Write IMU samples to CSV (will be implemented in IMU writer task)
            // For now, just count IMU samples
            val totalImuSamples = data.imuSamplesBefore.size + data.imuSamplesAfter.size
            imuSampleCount.addAndGet(totalImuSamples.toLong())

            // Log periodically for debugging (every 100 frames)
            val currentFrameCount = frameCount.get()
            if (currentFrameCount % 100 == 0L) {
                Log.d(
                    TAG,
                    "Recording progress: $currentFrameCount frames, " +
                            "${imuSampleCount.get()} IMU samples"
                )
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error processing data", e)
            // Don't transition to ERROR state for individual data processing errors
            // This allows recording to continue even if some frames fail
        }
    }

    override fun isEnabled(): Boolean {
        return state.get() == RecorderState.RECORDING
    }

    /**
     * Validate recorder configuration.
     *
     * Checks that all configuration parameters are valid and within acceptable ranges.
     *
     * @param config Configuration to validate
     * @return Success or failure with validation error
     */
    private fun validateConfig(config: RecorderConfig): Result<Unit> {
        return try {
            // Output directory validation
            if (!config.outputDirectory.exists()) {
                return Result.failure(
                    IllegalArgumentException(
                        "Output directory does not exist: ${config.outputDirectory.absolutePath}"
                    )
                )
            }

            if (!config.outputDirectory.isDirectory) {
                return Result.failure(
                    IllegalArgumentException(
                        "Output path is not a directory: ${config.outputDirectory.absolutePath}"
                    )
                )
            }

            if (!config.outputDirectory.canWrite()) {
                return Result.failure(
                    IllegalArgumentException(
                        "Output directory is not writable: ${config.outputDirectory.absolutePath}"
                    )
                )
            }

            // Note: RecorderConfig's init block already validates:
            // - maxDurationMs > 0
            // - video dimensions > 0
            // - videoFps in (0, 120]
            // - videoBitrate > 0

            Result.success(Unit)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Generate a unique recording ID.
     *
     * Format: <timestamp>_<uuid>
     * Example: 20250126_143052_a1b2c3d4
     *
     * @return Unique recording identifier
     */
    private fun generateRecordingId(): String {
        val timestamp = java.text.SimpleDateFormat(
            "yyyyMMdd_HHmmss",
            java.util.Locale.US
        ).format(java.util.Date())

        val uuid = UUID.randomUUID().toString().substring(0, 8)

        return "${timestamp}_$uuid"
    }
}
