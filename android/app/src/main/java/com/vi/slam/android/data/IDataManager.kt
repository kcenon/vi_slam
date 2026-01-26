package com.vi.slam.android.data

/**
 * Session mode for data management.
 *
 * @property RECORD_ONLY Only record data to local storage
 * @property STREAM_ONLY Only stream data to remote server
 * @property RECORD_AND_STREAM Both record and stream simultaneously
 */
enum class SessionMode {
    RECORD_ONLY,
    STREAM_ONLY,
    RECORD_AND_STREAM
}

/**
 * Session status enumeration.
 *
 * @property IDLE No active session
 * @property STARTING Session is being initialized
 * @property ACTIVE Session is running
 * @property STOPPING Session is being terminated
 * @property ERROR Session encountered an error
 */
enum class SessionStatus {
    IDLE,
    STARTING,
    ACTIVE,
    STOPPING,
    ERROR
}

/**
 * Session information.
 *
 * @property sessionId Unique identifier for the session
 * @property startTime Session start timestamp in milliseconds
 * @property mode Session operating mode
 * @property outputDirectory Output directory for recorded data (null for STREAM_ONLY)
 */
data class SessionInfo(
    val sessionId: String,
    val startTime: Long,
    val mode: SessionMode,
    val outputDirectory: String?
)

/**
 * Session statistics for monitoring.
 *
 * @property frameCount Total number of frames processed
 * @property imuSampleCount Total number of IMU samples processed
 * @property durationMs Session duration in milliseconds
 * @property frameDropCount Number of dropped frames
 * @property averageFps Average frames per second
 * @property averageImuRate Average IMU sampling rate in Hz
 */
data class SessionStatistics(
    val frameCount: Long = 0,
    val imuSampleCount: Long = 0,
    val durationMs: Long = 0,
    val frameDropCount: Long = 0,
    val averageFps: Float = 0f,
    val averageImuRate: Float = 0f
)

/**
 * Session summary at termination.
 *
 * @property sessionId Session identifier
 * @property mode Session mode that was used
 * @property statistics Final statistics
 * @property outputDirectory Output directory (null for STREAM_ONLY)
 * @property success True if session completed successfully
 * @property errorMessage Error message if session failed
 */
data class SessionSummary(
    val sessionId: String,
    val mode: SessionMode,
    val statistics: SessionStatistics,
    val outputDirectory: String?,
    val success: Boolean,
    val errorMessage: String? = null
)

/**
 * Interface for data management.
 *
 * This interface defines the contract for managing data flow between
 * capture sources (camera, IMU), storage (recorder), and streaming.
 * It coordinates session lifecycle and routes synchronized data to
 * appropriate consumers.
 */
interface IDataManager {
    /**
     * Initialize the Data Manager.
     *
     * Sets up internal components and prepares for session management.
     * Must be called before starting any session.
     *
     * @return Result.success if initialization succeeded, Result.failure with exception otherwise
     */
    fun initialize(): Result<Unit>

    /**
     * Start a data collection session.
     *
     * Begins capturing camera and IMU data, and routes the synchronized
     * data according to the specified mode (record, stream, or both).
     *
     * @param mode Session mode determining data flow
     * @return Result.success with SessionInfo if started, Result.failure with exception otherwise
     * @throws IllegalStateException if already in an active session
     */
    fun startSession(mode: SessionMode): Result<SessionInfo>

    /**
     * Stop the current session.
     *
     * Stops data capture, flushes buffers, and finalizes recording/streaming.
     * Returns summary statistics for the completed session.
     *
     * @return Result.success with SessionSummary if stopped, Result.failure with exception otherwise
     * @throws IllegalStateException if no active session
     */
    fun stopSession(): Result<SessionSummary>

    /**
     * Get current session status.
     *
     * @return Current session status
     */
    fun getSessionStatus(): SessionStatus

    /**
     * Get current session statistics.
     *
     * Returns real-time statistics for the active session.
     * Returns zero values if no session is active.
     *
     * @return Current session statistics
     */
    fun getStatistics(): SessionStatistics

    /**
     * Enable or disable the recorder.
     *
     * Can be called during an active session to dynamically control recording.
     * Only effective if session mode includes recording.
     *
     * @param enabled True to enable recorder, false to disable
     */
    fun setRecorderEnabled(enabled: Boolean)

    /**
     * Enable or disable the streamer.
     *
     * Can be called during an active session to dynamically control streaming.
     * Only effective if session mode includes streaming.
     *
     * @param enabled True to enable streamer, false to disable
     */
    fun setStreamerEnabled(enabled: Boolean)
}
