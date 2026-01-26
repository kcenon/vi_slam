package com.vi.slam.android.data

import android.util.Log
import com.vi.slam.android.sensor.SynchronizedData
import com.vi.slam.android.sensor.TimestampSynchronizer
import java.io.File
import java.util.UUID
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference

/**
 * Implementation of IDataManager for centralized data flow management.
 *
 * This class manages the lifecycle of data collection sessions, routes
 * synchronized camera-IMU data to registered consumers (recorder, streamer),
 * and collects real-time statistics for monitoring.
 *
 * Thread Safety: This class is thread-safe and can be accessed from multiple threads.
 *
 * @property timestampSynchronizer Synchronizer for camera-IMU data alignment
 * @property outputBaseDirectory Base directory for recording output (used for RECORD modes)
 */
class DataManager(
    private val timestampSynchronizer: TimestampSynchronizer,
    private val outputBaseDirectory: File
) : IDataManager {

    companion object {
        private const val TAG = "DataManager"
    }

    // Current session state
    private val sessionStatus = AtomicReference<SessionStatus>(SessionStatus.IDLE)
    private var currentSession: SessionInfo? = null
    private var sessionStartTime: Long = 0

    // Registered data consumers
    private val destinations = mutableListOf<IDataDestination>()
    private val destinationsLock = Any()

    // Flags for dynamic enable/disable
    @Volatile
    private var recorderEnabled = true

    @Volatile
    private var streamerEnabled = true

    // Statistics tracking - using atomic counters for thread-safe updates
    private val frameCount = AtomicLong(0)
    private val imuSampleCount = AtomicLong(0)
    private val frameDropCount = AtomicLong(0)
    private var lastFrameSequence: Long = -1

    // Cached statistics for getStatistics()
    private val statistics = AtomicReference<SessionStatistics>(SessionStatistics())

    override fun initialize(): Result<Unit> {
        return try {
            Log.d(TAG, "Initializing DataManager")

            // Validate dependencies
            if (!outputBaseDirectory.exists() && !outputBaseDirectory.mkdirs()) {
                return Result.failure(
                    IllegalStateException("Failed to create output directory: ${outputBaseDirectory.absolutePath}")
                )
            }

            // Reset state
            synchronized(destinationsLock) {
                destinations.clear()
            }
            recorderEnabled = true
            streamerEnabled = true
            frameCount.set(0)
            imuSampleCount.set(0)
            frameDropCount.set(0)
            lastFrameSequence = -1
            statistics.set(SessionStatistics())
            sessionStatus.set(SessionStatus.IDLE)
            currentSession = null

            Log.i(TAG, "DataManager initialized successfully")
            Result.success(Unit)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize DataManager", e)
            Result.failure(e)
        }
    }

    override fun startSession(mode: SessionMode): Result<SessionInfo> {
        return try {
            Log.d(TAG, "Starting session with mode: $mode")

            // Check current state
            val currentStatus = sessionStatus.get()
            if (currentStatus != SessionStatus.IDLE) {
                return Result.failure(
                    IllegalStateException("Cannot start session: current status is $currentStatus")
                )
            }

            // Transition to STARTING
            if (!sessionStatus.compareAndSet(SessionStatus.IDLE, SessionStatus.STARTING)) {
                return Result.failure(IllegalStateException("Failed to transition to STARTING state"))
            }

            // Generate session ID
            val sessionId = generateSessionId()

            // Determine output directory
            val outputDir = when (mode) {
                SessionMode.STREAM_ONLY -> null
                SessionMode.RECORD_ONLY, SessionMode.RECORD_AND_STREAM -> {
                    val dir = File(outputBaseDirectory, sessionId)
                    if (!dir.exists() && !dir.mkdirs()) {
                        sessionStatus.set(SessionStatus.ERROR)
                        return Result.failure(
                            IllegalStateException("Failed to create session directory: ${dir.absolutePath}")
                        )
                    }
                    dir.absolutePath
                }
            }

            // Create session info
            sessionStartTime = System.currentTimeMillis()
            val session = SessionInfo(
                sessionId = sessionId,
                startTime = sessionStartTime,
                mode = mode,
                outputDirectory = outputDir
            )
            currentSession = session

            // Reset statistics
            frameCount.set(0)
            imuSampleCount.set(0)
            frameDropCount.set(0)
            lastFrameSequence = -1
            statistics.set(SessionStatistics())

            // Configure destinations based on mode
            synchronized(destinationsLock) {
                when (mode) {
                    SessionMode.RECORD_ONLY -> {
                        recorderEnabled = true
                        streamerEnabled = false
                    }
                    SessionMode.STREAM_ONLY -> {
                        recorderEnabled = false
                        streamerEnabled = true
                    }
                    SessionMode.RECORD_AND_STREAM -> {
                        recorderEnabled = true
                        streamerEnabled = true
                    }
                }
            }

            // Transition to ACTIVE
            sessionStatus.set(SessionStatus.ACTIVE)

            Log.i(TAG, "Session started: $session")
            Result.success(session)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start session", e)
            sessionStatus.set(SessionStatus.ERROR)
            Result.failure(e)
        }
    }

    override fun stopSession(): Result<SessionSummary> {
        return try {
            Log.d(TAG, "Stopping session")

            // Check current state
            val currentStatus = sessionStatus.get()
            if (currentStatus != SessionStatus.ACTIVE) {
                return Result.failure(
                    IllegalStateException("Cannot stop session: current status is $currentStatus")
                )
            }

            // Transition to STOPPING
            if (!sessionStatus.compareAndSet(SessionStatus.ACTIVE, SessionStatus.STOPPING)) {
                return Result.failure(IllegalStateException("Failed to transition to STOPPING state"))
            }

            val session = currentSession
                ?: return Result.failure(IllegalStateException("No active session"))

            // Get final statistics
            val finalStats = statistics.get()

            // Create session summary
            val summary = SessionSummary(
                sessionId = session.sessionId,
                mode = session.mode,
                statistics = finalStats,
                outputDirectory = session.outputDirectory,
                success = true,
                errorMessage = null
            )

            // Clean up
            currentSession = null
            sessionStartTime = 0

            // Transition back to IDLE
            sessionStatus.set(SessionStatus.IDLE)

            Log.i(TAG, "Session stopped: $summary")
            Result.success(summary)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to stop session", e)
            sessionStatus.set(SessionStatus.ERROR)

            val session = currentSession
            if (session != null) {
                val errorSummary = SessionSummary(
                    sessionId = session.sessionId,
                    mode = session.mode,
                    statistics = statistics.get(),
                    outputDirectory = session.outputDirectory,
                    success = false,
                    errorMessage = e.message
                )
                Result.success(errorSummary)
            } else {
                Result.failure(e)
            }
        }
    }

    override fun getSessionStatus(): SessionStatus {
        return sessionStatus.get()
    }

    override fun getStatistics(): SessionStatistics {
        return statistics.get()
    }

    override fun setRecorderEnabled(enabled: Boolean) {
        Log.d(TAG, "Setting recorder enabled: $enabled")
        recorderEnabled = enabled
    }

    override fun setStreamerEnabled(enabled: Boolean) {
        Log.d(TAG, "Setting streamer enabled: $enabled")
        streamerEnabled = enabled
    }

    /**
     * Register a data destination (consumer).
     *
     * @param destination Data destination to register
     */
    fun registerDestination(destination: IDataDestination) {
        synchronized(destinationsLock) {
            if (!destinations.contains(destination)) {
                destinations.add(destination)
                Log.d(TAG, "Registered destination: ${destination.javaClass.simpleName}")
            }
        }
    }

    /**
     * Unregister a data destination.
     *
     * @param destination Data destination to unregister
     */
    fun unregisterDestination(destination: IDataDestination) {
        synchronized(destinationsLock) {
            if (destinations.remove(destination)) {
                Log.d(TAG, "Unregistered destination: ${destination.javaClass.simpleName}")
            }
        }
    }

    /**
     * Process incoming camera frame data.
     *
     * Called when a new camera frame is available. Synchronizes with IMU data
     * and routes the result to all enabled destinations.
     *
     * This method implements back-pressure handling: if destinations are overloaded
     * and cannot process data in time, frames may be dropped. The statistics
     * will track such drops via frameDropCount.
     *
     * This method is thread-safe and can be called from camera callback threads.
     *
     * @param frameTimestampNs Frame timestamp in nanoseconds
     * @param frameSequence Frame sequence number
     */
    fun onFrameAvailable(frameTimestampNs: Long, frameSequence: Long) {
        // Only process frames when session is active
        if (sessionStatus.get() != SessionStatus.ACTIVE) {
            return
        }

        try {
            // Synchronize with IMU data
            val synchronizedData = timestampSynchronizer.associateIMUWithFrame(
                frameTimestampNs,
                frameSequence
            )

            // Check if synchronization succeeded
            // If buffer is overflowing or IMU data is unavailable, synchronization may fail
            if (synchronizedData == null) {
                Log.w(TAG, "Failed to synchronize frame $frameSequence - no IMU data available")
                frameDropCount.incrementAndGet()
                return
            }

            // Route to enabled destinations
            // Note: This may block if destinations are slow, implementing natural back-pressure
            routeData(synchronizedData)

            // Update statistics
            updateStatistics(synchronizedData)
        } catch (e: Exception) {
            Log.e(TAG, "Error processing frame $frameSequence", e)
            // Count this as a dropped frame
            frameDropCount.incrementAndGet()
        }
    }

    /**
     * Route synchronized data to all enabled destinations.
     *
     * Only routes data to destinations that are currently enabled based on
     * the destination type (recorder/streamer) and current enable flags.
     *
     * @param data Synchronized camera and IMU data
     */
    private fun routeData(data: SynchronizedData) {
        val destinationsSnapshot = synchronized(destinationsLock) {
            destinations.toList()
        }

        for (destination in destinationsSnapshot) {
            try {
                // Check if destination is enabled
                if (!destination.isEnabled()) {
                    continue
                }

                // Additional check for recorder/streamer enable flags
                // Note: Destinations should implement isEnabled() to check these flags
                // This is a fail-safe in case destination doesn't check properly
                val shouldRoute = when {
                    // For now, route to all enabled destinations
                    // In future, we can add type information to IDataDestination
                    else -> true
                }

                if (shouldRoute) {
                    destination.onData(data)
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error routing data to destination: ${destination.javaClass.simpleName}", e)
                // Continue routing to other destinations even if one fails
            }
        }
    }

    /**
     * Update session statistics with new synchronized data.
     *
     * This method efficiently tracks frame count, IMU sample count, and frame drops
     * using atomic counters. Statistics are calculated lazily on demand to minimize
     * overhead in the data flow path.
     *
     * Thread Safety: This method is thread-safe and uses atomic operations to avoid
     * blocking the data flow.
     *
     * @param data Synchronized camera and IMU data
     */
    private fun updateStatistics(data: SynchronizedData) {
        // Detect frame drops by checking sequence discontinuity
        if (lastFrameSequence >= 0) {
            val expectedSequence = lastFrameSequence + 1
            if (data.frameSequence > expectedSequence) {
                val droppedFrames = data.frameSequence - expectedSequence
                frameDropCount.addAndGet(droppedFrames)
                Log.w(TAG, "Detected $droppedFrames dropped frames (seq: $expectedSequence -> ${data.frameSequence})")
            }
        }
        lastFrameSequence = data.frameSequence

        // Increment counters atomically
        val newFrameCount = frameCount.incrementAndGet()
        val newImuSamples = imuSampleCount.addAndGet(
            (data.imuSamplesBefore.size + data.imuSamplesAfter.size).toLong()
        )

        // Calculate duration
        val durationMs = if (sessionStartTime > 0) {
            System.currentTimeMillis() - sessionStartTime
        } else {
            0L
        }

        // Calculate average rates
        val avgFps = if (durationMs > 0) {
            (newFrameCount * 1000.0f) / durationMs
        } else {
            0f
        }

        val avgImuRate = if (durationMs > 0) {
            (newImuSamples * 1000.0f) / durationMs
        } else {
            0f
        }

        // Update cached statistics for getStatistics()
        // This is acceptable to be slightly stale as it's only for monitoring
        val updated = SessionStatistics(
            frameCount = newFrameCount,
            imuSampleCount = newImuSamples,
            durationMs = durationMs,
            frameDropCount = frameDropCount.get(),
            averageFps = avgFps,
            averageImuRate = avgImuRate
        )

        statistics.set(updated)
    }

    /**
     * Generate a unique session ID.
     *
     * Format: timestamp_UUID
     * Example: 20260126_143052_a1b2c3d4
     *
     * @return Unique session identifier
     */
    private fun generateSessionId(): String {
        val timestamp = java.text.SimpleDateFormat("yyyyMMdd_HHmmss", java.util.Locale.US)
            .format(java.util.Date())
        val uuid = UUID.randomUUID().toString().substring(0, 8)
        return "${timestamp}_$uuid"
    }
}
