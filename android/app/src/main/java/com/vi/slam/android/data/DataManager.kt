package com.vi.slam.android.data

import android.util.Log
import com.vi.slam.android.sensor.TimestampSynchronizer
import java.io.File
import java.util.UUID
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

    // Statistics tracking
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
