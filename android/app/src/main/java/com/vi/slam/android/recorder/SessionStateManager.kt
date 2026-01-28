package com.vi.slam.android.recorder

import android.content.Context
import android.content.SharedPreferences
import android.util.Log
import com.google.gson.Gson
import com.google.gson.GsonBuilder

/**
 * Manager for persisting and retrieving recording session state.
 *
 * Uses SharedPreferences to track active recording sessions, allowing
 * detection and recovery of interrupted sessions on app restart.
 *
 * Session state is updated:
 * - On recording start (initial state)
 * - Every 100 frames (periodic checkpoint)
 * - On recording stop (session cleared)
 *
 * Thread Safety: All methods are synchronized for thread-safe access.
 */
class SessionStateManager(context: Context) {

    companion object {
        private const val TAG = "SessionStateManager"
        private const val PREFS_NAME = "vi_slam_recorder_state"
        private const val KEY_ACTIVE_SESSIONS = "active_sessions"
    }

    private val prefs: SharedPreferences = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
    private val gson: Gson = GsonBuilder().create()

    /**
     * Internal representation of session state for persistence.
     */
    private data class SessionState(
        val recordingId: String,
        val startTime: Long,
        val outputPath: String,
        val frameCount: Long,
        val imuCount: Long,
        val lastUpdateTime: Long
    )

    /**
     * Save or update session state.
     *
     * @param recordingId Unique recording identifier
     * @param startTime Recording start timestamp in milliseconds
     * @param outputPath Session output directory path
     * @param frameCount Current frame count
     * @param imuCount Current IMU sample count
     */
    @Synchronized
    fun saveSessionState(
        recordingId: String,
        startTime: Long,
        outputPath: String,
        frameCount: Long,
        imuCount: Long
    ) {
        try {
            val sessions = loadAllSessions().toMutableMap()

            val state = SessionState(
                recordingId = recordingId,
                startTime = startTime,
                outputPath = outputPath,
                frameCount = frameCount,
                imuCount = imuCount,
                lastUpdateTime = System.currentTimeMillis()
            )

            sessions[recordingId] = state

            val json = gson.toJson(sessions)
            prefs.edit().putString(KEY_ACTIVE_SESSIONS, json).apply()

            Log.d(TAG, "Session state saved: $recordingId (frames: $frameCount, imu: $imuCount)")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save session state: $recordingId", e)
        }
    }

    /**
     * Remove session state (called on successful recording stop).
     *
     * @param recordingId Recording ID to remove
     */
    @Synchronized
    fun removeSessionState(recordingId: String) {
        try {
            val sessions = loadAllSessions().toMutableMap()
            sessions.remove(recordingId)

            val json = gson.toJson(sessions)
            prefs.edit().putString(KEY_ACTIVE_SESSIONS, json).apply()

            Log.d(TAG, "Session state removed: $recordingId")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to remove session state: $recordingId", e)
        }
    }

    /**
     * List all incomplete sessions that may need recovery.
     *
     * @return List of recoverable sessions
     */
    @Synchronized
    fun listIncompleteSession(): List<RecoverableSession> {
        return try {
            val sessions = loadAllSessions()

            sessions.values.map { state ->
                RecoverableSession(
                    recordingId = state.recordingId,
                    startTime = state.startTime,
                    outputPath = state.outputPath,
                    estimatedFrameCount = state.frameCount,
                    estimatedImuCount = state.imuCount,
                    lastUpdateTime = state.lastUpdateTime
                )
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to list incomplete sessions", e)
            emptyList()
        }
    }

    /**
     * Get session state by recording ID.
     *
     * @param recordingId Recording ID to retrieve
     * @return RecoverableSession or null if not found
     */
    @Synchronized
    fun getSessionState(recordingId: String): RecoverableSession? {
        return try {
            val sessions = loadAllSessions()
            val state = sessions[recordingId] ?: return null

            RecoverableSession(
                recordingId = state.recordingId,
                startTime = state.startTime,
                outputPath = state.outputPath,
                estimatedFrameCount = state.frameCount,
                estimatedImuCount = state.imuCount,
                lastUpdateTime = state.lastUpdateTime
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to get session state: $recordingId", e)
            null
        }
    }

    /**
     * Clear all session state (for testing or reset).
     */
    @Synchronized
    fun clearAllSessions() {
        prefs.edit().remove(KEY_ACTIVE_SESSIONS).apply()
        Log.d(TAG, "All session states cleared")
    }

    /**
     * Load all session states from SharedPreferences.
     *
     * @return Map of recording ID to SessionState
     */
    private fun loadAllSessions(): Map<String, SessionState> {
        return try {
            val json = prefs.getString(KEY_ACTIVE_SESSIONS, null) ?: return emptyMap()

            @Suppress("UNCHECKED_CAST")
            val sessionsMap = gson.fromJson(json, Map::class.java) as? Map<String, Map<String, Any>>
                ?: return emptyMap()

            sessionsMap.mapValues { (_, stateMap) ->
                SessionState(
                    recordingId = stateMap["recordingId"] as String,
                    startTime = (stateMap["startTime"] as Number).toLong(),
                    outputPath = stateMap["outputPath"] as String,
                    frameCount = (stateMap["frameCount"] as Number).toLong(),
                    imuCount = (stateMap["imuCount"] as Number).toLong(),
                    lastUpdateTime = (stateMap["lastUpdateTime"] as Number).toLong()
                )
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load session states", e)
            emptyMap()
        }
    }
}
