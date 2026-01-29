package com.vi.slam.android.data

import android.content.Context
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.File

/**
 * Repository for managing recording sessions.
 * Provides CRUD operations for session metadata.
 */
class SessionRepository(private val context: Context) {

    private val _sessions = MutableStateFlow<List<Session>>(emptyList())
    val sessionsFlow: Flow<List<Session>> = _sessions.asStateFlow()

    /**
     * Load all sessions from storage.
     */
    suspend fun loadSessions() {
        val sessionList = mutableListOf<Session>()
        val sessionsDir = File(context.filesDir, "sessions")

        if (sessionsDir.exists() && sessionsDir.isDirectory) {
            sessionsDir.listFiles()?.forEach { sessionDir ->
                if (sessionDir.isDirectory) {
                    val metadataFile = File(sessionDir, "metadata.txt")
                    if (metadataFile.exists()) {
                        try {
                            val session = parseMetadata(sessionDir, metadataFile)
                            sessionList.add(session)
                        } catch (e: Exception) {
                            // Skip invalid sessions
                        }
                    }
                }
            }
        }

        _sessions.value = sessionList.sortedByDescending { it.startTime }
    }

    /**
     * Add a new session.
     */
    suspend fun addSession(session: Session) {
        val currentSessions = _sessions.value.toMutableList()
        currentSessions.add(session)
        _sessions.value = currentSessions.sortedByDescending { it.startTime }
    }

    /**
     * Delete a session by ID.
     */
    suspend fun deleteSession(sessionId: String) {
        val session = _sessions.value.find { it.id == sessionId } ?: return

        // Delete session files
        val sessionDir = File(session.dataPath)
        if (sessionDir.exists()) {
            sessionDir.deleteRecursively()
        }

        // Update state
        val currentSessions = _sessions.value.toMutableList()
        currentSessions.removeIf { it.id == sessionId }
        _sessions.value = currentSessions
    }

    /**
     * Get session by ID.
     */
    fun getSession(sessionId: String): Session? {
        return _sessions.value.find { it.id == sessionId }
    }

    /**
     * Parse metadata file to create Session object.
     */
    private fun parseMetadata(sessionDir: File, metadataFile: File): Session {
        val metadata = metadataFile.readLines()
            .associate {
                val parts = it.split("=", limit = 2)
                if (parts.size == 2) parts[0] to parts[1] else "" to ""
            }

        val id = sessionDir.name
        val startTime = metadata["start_time"]?.toLongOrNull() ?: 0L
        val duration = metadata["duration"]?.toLongOrNull() ?: 0L
        val frameCount = metadata["frame_count"]?.toIntOrNull() ?: 0
        val imuSampleCount = metadata["imu_sample_count"]?.toIntOrNull() ?: 0
        val thumbnailPath = File(sessionDir, "thumbnail.jpg").let {
            if (it.exists()) it.absolutePath else null
        }

        // Calculate total file size
        val fileSize = sessionDir.walkTopDown()
            .filter { it.isFile }
            .map { it.length() }
            .sum()

        return Session(
            id = id,
            startTime = startTime,
            duration = duration,
            fileSize = fileSize,
            frameCount = frameCount,
            imuSampleCount = imuSampleCount,
            thumbnailPath = thumbnailPath,
            dataPath = sessionDir.absolutePath
        )
    }

    /**
     * Search sessions by keyword.
     */
    fun searchSessions(query: String): List<Session> {
        if (query.isBlank()) return _sessions.value

        return _sessions.value.filter { session ->
            session.id.contains(query, ignoreCase = true)
        }
    }

    /**
     * Sort sessions by different criteria.
     */
    fun sortSessions(sortBy: SortCriteria): List<Session> {
        return when (sortBy) {
            SortCriteria.DATE -> _sessions.value.sortedByDescending { it.startTime }
            SortCriteria.DURATION -> _sessions.value.sortedByDescending { it.duration }
            SortCriteria.SIZE -> _sessions.value.sortedByDescending { it.fileSize }
        }
    }
}

/**
 * Sort criteria for sessions.
 */
enum class SortCriteria {
    DATE,
    DURATION,
    SIZE
}
