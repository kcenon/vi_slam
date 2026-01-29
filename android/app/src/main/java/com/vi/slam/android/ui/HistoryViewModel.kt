package com.vi.slam.android.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.vi.slam.android.data.Session
import com.vi.slam.android.data.SessionRepository
import com.vi.slam.android.data.SortCriteria
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

/**
 * ViewModel for the session history screen.
 * Manages session list, search, and sort operations.
 */
class HistoryViewModel(application: Application) : AndroidViewModel(application) {

    private val repository = SessionRepository(application)

    private val _searchQuery = MutableStateFlow("")
    private val _sortCriteria = MutableStateFlow(SortCriteria.DATE)
    private val _isLoading = MutableStateFlow(true)

    /**
     * Current search query.
     */
    val searchQuery: StateFlow<String> = _searchQuery

    /**
     * Current sort criteria.
     */
    val sortCriteria: StateFlow<SortCriteria> = _sortCriteria

    /**
     * Loading state.
     */
    val isLoading: StateFlow<Boolean> = _isLoading

    /**
     * List of sessions with search and sort applied.
     */
    val sessions: StateFlow<List<Session>> = combine(
        repository.sessionsFlow,
        _searchQuery,
        _sortCriteria
    ) { sessions, query, sortBy ->
        val filtered = if (query.isBlank()) {
            sessions
        } else {
            sessions.filter { session ->
                session.id.contains(query, ignoreCase = true)
            }
        }

        when (sortBy) {
            SortCriteria.DATE -> filtered.sortedByDescending { it.startTime }
            SortCriteria.DURATION -> filtered.sortedByDescending { it.duration }
            SortCriteria.SIZE -> filtered.sortedByDescending { it.fileSize }
        }
    }.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(5000),
        initialValue = emptyList()
    )

    init {
        loadSessions()
    }

    /**
     * Load sessions from storage.
     */
    fun loadSessions() {
        viewModelScope.launch {
            _isLoading.value = true
            repository.loadSessions()
            _isLoading.value = false
        }
    }

    /**
     * Update search query.
     *
     * @param query Search query string
     */
    fun updateSearchQuery(query: String) {
        _searchQuery.value = query
    }

    /**
     * Update sort criteria.
     *
     * @param criteria New sort criteria
     */
    fun updateSortCriteria(criteria: SortCriteria) {
        _sortCriteria.value = criteria
    }

    /**
     * Delete a session.
     *
     * @param sessionId Session ID to delete
     */
    fun deleteSession(sessionId: String) {
        viewModelScope.launch {
            repository.deleteSession(sessionId)
        }
    }

    /**
     * Get session by ID.
     *
     * @param sessionId Session ID
     * @return Session object or null if not found
     */
    fun getSession(sessionId: String): Session? {
        return repository.getSession(sessionId)
    }
}
