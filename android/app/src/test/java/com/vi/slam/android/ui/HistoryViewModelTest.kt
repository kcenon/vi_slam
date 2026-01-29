package com.vi.slam.android.ui

import android.app.Application
import com.vi.slam.android.data.Session
import com.vi.slam.android.data.SortCriteria
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.*
import org.junit.After
import org.junit.Before
import org.junit.Test
import org.junit.Assert.*
import org.mockito.Mockito.mock
import org.mockito.Mockito.`when`

/**
 * Unit tests for HistoryViewModel.
 * Tests session list management, search, and sort functionality.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class HistoryViewModelTest {

    private lateinit var viewModel: HistoryViewModel
    private lateinit var mockApplication: Application
    private val testDispatcher = StandardTestDispatcher()

    @Before
    fun setup() {
        Dispatchers.setMain(testDispatcher)
        mockApplication = mock(Application::class.java)
        `when`(mockApplication.applicationContext).thenReturn(mockApplication)
        `when`(mockApplication.filesDir).thenReturn(mock())
        viewModel = HistoryViewModel(mockApplication)
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun `initial state should be loading`() {
        assertTrue(viewModel.isLoading.value)
    }

    @Test
    fun `initial sessions should be empty`() {
        assertEquals(emptyList<Session>(), viewModel.sessions.value)
    }

    @Test
    fun `initial search query should be empty`() {
        assertEquals("", viewModel.searchQuery.value)
    }

    @Test
    fun `initial sort criteria should be DATE`() {
        assertEquals(SortCriteria.DATE, viewModel.sortCriteria.value)
    }

    @Test
    fun `updateSearchQuery should update query state`() = runTest {
        val testQuery = "session_123"
        viewModel.updateSearchQuery(testQuery)
        advanceUntilIdle()

        assertEquals(testQuery, viewModel.searchQuery.value)
    }

    @Test
    fun `updateSearchQuery with empty string should clear search`() = runTest {
        viewModel.updateSearchQuery("test")
        advanceUntilIdle()
        viewModel.updateSearchQuery("")
        advanceUntilIdle()

        assertEquals("", viewModel.searchQuery.value)
    }

    @Test
    fun `updateSortCriteria should update sort criteria`() = runTest {
        viewModel.updateSortCriteria(SortCriteria.DURATION)
        advanceUntilIdle()

        assertEquals(SortCriteria.DURATION, viewModel.sortCriteria.value)
    }

    @Test
    fun `updateSortCriteria to SIZE should update criteria`() = runTest {
        viewModel.updateSortCriteria(SortCriteria.SIZE)
        advanceUntilIdle()

        assertEquals(SortCriteria.SIZE, viewModel.sortCriteria.value)
    }

    @Test
    fun `loadSessions should set isLoading to false after completion`() = runTest {
        viewModel.loadSessions()
        advanceUntilIdle()

        assertFalse(viewModel.isLoading.value)
    }

    @Test
    fun `getSession with non-existent ID should return null`() {
        val session = viewModel.getSession("non_existent_id")

        assertNull(session)
    }

    @Test
    fun `SortCriteria should have all expected values`() {
        val criteria = SortCriteria.values()

        assertEquals(3, criteria.size)
        assertTrue(criteria.contains(SortCriteria.DATE))
        assertTrue(criteria.contains(SortCriteria.DURATION))
        assertTrue(criteria.contains(SortCriteria.SIZE))
    }
}
