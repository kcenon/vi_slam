package com.vi.slam.android.data

import android.content.Context
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.test.runTest
import org.junit.After
import org.junit.Before
import org.junit.Test
import org.junit.Assert.*
import org.mockito.Mockito.mock
import org.mockito.Mockito.`when`
import java.io.File

/**
 * Unit tests for SessionRepository.
 * Tests session CRUD operations and filtering functionality.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SessionRepositoryTest {

    private lateinit var repository: SessionRepository
    private lateinit var mockContext: Context
    private lateinit var testFilesDir: File

    @Before
    fun setup() {
        mockContext = mock(Context::class.java)
        testFilesDir = createTempDir("vi_slam_test")
        `when`(mockContext.filesDir).thenReturn(testFilesDir)
        repository = SessionRepository(mockContext)
    }

    @After
    fun tearDown() {
        testFilesDir.deleteRecursively()
    }

    @Test
    fun `initial sessions flow should be empty`() = runTest {
        val sessions = repository.sessionsFlow.first()
        assertEquals(emptyList<Session>(), sessions)
    }

    @Test
    fun `loadSessions with no sessions directory should return empty list`() = runTest {
        repository.loadSessions()
        val sessions = repository.sessionsFlow.first()

        assertEquals(emptyList<Session>(), sessions)
    }

    @Test
    fun `loadSessions should parse valid session metadata`() = runTest {
        // Create test session directory with metadata
        val sessionsDir = File(testFilesDir, "sessions")
        sessionsDir.mkdirs()

        val sessionDir = File(sessionsDir, "session_001")
        sessionDir.mkdirs()

        val metadataFile = File(sessionDir, "metadata.txt")
        metadataFile.writeText("""
            start_time=1609459200000
            duration=120000
            frame_count=3600
            imu_sample_count=12000
        """.trimIndent())

        // Create a dummy file to calculate size
        val dataFile = File(sessionDir, "data.bin")
        dataFile.writeText("test data")

        repository.loadSessions()
        val sessions = repository.sessionsFlow.first()

        assertEquals(1, sessions.size)
        assertEquals("session_001", sessions[0].id)
        assertEquals(1609459200000L, sessions[0].startTime)
        assertEquals(120000L, sessions[0].duration)
        assertEquals(3600, sessions[0].frameCount)
        assertEquals(12000, sessions[0].imuSampleCount)
    }

    @Test
    fun `loadSessions should skip invalid session directories`() = runTest {
        val sessionsDir = File(testFilesDir, "sessions")
        sessionsDir.mkdirs()

        // Create session without metadata file
        val invalidSessionDir = File(sessionsDir, "invalid_session")
        invalidSessionDir.mkdirs()

        repository.loadSessions()
        val sessions = repository.sessionsFlow.first()

        assertEquals(emptyList<Session>(), sessions)
    }

    @Test
    fun `addSession should add new session to list`() = runTest {
        val session = Session(
            id = "test_session",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session"
        )

        repository.addSession(session)
        val sessions = repository.sessionsFlow.first()

        assertEquals(1, sessions.size)
        assertEquals("test_session", sessions[0].id)
    }

    @Test
    fun `addSession should sort sessions by startTime descending`() = runTest {
        val session1 = Session(
            id = "session_1",
            startTime = 1000,
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session1"
        )

        val session2 = Session(
            id = "session_2",
            startTime = 2000,
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session2"
        )

        repository.addSession(session1)
        repository.addSession(session2)
        val sessions = repository.sessionsFlow.first()

        assertEquals(2, sessions.size)
        assertEquals("session_2", sessions[0].id)
        assertEquals("session_1", sessions[1].id)
    }

    @Test
    fun `deleteSession should remove session from list`() = runTest {
        val sessionDir = File(testFilesDir, "test_session")
        sessionDir.mkdirs()
        val dataFile = File(sessionDir, "data.bin")
        dataFile.writeText("test")

        val session = Session(
            id = "test_session",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = sessionDir.absolutePath
        )

        repository.addSession(session)
        assertEquals(1, repository.sessionsFlow.first().size)

        repository.deleteSession("test_session")
        assertEquals(0, repository.sessionsFlow.first().size)
        assertFalse(sessionDir.exists())
    }

    @Test
    fun `deleteSession with non-existent ID should not throw exception`() = runTest {
        repository.deleteSession("non_existent_id")
        assertEquals(0, repository.sessionsFlow.first().size)
    }

    @Test
    fun `getSession should return session by ID`() = runTest {
        val session = Session(
            id = "test_session",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session"
        )

        repository.addSession(session)
        val result = repository.getSession("test_session")

        assertNotNull(result)
        assertEquals("test_session", result?.id)
    }

    @Test
    fun `getSession with non-existent ID should return null`() {
        val result = repository.getSession("non_existent")
        assertNull(result)
    }

    @Test
    fun `searchSessions with empty query should return all sessions`() = runTest {
        val session = Session(
            id = "test_session",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session"
        )

        repository.addSession(session)
        val results = repository.searchSessions("")

        assertEquals(1, results.size)
    }

    @Test
    fun `searchSessions should filter by session ID`() = runTest {
        val session1 = Session(
            id = "test_session_001",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session1"
        )

        val session2 = Session(
            id = "other_session_002",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session2"
        )

        repository.addSession(session1)
        repository.addSession(session2)
        val results = repository.searchSessions("test")

        assertEquals(1, results.size)
        assertEquals("test_session_001", results[0].id)
    }

    @Test
    fun `sortSessions by DATE should sort descending`() = runTest {
        val session1 = Session(
            id = "session_1",
            startTime = 1000,
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session1"
        )

        val session2 = Session(
            id = "session_2",
            startTime = 2000,
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session2"
        )

        repository.addSession(session1)
        repository.addSession(session2)
        val sorted = repository.sortSessions(SortCriteria.DATE)

        assertEquals("session_2", sorted[0].id)
        assertEquals("session_1", sorted[1].id)
    }

    @Test
    fun `sortSessions by DURATION should sort descending`() = runTest {
        val session1 = Session(
            id = "session_1",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session1"
        )

        val session2 = Session(
            id = "session_2",
            startTime = System.currentTimeMillis(),
            duration = 120000,
            fileSize = 1024,
            frameCount = 3600,
            imuSampleCount = 12000,
            dataPath = "/path/to/session2"
        )

        repository.addSession(session1)
        repository.addSession(session2)
        val sorted = repository.sortSessions(SortCriteria.DURATION)

        assertEquals("session_2", sorted[0].id)
        assertEquals("session_1", sorted[1].id)
    }

    @Test
    fun `sortSessions by SIZE should sort descending`() = runTest {
        val session1 = Session(
            id = "session_1",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 1024,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session1"
        )

        val session2 = Session(
            id = "session_2",
            startTime = System.currentTimeMillis(),
            duration = 60000,
            fileSize = 2048,
            frameCount = 1800,
            imuSampleCount = 6000,
            dataPath = "/path/to/session2"
        )

        repository.addSession(session1)
        repository.addSession(session2)
        val sorted = repository.sortSessions(SortCriteria.SIZE)

        assertEquals("session_2", sorted[0].id)
        assertEquals("session_1", sorted[1].id)
    }
}
