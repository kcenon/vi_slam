package com.vi.slam.android.ui

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.*
import org.junit.After
import org.junit.Before
import org.junit.Test
import org.junit.Assert.*

/**
 * Unit tests for MainViewModel.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class MainViewModelTest {

    private lateinit var viewModel: MainViewModel
    private val testDispatcher = StandardTestDispatcher()

    @Before
    fun setup() {
        Dispatchers.setMain(testDispatcher)
        viewModel = MainViewModel()
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun `initial state should have default values`() {
        val state = viewModel.uiState.value

        assertFalse(state.isRecording)
        assertFalse(state.isStreaming)
        assertEquals(0, state.cameraFps)
        assertEquals(0, state.imuRate)
        assertEquals(ConnectionStatus.DISCONNECTED, state.connectionStatus)
        assertNull(state.recordingError)
        assertNull(state.streamingError)
    }

    @Test
    fun `toggleRecording should start recording when not recording`() = runTest {
        viewModel.toggleRecording()
        advanceUntilIdle()

        val state = viewModel.uiState.value
        assertTrue(state.isRecording)
        assertNull(state.recordingError)
    }

    @Test
    fun `toggleRecording should stop recording when already recording`() = runTest {
        // Start recording first
        viewModel.toggleRecording()
        advanceUntilIdle()
        assertTrue(viewModel.uiState.value.isRecording)

        // Stop recording
        viewModel.toggleRecording()
        advanceUntilIdle()

        val state = viewModel.uiState.value
        assertFalse(state.isRecording)
    }

    @Test
    fun `toggleStreaming should start streaming when not streaming`() = runTest {
        viewModel.toggleStreaming()
        advanceUntilIdle()

        val state = viewModel.uiState.value
        assertTrue(state.isStreaming)
        assertEquals(ConnectionStatus.CONNECTED, state.connectionStatus)
        assertNull(state.streamingError)
    }

    @Test
    fun `toggleStreaming should stop streaming when already streaming`() = runTest {
        // Start streaming first
        viewModel.toggleStreaming()
        advanceUntilIdle()
        assertTrue(viewModel.uiState.value.isStreaming)

        // Stop streaming
        viewModel.toggleStreaming()
        advanceUntilIdle()

        val state = viewModel.uiState.value
        assertFalse(state.isStreaming)
        assertEquals(ConnectionStatus.DISCONNECTED, state.connectionStatus)
    }

    @Test
    fun `updateFps should update camera FPS`() {
        viewModel.updateFps(30)

        assertEquals(30, viewModel.uiState.value.cameraFps)
    }

    @Test
    fun `updateImuRate should update IMU rate`() {
        viewModel.updateImuRate(200)

        assertEquals(200, viewModel.uiState.value.imuRate)
    }

    @Test
    fun `onCleared should stop recording if active`() = runTest {
        viewModel.toggleRecording()
        advanceUntilIdle()
        assertTrue(viewModel.uiState.value.isRecording)

        viewModel.onCleared()

        assertFalse(viewModel.uiState.value.isRecording)
    }

    @Test
    fun `onCleared should stop streaming if active`() = runTest {
        viewModel.toggleStreaming()
        advanceUntilIdle()
        assertTrue(viewModel.uiState.value.isStreaming)

        viewModel.onCleared()

        assertFalse(viewModel.uiState.value.isStreaming)
    }

    @Test
    fun `streaming should transition through CONNECTING state`() = runTest {
        viewModel.toggleStreaming()

        // Initially should be CONNECTING
        var state = viewModel.uiState.value
        assertEquals(ConnectionStatus.CONNECTING, state.connectionStatus)

        // After delay, should be CONNECTED
        advanceUntilIdle()
        state = viewModel.uiState.value
        assertEquals(ConnectionStatus.CONNECTED, state.connectionStatus)
    }
}
