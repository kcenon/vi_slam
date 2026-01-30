package com.vi.slam.android.ui

import android.content.Context
import androidx.test.core.app.ApplicationProvider
import com.vi.slam.android.recorder.IRecorder
import com.vi.slam.android.recorder.RecordingInfo
import com.vi.slam.android.recorder.RecordingSummary
import com.vi.slam.android.recorder.RecorderConfig
import com.vi.slam.android.recorder.VideoFormat
import com.vi.slam.android.recorder.ImuFormat
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.*
import org.junit.After
import org.junit.Before
import org.junit.Test
import org.junit.Assert.*
import org.junit.runner.RunWith
import org.mockito.Mockito.*
import org.mockito.kotlin.any
import org.robolectric.RobolectricTestRunner
import java.io.File

/**
 * Unit tests for MainViewModel.
 */
@OptIn(ExperimentalCoroutinesApi::class)
@RunWith(RobolectricTestRunner::class)
class MainViewModelTest {

    private lateinit var viewModel: MainViewModel
    private lateinit var testContext: Context
    private lateinit var mockRecorder: IRecorder
    private val testDispatcher = StandardTestDispatcher()

    @Before
    fun setup() {
        Dispatchers.setMain(testDispatcher)
        testContext = ApplicationProvider.getApplicationContext()
        mockRecorder = mock(IRecorder::class.java)

        // Setup mock recorder to return successful results
        val testOutputDir = File(testContext.filesDir, "recordings")
        testOutputDir.mkdirs()

        val mockConfig = RecorderConfig(
            outputDirectory = testOutputDir,
            videoFormat = VideoFormat.H264_MP4,
            imuFormat = ImuFormat.CSV
        )

        `when`(mockRecorder.initialize(any())).thenReturn(Result.success(Unit))
        `when`(mockRecorder.startRecording()).thenReturn(
            Result.success(
                RecordingInfo(
                    recordingId = "test-recording-id",
                    startTime = System.currentTimeMillis(),
                    outputPath = testOutputDir.absolutePath,
                    config = mockConfig
                )
            )
        )
        `when`(mockRecorder.stopRecording()).thenReturn(
            Result.success(
                RecordingSummary(
                    recordingId = "test-recording-id",
                    success = true,
                    frameCount = 30,
                    imuSampleCount = 200,
                    durationMs = 1000,
                    outputFiles = listOf("test.mp4", "test.csv"),
                    videoFile = "test.mp4",
                    imuFile = "test.csv"
                )
            )
        )

        viewModel = MainViewModel(testContext, mockRecorder)
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

    // TODO: Refactor MainViewModel to inject WebRtcConnectionManager for testability
    // WebRTC native library cannot be loaded in unit tests
    @org.junit.Ignore("WebRTC native library not available in unit tests")
    @Test
    fun `toggleStreaming should start streaming when not streaming`() = runTest {
        viewModel.toggleStreaming()
        advanceUntilIdle()

        val state = viewModel.uiState.value
        assertTrue(state.isStreaming)
        // WebRTC connection is async, so it may still be CONNECTING in tests
        assertTrue(
            state.connectionStatus == ConnectionStatus.CONNECTING ||
            state.connectionStatus == ConnectionStatus.CONNECTED
        )
    }

    @org.junit.Ignore("WebRTC native library not available in unit tests")
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
        // Connection should be disconnected after stopping
        assertTrue(
            state.connectionStatus == ConnectionStatus.DISCONNECTED ||
            state.connectionStatus == ConnectionStatus.CONNECTING
        )
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

    // Note: onCleared() is protected and will be called automatically by Android framework
    // Testing cleanup behavior indirectly through toggleRecording/toggleStreaming

    @org.junit.Ignore("WebRTC native library not available in unit tests")
    @Test
    fun `streaming should transition through CONNECTING state`() = runTest {
        viewModel.toggleStreaming()
        advanceUntilIdle()

        // Should be in CONNECTING state (WebRTC connection is async)
        val state = viewModel.uiState.value
        assertTrue(state.isStreaming)
        assertTrue(
            state.connectionStatus == ConnectionStatus.CONNECTING ||
            state.connectionStatus == ConnectionStatus.CONNECTED
        )
    }
}
