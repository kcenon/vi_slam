package com.vi.slam.android.ui

import android.content.Context
import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.vi.slam.android.recorder.IRecorder
import com.vi.slam.android.recorder.LocalRecorder
import com.vi.slam.android.recorder.RecorderConfig
import com.vi.slam.android.recorder.VideoFormat
import com.vi.slam.android.recorder.ImuFormat
import com.vi.slam.android.streaming.WebRtcConnectionManager
import com.vi.slam.android.streaming.ConnectionListener
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import org.webrtc.DataChannel
import java.io.File

/**
 * ViewModel for the main screen UI.
 * Manages UI state and coordinates camera, sensor, and recording operations.
 *
 * @param context Application context for accessing file system and WebRTC
 * @param recorder Recorder instance for local recording (defaults to LocalRecorder)
 */
class MainViewModel(
    private val context: Context,
    private val recorder: IRecorder = LocalRecorder()
) : ViewModel() {

    companion object {
        private const val TAG = "MainViewModel"
    }

    private val _uiState = MutableStateFlow(MainUiState())
    val uiState: StateFlow<MainUiState> = _uiState.asStateFlow()

    private var connectionManager: WebRtcConnectionManager? = null
    private var isRecorderInitialized = false

    /**
     * Start or stop recording based on current state.
     */
    fun toggleRecording() {
        viewModelScope.launch {
            val currentState = _uiState.value
            if (currentState.isRecording) {
                stopRecording()
            } else {
                startRecording()
            }
        }
    }

    /**
     * Start or stop streaming based on current state.
     */
    fun toggleStreaming() {
        viewModelScope.launch {
            val currentState = _uiState.value
            if (currentState.isStreaming) {
                stopStreaming()
            } else {
                startStreaming()
            }
        }
    }

    /**
     * Update camera FPS.
     */
    fun updateFps(fps: Int) {
        _uiState.value = _uiState.value.copy(cameraFps = fps)
    }

    /**
     * Update IMU sample rate.
     */
    fun updateImuRate(rate: Int) {
        _uiState.value = _uiState.value.copy(imuRate = rate)
    }

    private fun startRecording() {
        viewModelScope.launch {
            try {
                // Initialize recorder if not already done
                if (!isRecorderInitialized) {
                    val outputDir = File(context.filesDir, "recordings").apply {
                        if (!exists()) mkdirs()
                    }

                    val config = RecorderConfig(
                        outputDirectory = outputDir,
                        videoFormat = VideoFormat.H264_MP4,
                        imuFormat = ImuFormat.CSV,
                        maxDurationMs = 60 * 60 * 1000,  // 60 minutes
                        enableMetadata = true,
                        videoWidth = 640,
                        videoHeight = 480,
                        videoFps = 30,
                        videoBitrate = 2_000_000  // 2 Mbps
                    )

                    recorder.initialize(config).onFailure { error ->
                        Log.e(TAG, "Failed to initialize recorder", error)
                        _uiState.value = _uiState.value.copy(
                            recordingError = "Failed to initialize: ${error.message}"
                        )
                        return@launch
                    }

                    isRecorderInitialized = true
                    Log.i(TAG, "Recorder initialized successfully")
                }

                // Start recording
                recorder.startRecording().onSuccess { recordingInfo ->
                    Log.i(TAG, "Recording started: ${recordingInfo.recordingId}")
                    _uiState.value = _uiState.value.copy(
                        isRecording = true,
                        recordingError = null
                    )
                }.onFailure { error ->
                    Log.e(TAG, "Failed to start recording", error)
                    _uiState.value = _uiState.value.copy(
                        isRecording = false,
                        recordingError = "Failed to start: ${error.message}"
                    )
                }
            } catch (e: Exception) {
                Log.e(TAG, "Exception during recording start", e)
                _uiState.value = _uiState.value.copy(
                    isRecording = false,
                    recordingError = "Error: ${e.message}"
                )
            }
        }
    }

    private fun stopRecording() {
        viewModelScope.launch {
            try {
                recorder.stopRecording().onSuccess { summary ->
                    Log.i(TAG, "Recording stopped: ${summary.frameCount} frames, duration: ${summary.durationMs}ms")
                    _uiState.value = _uiState.value.copy(
                        isRecording = false,
                        recordingError = null
                    )
                }.onFailure { error ->
                    Log.e(TAG, "Failed to stop recording", error)
                    _uiState.value = _uiState.value.copy(
                        isRecording = false,
                        recordingError = "Failed to stop: ${error.message}"
                    )
                }
            } catch (e: Exception) {
                Log.e(TAG, "Exception during recording stop", e)
                _uiState.value = _uiState.value.copy(
                    isRecording = false,
                    recordingError = "Error: ${error.message}"
                )
            }
        }
    }

    private fun startStreaming() {
        viewModelScope.launch {
            try {
                // Update UI to show connecting state
                _uiState.value = _uiState.value.copy(
                    isStreaming = true,
                    connectionStatus = ConnectionStatus.CONNECTING,
                    streamingError = null
                )

                // Initialize connection manager if needed
                if (connectionManager == null) {
                    connectionManager = WebRtcConnectionManager(
                        context = context,
                        signalingServerUrl = "ws://localhost:8080/signaling",  // TODO: Get from settings
                        iceServers = emptyList(),  // TODO: Get from settings
                        listener = object : ConnectionListener {
                            override fun onConnected() {
                                Log.i(TAG, "WebRTC connection established")
                                _uiState.value = _uiState.value.copy(
                                    connectionStatus = ConnectionStatus.CONNECTED,
                                    streamingError = null
                                )
                            }

                            override fun onDisconnected(reason: String) {
                                Log.w(TAG, "WebRTC disconnected: $reason")
                                _uiState.value = _uiState.value.copy(
                                    isStreaming = false,
                                    connectionStatus = ConnectionStatus.DISCONNECTED,
                                    streamingError = reason
                                )
                            }

                            override fun onDataChannelReady(channel: DataChannel) {
                                Log.i(TAG, "DataChannel ready: ${channel.label()}")
                                // TODO: Pass channel to data pipeline for streaming
                            }

                            override fun onError(message: String) {
                                Log.e(TAG, "WebRTC error: $message")
                                _uiState.value = _uiState.value.copy(
                                    isStreaming = false,
                                    connectionStatus = ConnectionStatus.ERROR,
                                    streamingError = message
                                )
                            }
                        }
                    )

                    connectionManager?.initialize()
                    Log.i(TAG, "WebRTC connection manager initialized")
                }

                // Start connection as offerer (Android app initiates)
                connectionManager?.startAsOfferer()
                Log.i(TAG, "WebRTC connection started")

            } catch (e: Exception) {
                Log.e(TAG, "Exception during streaming start", e)
                _uiState.value = _uiState.value.copy(
                    isStreaming = false,
                    connectionStatus = ConnectionStatus.ERROR,
                    streamingError = "Error: ${e.message}"
                )
            }
        }
    }

    private fun stopStreaming() {
        viewModelScope.launch {
            try {
                connectionManager?.disconnect()
                Log.i(TAG, "WebRTC connection stopped")

                _uiState.value = _uiState.value.copy(
                    isStreaming = false,
                    connectionStatus = ConnectionStatus.DISCONNECTED,
                    streamingError = null
                )
            } catch (e: Exception) {
                Log.e(TAG, "Exception during streaming stop", e)
                _uiState.value = _uiState.value.copy(
                    isStreaming = false,
                    connectionStatus = ConnectionStatus.DISCONNECTED,
                    streamingError = "Error: ${e.message}"
                )
            }
        }
    }

    override fun onCleared() {
        super.onCleared()
        // Clean up resources
        if (_uiState.value.isRecording) {
            stopRecording()
        }
        if (_uiState.value.isStreaming) {
            stopStreaming()
        }
    }
}

/**
 * UI state for the main screen.
 */
data class MainUiState(
    val isRecording: Boolean = false,
    val isStreaming: Boolean = false,
    val cameraFps: Int = 0,
    val imuRate: Int = 0,
    val connectionStatus: ConnectionStatus = ConnectionStatus.DISCONNECTED,
    val recordingError: String? = null,
    val streamingError: String? = null
)

/**
 * Connection status for streaming.
 */
enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
}
