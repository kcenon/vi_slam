package com.vi.slam.android.ui

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * ViewModel for the main screen UI.
 * Manages UI state and coordinates camera, sensor, and recording operations.
 */
class MainViewModel : ViewModel() {

    private val _uiState = MutableStateFlow(MainUiState())
    val uiState: StateFlow<MainUiState> = _uiState.asStateFlow()

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
        // TODO: Integrate with RecorderModule (CMP-005)
        _uiState.value = _uiState.value.copy(
            isRecording = true,
            recordingError = null
        )
    }

    private fun stopRecording() {
        // TODO: Integrate with RecorderModule (CMP-005)
        _uiState.value = _uiState.value.copy(
            isRecording = false
        )
    }

    private fun startStreaming() {
        // TODO: Integrate with WebRtcConnectionManager
        _uiState.value = _uiState.value.copy(
            isStreaming = true,
            connectionStatus = ConnectionStatus.CONNECTING,
            streamingError = null
        )

        // Simulate connection (will be replaced with actual implementation)
        viewModelScope.launch {
            kotlinx.coroutines.delay(500)
            _uiState.value = _uiState.value.copy(
                connectionStatus = ConnectionStatus.CONNECTED
            )
        }
    }

    private fun stopStreaming() {
        // TODO: Integrate with WebRtcConnectionManager
        _uiState.value = _uiState.value.copy(
            isStreaming = false,
            connectionStatus = ConnectionStatus.DISCONNECTED
        )
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
