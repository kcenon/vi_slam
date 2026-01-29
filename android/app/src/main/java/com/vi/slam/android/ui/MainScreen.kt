package com.vi.slam.android.ui

import androidx.compose.foundation.layout.*
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.lifecycle.viewmodel.compose.viewModel

/**
 * Main screen of the VI-SLAM application.
 * Shows camera preview, control buttons, and status information.
 *
 * @param viewModel ViewModel managing the main screen state
 * @param onSettingsClick Callback when settings button is clicked
 * @param onCalibrationClick Callback when calibration button is clicked
 * @param onHistoryClick Callback when history button is clicked
 * @param modifier Modifier for the screen container
 */
@Composable
fun MainScreen(
    viewModel: MainViewModel = viewModel(),
    onSettingsClick: () -> Unit = {},
    onCalibrationClick: () -> Unit = {},
    onHistoryClick: () -> Unit = {},
    modifier: Modifier = Modifier
) {
    val uiState by viewModel.uiState.collectAsState()

    Surface(
        modifier = modifier.fillMaxSize(),
        color = MaterialTheme.colorScheme.background
    ) {
        Box(modifier = Modifier.fillMaxSize()) {
            // Camera preview (background layer)
            CameraPreview(
                modifier = Modifier.fillMaxSize()
            )

            // Status bar (top layer)
            Column(
                modifier = Modifier.fillMaxSize()
            ) {
                StatusBar(
                    cameraFps = uiState.cameraFps,
                    imuRate = uiState.imuRate,
                    connectionStatus = uiState.connectionStatus,
                    isRecording = uiState.isRecording
                )

                // Control buttons (bottom layer)
                ControlButtons(
                    isRecording = uiState.isRecording,
                    isStreaming = uiState.isStreaming,
                    onRecordClick = { viewModel.toggleRecording() },
                    onStreamClick = { viewModel.toggleStreaming() },
                    onSettingsClick = onSettingsClick,
                    onCalibrationClick = onCalibrationClick,
                    onHistoryClick = onHistoryClick,
                    modifier = Modifier.fillMaxSize()
                )
            }
        }
    }
}
