package com.vi.slam.android.ui

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.Canvas
import androidx.compose.ui.geometry.Offset

/**
 * Control buttons for recording and streaming.
 *
 * @param isRecording Whether recording is active
 * @param isStreaming Whether streaming is active
 * @param onRecordClick Callback when record button is clicked
 * @param onStreamClick Callback when stream button is clicked
 * @param onSettingsClick Callback when settings button is clicked
 * @param onCalibrationClick Callback when calibration button is clicked
 * @param onHistoryClick Callback when history button is clicked
 * @param modifier Modifier for the button container
 */
@Composable
fun ControlButtons(
    isRecording: Boolean,
    isStreaming: Boolean,
    onRecordClick: () -> Unit,
    onStreamClick: () -> Unit,
    onSettingsClick: () -> Unit,
    onCalibrationClick: () -> Unit,
    onHistoryClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // Top row: Settings, Calibration, and History buttons
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.End,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // History button
            IconButton(onClick = onHistoryClick) {
                Icon(
                    imageVector = Icons.Default.List,
                    contentDescription = "History",
                    tint = MaterialTheme.colorScheme.onSurface
                )
            }

            Spacer(modifier = Modifier.width(8.dp))

            // Calibration button
            IconButton(onClick = onCalibrationClick) {
                Icon(
                    imageVector = Icons.Default.Build,
                    contentDescription = "Calibration",
                    tint = MaterialTheme.colorScheme.onSurface
                )
            }

            Spacer(modifier = Modifier.width(8.dp))

            // Settings button
            IconButton(onClick = onSettingsClick) {
                Icon(
                    imageVector = Icons.Default.Settings,
                    contentDescription = "Settings",
                    tint = MaterialTheme.colorScheme.onSurface
                )
            }
        }

        Spacer(modifier = Modifier.weight(1f))

        // Bottom row: Record and Stream buttons
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceEvenly,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Record button
            RecordButton(
                isRecording = isRecording,
                onClick = onRecordClick
            )

            // Stream button
            StreamButton(
                isStreaming = isStreaming,
                onClick = onStreamClick
            )
        }
    }
}

/**
 * Record button with toggle state.
 */
@Composable
private fun RecordButton(
    isRecording: Boolean,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    FloatingActionButton(
        onClick = onClick,
        modifier = modifier.size(72.dp),
        shape = CircleShape,
        containerColor = if (isRecording) Color.Red else MaterialTheme.colorScheme.error,
        contentColor = Color.White
    ) {
        if (isRecording) {
            // Stop icon: Square (using Canvas)
            Canvas(modifier = Modifier.size(36.dp)) {
                drawRect(
                    color = Color.White,
                    topLeft = Offset(size.width * 0.25f, size.height * 0.25f),
                    size = androidx.compose.ui.geometry.Size(
                        size.width * 0.5f,
                        size.height * 0.5f
                    )
                )
            }
        } else {
            // Record icon: Circle (using Canvas)
            Canvas(modifier = Modifier.size(36.dp)) {
                drawCircle(
                    color = Color.White,
                    radius = size.minDimension / 2,
                    center = Offset(size.width / 2, size.height / 2)
                )
            }
        }
    }
}

/**
 * Stream button with toggle state.
 */
@Composable
private fun StreamButton(
    isStreaming: Boolean,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    FloatingActionButton(
        onClick = onClick,
        modifier = modifier.size(72.dp),
        shape = CircleShape,
        containerColor = if (isStreaming) Color.Blue else MaterialTheme.colorScheme.primary,
        contentColor = Color.White
    ) {
        Icon(
            imageVector = if (isStreaming) Icons.Default.Close else Icons.Default.Send,
            contentDescription = if (isStreaming) "Stop Streaming" else "Start Streaming",
            modifier = Modifier.size(36.dp)
        )
    }
}
