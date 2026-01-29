package com.vi.slam.android.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.Canvas
import androidx.compose.material.icons.Icons
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.geometry.Offset

/**
 * Status bar showing camera FPS, IMU rate, connection status, and recording indicator.
 *
 * @param cameraFps Camera frames per second
 * @param imuRate IMU sample rate (Hz)
 * @param connectionStatus Streaming connection status
 * @param isRecording Whether recording is active
 * @param modifier Modifier for the status bar
 */
@Composable
fun StatusBar(
    cameraFps: Int,
    imuRate: Int,
    connectionStatus: ConnectionStatus,
    isRecording: Boolean,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier
            .fillMaxWidth()
            .background(MaterialTheme.colorScheme.surface.copy(alpha = 0.9f)),
        color = MaterialTheme.colorScheme.surface.copy(alpha = 0.9f)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 16.dp, vertical = 8.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Left side: FPS and IMU rate
            Row(
                horizontalArrangement = Arrangement.spacedBy(16.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                StatusItem(
                    label = "FPS",
                    value = cameraFps.toString()
                )

                StatusItem(
                    label = "IMU",
                    value = "${imuRate}Hz"
                )
            }

            // Right side: Connection and Recording indicators
            Row(
                horizontalArrangement = Arrangement.spacedBy(12.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                // Connection status indicator
                ConnectionIndicator(connectionStatus = connectionStatus)

                // Recording indicator
                if (isRecording) {
                    RecordingIndicator()
                }
            }
        }
    }
}

/**
 * Single status item showing label and value.
 */
@Composable
private fun StatusItem(
    label: String,
    value: String,
    modifier: Modifier = Modifier
) {
    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(4.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = "$label:",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.6f)
        )
        Text(
            text = value,
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurface
        )
    }
}

/**
 * Connection status indicator with colored dot.
 */
@Composable
private fun ConnectionIndicator(
    connectionStatus: ConnectionStatus,
    modifier: Modifier = Modifier
) {
    val (color, text) = when (connectionStatus) {
        ConnectionStatus.DISCONNECTED -> Color.Gray to "Offline"
        ConnectionStatus.CONNECTING -> Color.Yellow to "Connecting"
        ConnectionStatus.CONNECTED -> Color.Green to "Online"
        ConnectionStatus.ERROR -> Color.Red to "Error"
    }

    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(4.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        // Connection status indicator dot
        Canvas(modifier = Modifier.size(12.dp)) {
            drawCircle(
                color = color,
                radius = size.minDimension / 2,
                center = Offset(size.width / 2, size.height / 2)
            )
        }
        Text(
            text = text,
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurface
        )
    }
}

/**
 * Recording indicator with blinking red dot.
 */
@Composable
private fun RecordingIndicator(
    modifier: Modifier = Modifier
) {
    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(4.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        // Recording indicator dot
        Canvas(modifier = Modifier.size(12.dp)) {
            drawCircle(
                color = Color.Red,
                radius = size.minDimension / 2,
                center = Offset(size.width / 2, size.height / 2)
            )
        }
        Text(
            text = "REC",
            style = MaterialTheme.typography.bodySmall,
            color = Color.Red
        )
    }
}
