package com.vi.slam.android.ui

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel

/**
 * Settings screen for configuring camera, server, and IMU parameters.
 *
 * @param viewModel ViewModel managing settings state
 * @param onBackClick Callback when back button is clicked
 * @param modifier Modifier for the screen container
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SettingsScreen(
    viewModel: SettingsViewModel = viewModel(),
    onBackClick: () -> Unit = {},
    modifier: Modifier = Modifier
) {
    val settings by viewModel.settings.collectAsState()
    var showResolutionDialog by remember { mutableStateOf(false) }
    var showFpsDialog by remember { mutableStateOf(false) }
    var showImuRateDialog by remember { mutableStateOf(false) }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Settings") },
                navigationIcon = {
                    IconButton(onClick = onBackClick) {
                        Icon(
                            imageVector = Icons.Filled.ArrowBack,
                            contentDescription = "Back"
                        )
                    }
                }
            )
        }
    ) { paddingValues ->
        Column(
            modifier = modifier
                .fillMaxSize()
                .padding(paddingValues)
                .verticalScroll(rememberScrollState())
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            // Camera section
            Text(
                text = "Camera",
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.primary
            )
            Divider()
            Spacer(modifier = Modifier.height(8.dp))

            // Resolution setting
            SettingsItem(
                title = "Resolution",
                subtitle = settings.resolutionString,
                onClick = { showResolutionDialog = true }
            )

            // FPS setting
            SettingsItem(
                title = "FPS",
                subtitle = "${settings.fps} fps",
                onClick = { showFpsDialog = true }
            )

            Spacer(modifier = Modifier.height(16.dp))

            // Server section
            Text(
                text = "Server Connection",
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.primary
            )
            Divider()
            Spacer(modifier = Modifier.height(8.dp))

            // Server IP setting
            IpAddressField(
                value = settings.serverIp,
                onValueChange = { viewModel.updateServerIp(it) }
            )

            // Server port setting
            PortField(
                value = settings.serverPort.toString(),
                onValueChange = { port ->
                    port.toIntOrNull()?.let { viewModel.updateServerPort(it) }
                }
            )

            Spacer(modifier = Modifier.height(16.dp))

            // IMU section
            Text(
                text = "IMU (Inertial Measurement Unit)",
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.primary
            )
            Divider()
            Spacer(modifier = Modifier.height(8.dp))

            // Enable IMU switch
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Column(modifier = Modifier.weight(1f)) {
                    Text(
                        text = "Enable IMU",
                        style = MaterialTheme.typography.bodyLarge
                    )
                    Text(
                        text = "Record accelerometer and gyroscope data",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant
                    )
                }
                Switch(
                    checked = settings.enableImu,
                    onCheckedChange = { viewModel.updateEnableImu(it) }
                )
            }

            // IMU rate setting
            if (settings.enableImu) {
                SettingsItem(
                    title = "Sample Rate",
                    subtitle = "${settings.imuRate} Hz",
                    onClick = { showImuRateDialog = true }
                )
            }
        }
    }

    // Resolution selection dialog
    if (showResolutionDialog) {
        AlertDialog(
            onDismissRequest = { showResolutionDialog = false },
            title = { Text("Select Resolution") },
            text = {
                Column {
                    SettingsViewModel.SUPPORTED_RESOLUTIONS.forEach { resolution ->
                        val isSelected = resolution.width == settings.resolutionWidth &&
                                resolution.height == settings.resolutionHeight
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable {
                                    viewModel.updateResolution(resolution.width, resolution.height)
                                    showResolutionDialog = false
                                }
                                .padding(vertical = 12.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            RadioButton(
                                selected = isSelected,
                                onClick = {
                                    viewModel.updateResolution(resolution.width, resolution.height)
                                    showResolutionDialog = false
                                }
                            )
                            Spacer(modifier = Modifier.width(8.dp))
                            Text(resolution.toString())
                        }
                    }
                }
            },
            confirmButton = {
                TextButton(onClick = { showResolutionDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }

    // FPS selection dialog
    if (showFpsDialog) {
        AlertDialog(
            onDismissRequest = { showFpsDialog = false },
            title = { Text("Select FPS") },
            text = {
                Column {
                    SettingsViewModel.SUPPORTED_FPS.forEach { fps ->
                        val isSelected = fps == settings.fps
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable {
                                    viewModel.updateFps(fps)
                                    showFpsDialog = false
                                }
                                .padding(vertical = 12.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            RadioButton(
                                selected = isSelected,
                                onClick = {
                                    viewModel.updateFps(fps)
                                    showFpsDialog = false
                                }
                            )
                            Spacer(modifier = Modifier.width(8.dp))
                            Text("$fps fps")
                        }
                    }
                }
            },
            confirmButton = {
                TextButton(onClick = { showFpsDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }

    // IMU rate selection dialog
    if (showImuRateDialog) {
        AlertDialog(
            onDismissRequest = { showImuRateDialog = false },
            title = { Text("Select IMU Sample Rate") },
            text = {
                Column {
                    SettingsViewModel.SUPPORTED_IMU_RATES.forEach { rate ->
                        val isSelected = rate == settings.imuRate
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable {
                                    viewModel.updateImuRate(rate)
                                    showImuRateDialog = false
                                }
                                .padding(vertical = 12.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            RadioButton(
                                selected = isSelected,
                                onClick = {
                                    viewModel.updateImuRate(rate)
                                    showImuRateDialog = false
                                }
                            )
                            Spacer(modifier = Modifier.width(8.dp))
                            Text("$rate Hz")
                        }
                    }
                }
            },
            confirmButton = {
                TextButton(onClick = { showImuRateDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }
}

/**
 * Reusable settings item with title and subtitle.
 */
@Composable
private fun SettingsItem(
    title: String,
    subtitle: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Surface(
        onClick = onClick,
        modifier = modifier.fillMaxWidth()
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 12.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column {
                Text(
                    text = title,
                    style = MaterialTheme.typography.bodyLarge
                )
                Text(
                    text = subtitle,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
        }
    }
}

/**
 * IP address input field.
 */
@Composable
private fun IpAddressField(
    value: String,
    onValueChange: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text("Server IP Address") },
        placeholder = { Text("192.168.1.100") },
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
        singleLine = true,
        modifier = modifier.fillMaxWidth()
    )
}

/**
 * Port number input field.
 */
@Composable
private fun PortField(
    value: String,
    onValueChange: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text("Server Port") },
        placeholder = { Text("8080") },
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
        singleLine = true,
        modifier = modifier.fillMaxWidth()
    )
}
