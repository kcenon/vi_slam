package com.vi.slam.android.ui

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel

/**
 * Calibration screen with step-by-step guided flow for camera and IMU calibration.
 *
 * @param viewModel ViewModel managing calibration state
 * @param onBackClick Callback when back button is clicked
 * @param modifier Modifier for the screen container
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun CalibrationScreen(
    viewModel: CalibrationViewModel = viewModel(),
    onBackClick: () -> Unit = {},
    modifier: Modifier = Modifier
) {
    val uiState by viewModel.uiState.collectAsState()

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Calibration") },
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
        ) {
            // Progress indicator
            LinearProgressIndicator(
                progress = uiState.progress,
                modifier = Modifier.fillMaxWidth()
            )

            // Step content
            Column(
                modifier = Modifier
                    .weight(1f)
                    .verticalScroll(rememberScrollState())
                    .padding(16.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                when (uiState.currentStep) {
                    CalibrationStep.INTRODUCTION -> IntroductionStep()
                    CalibrationStep.CAMERA_SETUP -> CameraSetupStep()
                    CalibrationStep.CHECKERBOARD_CAPTURE -> CheckerboardCaptureStep(
                        captureCount = uiState.checkerboardCaptureCount
                    )
                    CalibrationStep.CAMERA_PROCESSING -> CameraProcessingStep(
                        isProcessing = uiState.isProcessing
                    )
                    CalibrationStep.IMU_STATIC -> ImuStaticStep(
                        remainingSeconds = uiState.staticCalibrationRemaining
                    )
                    CalibrationStep.IMU_MOTION -> ImuMotionStep(
                        remainingSeconds = uiState.motionCalibrationRemaining
                    )
                    CalibrationStep.RESULTS -> ResultsStep(
                        cameraQuality = uiState.cameraCalibrationQuality,
                        imuQuality = uiState.imuCalibrationQuality
                    )
                }
            }

            // Action buttons
            CalibrationActionButtons(
                currentStep = uiState.currentStep,
                canGoNext = uiState.canProceed,
                isProcessing = uiState.isProcessing,
                onPreviousClick = { viewModel.previousStep() },
                onNextClick = { viewModel.nextStep() },
                onCaptureClick = { viewModel.captureCheckerboard() },
                onStartClick = { viewModel.startProcessing() },
                onRetryClick = { viewModel.retryCalibration() },
                onSaveClick = { viewModel.saveCalibration() },
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp)
            )
        }
    }
}

/**
 * Introduction step explaining calibration purpose.
 */
@Composable
private fun IntroductionStep() {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Welcome to Calibration",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "Camera and IMU calibration are critical for accurate SLAM performance. " +
                    "This wizard will guide you through the calibration process step by step.",
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.height(16.dp))
        Text(
            text = "What you'll need:",
            style = MaterialTheme.typography.titleMedium
        )
        Text(
            text = "• Printed checkerboard pattern\n" +
                    "• Well-lit environment\n" +
                    "• Stable surface for device\n" +
                    "• 5-10 minutes of time",
            style = MaterialTheme.typography.bodyMedium
        )
    }
}

/**
 * Camera setup step with instructions.
 */
@Composable
private fun CameraSetupStep() {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Camera Setup",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "Prepare your device and environment for calibration:",
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.height(8.dp))
        Card(
            modifier = Modifier.fillMaxWidth()
        ) {
            Column(
                modifier = Modifier.padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                Text(
                    text = "1. Place the device on a stable surface",
                    style = MaterialTheme.typography.bodyMedium
                )
                Text(
                    text = "2. Ensure good lighting (avoid direct sunlight)",
                    style = MaterialTheme.typography.bodyMedium
                )
                Text(
                    text = "3. Have the checkerboard pattern ready",
                    style = MaterialTheme.typography.bodyMedium
                )
                Text(
                    text = "4. Clear the area of obstructions",
                    style = MaterialTheme.typography.bodyMedium
                )
            }
        }
    }
}

/**
 * Checkerboard capture step with camera preview.
 */
@Composable
private fun CheckerboardCaptureStep(captureCount: Int) {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Capture Checkerboard",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "Capture the checkerboard from multiple angles. " +
                    "Aim for at least 10 captures from different positions.",
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.height(8.dp))

        // Camera preview placeholder
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .aspectRatio(4f / 3f)
        ) {
            Box(
                modifier = Modifier.fillMaxSize(),
                contentAlignment = Alignment.Center
            ) {
                Text("Camera Preview")
            }
        }

        // Capture count
        Text(
            text = "Captures: $captureCount / 10",
            style = MaterialTheme.typography.titleLarge,
            color = MaterialTheme.colorScheme.primary
        )

        Text(
            text = "Tips:\n" +
                    "• Keep the entire checkerboard visible\n" +
                    "• Vary the distance and angle\n" +
                    "• Hold steady while capturing",
            style = MaterialTheme.typography.bodySmall
        )
    }
}

/**
 * Camera processing step showing progress.
 */
@Composable
private fun CameraProcessingStep(isProcessing: Boolean) {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Processing Camera Calibration",
            style = MaterialTheme.typography.headlineMedium
        )
        Spacer(modifier = Modifier.height(16.dp))

        if (isProcessing) {
            CircularProgressIndicator(modifier = Modifier.size(64.dp))
            Spacer(modifier = Modifier.height(16.dp))
            Text(
                text = "Analyzing captured images...\nThis may take 30-60 seconds.",
                style = MaterialTheme.typography.bodyLarge,
                textAlign = TextAlign.Center
            )
        } else {
            Text(
                text = "Ready to process calibration data",
                style = MaterialTheme.typography.bodyLarge,
                textAlign = TextAlign.Center
            )
        }
    }
}

/**
 * IMU static calibration step.
 */
@Composable
private fun ImuStaticStep(remainingSeconds: Int) {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "IMU Static Calibration",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "Place the device on a stable surface and keep it completely still.",
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.height(16.dp))

        if (remainingSeconds > 0) {
            Text(
                text = "$remainingSeconds",
                style = MaterialTheme.typography.displayLarge,
                color = MaterialTheme.colorScheme.primary
            )
            Text(
                text = "seconds remaining",
                style = MaterialTheme.typography.bodyMedium
            )
        } else {
            Text(
                text = "Ready to start",
                style = MaterialTheme.typography.titleLarge
            )
        }
    }
}

/**
 * IMU motion calibration step.
 */
@Composable
private fun ImuMotionStep(remainingSeconds: Int) {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "IMU Motion Calibration",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "Follow the motion instructions to calibrate the IMU.",
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.height(8.dp))

        Card(
            modifier = Modifier.fillMaxWidth()
        ) {
            Column(
                modifier = Modifier.padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                Text(
                    text = "1. Slowly rotate the device around X-axis",
                    style = MaterialTheme.typography.bodyMedium
                )
                Text(
                    text = "2. Slowly rotate the device around Y-axis",
                    style = MaterialTheme.typography.bodyMedium
                )
                Text(
                    text = "3. Slowly rotate the device around Z-axis",
                    style = MaterialTheme.typography.bodyMedium
                )
            }
        }

        if (remainingSeconds > 0) {
            Spacer(modifier = Modifier.height(8.dp))
            Text(
                text = "$remainingSeconds",
                style = MaterialTheme.typography.displayLarge,
                color = MaterialTheme.colorScheme.primary
            )
            Text(
                text = "seconds remaining",
                style = MaterialTheme.typography.bodyMedium
            )
        }
    }
}

/**
 * Results step showing calibration quality.
 */
@Composable
private fun ResultsStep(
    cameraQuality: CalibrationQuality,
    imuQuality: CalibrationQuality
) {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Calibration Complete",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "Review the calibration results below:",
            style = MaterialTheme.typography.bodyLarge,
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.height(8.dp))

        // Camera calibration result
        CalibrationResultCard(
            title = "Camera Calibration",
            quality = cameraQuality
        )

        // IMU calibration result
        CalibrationResultCard(
            title = "IMU Calibration",
            quality = imuQuality
        )

        if (cameraQuality == CalibrationQuality.POOR || imuQuality == CalibrationQuality.POOR) {
            Card(
                colors = CardDefaults.cardColors(
                    containerColor = MaterialTheme.colorScheme.errorContainer
                )
            ) {
                Text(
                    text = "Poor calibration quality detected. Consider retrying for better results.",
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.padding(16.dp)
                )
            }
        }
    }
}

/**
 * Calibration result card showing quality.
 */
@Composable
private fun CalibrationResultCard(
    title: String,
    quality: CalibrationQuality
) {
    Card(
        modifier = Modifier.fillMaxWidth()
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = title,
                style = MaterialTheme.typography.titleMedium
            )
            Spacer(modifier = Modifier.height(8.dp))
            Text(
                text = quality.label,
                style = MaterialTheme.typography.headlineSmall,
                color = when (quality) {
                    CalibrationQuality.EXCELLENT -> MaterialTheme.colorScheme.primary
                    CalibrationQuality.GOOD -> MaterialTheme.colorScheme.tertiary
                    CalibrationQuality.FAIR -> MaterialTheme.colorScheme.secondary
                    CalibrationQuality.POOR -> MaterialTheme.colorScheme.error
                }
            )
        }
    }
}

/**
 * Action buttons for calibration steps.
 */
@Composable
private fun CalibrationActionButtons(
    currentStep: CalibrationStep,
    canGoNext: Boolean,
    isProcessing: Boolean,
    onPreviousClick: () -> Unit,
    onNextClick: () -> Unit,
    onCaptureClick: () -> Unit,
    onStartClick: () -> Unit,
    onRetryClick: () -> Unit,
    onSaveClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        // Previous button
        if (currentStep != CalibrationStep.INTRODUCTION && currentStep != CalibrationStep.RESULTS) {
            OutlinedButton(
                onClick = onPreviousClick,
                enabled = !isProcessing,
                modifier = Modifier.weight(1f)
            ) {
                Text("Previous")
            }
        }

        // Step-specific action buttons
        when (currentStep) {
            CalibrationStep.CHECKERBOARD_CAPTURE -> {
                Button(
                    onClick = onCaptureClick,
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Capture")
                }
            }
            CalibrationStep.CAMERA_PROCESSING -> {
                Button(
                    onClick = onStartClick,
                    enabled = !isProcessing,
                    modifier = Modifier.weight(1f)
                ) {
                    Text(if (isProcessing) "Processing..." else "Start Processing")
                }
            }
            CalibrationStep.IMU_STATIC, CalibrationStep.IMU_MOTION -> {
                Button(
                    onClick = onStartClick,
                    enabled = !isProcessing,
                    modifier = Modifier.weight(1f)
                ) {
                    Text(if (isProcessing) "In Progress..." else "Start")
                }
            }
            CalibrationStep.RESULTS -> {
                OutlinedButton(
                    onClick = onRetryClick,
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Retry")
                }
                Button(
                    onClick = onSaveClick,
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Save & Finish")
                }
            }
            else -> {
                // Next button for other steps
                Button(
                    onClick = onNextClick,
                    enabled = canGoNext && !isProcessing,
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Next")
                }
            }
        }
    }
}
