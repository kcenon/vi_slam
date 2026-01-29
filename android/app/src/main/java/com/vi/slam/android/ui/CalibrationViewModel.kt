package com.vi.slam.android.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.vi.slam.android.data.CalibrationRepository
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

/**
 * ViewModel for calibration screen.
 * Manages calibration state machine and user interactions.
 */
class CalibrationViewModel(application: Application) : AndroidViewModel(application) {

    private val repository = CalibrationRepository(application)

    private val _uiState = MutableStateFlow(CalibrationUiState())
    val uiState: StateFlow<CalibrationUiState> = _uiState.asStateFlow()

    private var countdownJob: Job? = null

    /**
     * Move to the next calibration step.
     */
    fun nextStep() {
        val currentStep = _uiState.value.currentStep
        val nextStep = when (currentStep) {
            CalibrationStep.INTRODUCTION -> CalibrationStep.CAMERA_SETUP
            CalibrationStep.CAMERA_SETUP -> CalibrationStep.CHECKERBOARD_CAPTURE
            CalibrationStep.CHECKERBOARD_CAPTURE -> CalibrationStep.CAMERA_PROCESSING
            CalibrationStep.CAMERA_PROCESSING -> CalibrationStep.IMU_STATIC
            CalibrationStep.IMU_STATIC -> CalibrationStep.IMU_MOTION
            CalibrationStep.IMU_MOTION -> CalibrationStep.RESULTS
            CalibrationStep.RESULTS -> return
        }

        _uiState.update { state ->
            state.copy(
                currentStep = nextStep,
                progress = calculateProgress(nextStep),
                canProceed = canProceedToNextStep(nextStep)
            )
        }
    }

    /**
     * Move to the previous calibration step.
     */
    fun previousStep() {
        val currentStep = _uiState.value.currentStep
        val previousStep = when (currentStep) {
            CalibrationStep.INTRODUCTION -> return
            CalibrationStep.CAMERA_SETUP -> CalibrationStep.INTRODUCTION
            CalibrationStep.CHECKERBOARD_CAPTURE -> CalibrationStep.CAMERA_SETUP
            CalibrationStep.CAMERA_PROCESSING -> CalibrationStep.CHECKERBOARD_CAPTURE
            CalibrationStep.IMU_STATIC -> CalibrationStep.CAMERA_PROCESSING
            CalibrationStep.IMU_MOTION -> CalibrationStep.IMU_STATIC
            CalibrationStep.RESULTS -> CalibrationStep.IMU_MOTION
        }

        // Cancel any ongoing operations
        countdownJob?.cancel()

        _uiState.update { state ->
            state.copy(
                currentStep = previousStep,
                progress = calculateProgress(previousStep),
                isProcessing = false,
                canProceed = true
            )
        }
    }

    /**
     * Capture checkerboard image for camera calibration.
     */
    fun captureCheckerboard() {
        val currentCount = _uiState.value.checkerboardCaptureCount
        _uiState.update { state ->
            state.copy(
                checkerboardCaptureCount = currentCount + 1,
                canProceed = currentCount + 1 >= MINIMUM_CAPTURES
            )
        }
    }

    /**
     * Start processing calibration data.
     */
    fun startProcessing() {
        val currentStep = _uiState.value.currentStep

        when (currentStep) {
            CalibrationStep.CAMERA_PROCESSING -> startCameraProcessing()
            CalibrationStep.IMU_STATIC -> startStaticCalibration()
            CalibrationStep.IMU_MOTION -> startMotionCalibration()
            else -> return
        }
    }

    /**
     * Retry calibration from the beginning.
     */
    fun retryCalibration() {
        countdownJob?.cancel()
        _uiState.value = CalibrationUiState()
    }

    /**
     * Save calibration data and finish.
     */
    fun saveCalibration() {
        viewModelScope.launch {
            val state = _uiState.value
            repository.saveCalibration(
                cameraQuality = state.cameraCalibrationQuality,
                imuQuality = state.imuCalibrationQuality
            )
        }
    }

    /**
     * Start camera calibration processing.
     */
    private fun startCameraProcessing() {
        _uiState.update { it.copy(isProcessing = true, canProceed = false) }

        viewModelScope.launch {
            // Simulate processing time
            delay(3000)

            // Simulate calibration quality calculation
            val quality = calculateCameraQuality()

            _uiState.update { state ->
                state.copy(
                    isProcessing = false,
                    cameraCalibrationQuality = quality,
                    canProceed = true
                )
            }
        }
    }

    /**
     * Start IMU static calibration.
     */
    private fun startStaticCalibration() {
        _uiState.update { it.copy(isProcessing = true, canProceed = false) }

        countdownJob = viewModelScope.launch {
            var remaining = STATIC_CALIBRATION_DURATION
            while (remaining > 0) {
                _uiState.update { it.copy(staticCalibrationRemaining = remaining) }
                delay(1000)
                remaining--
            }

            _uiState.update { state ->
                state.copy(
                    isProcessing = false,
                    staticCalibrationRemaining = 0,
                    canProceed = true
                )
            }
        }
    }

    /**
     * Start IMU motion calibration.
     */
    private fun startMotionCalibration() {
        _uiState.update { it.copy(isProcessing = true, canProceed = false) }

        countdownJob = viewModelScope.launch {
            var remaining = MOTION_CALIBRATION_DURATION
            while (remaining > 0) {
                _uiState.update { it.copy(motionCalibrationRemaining = remaining) }
                delay(1000)
                remaining--
            }

            // Simulate IMU quality calculation
            val quality = calculateImuQuality()

            _uiState.update { state ->
                state.copy(
                    isProcessing = false,
                    motionCalibrationRemaining = 0,
                    imuCalibrationQuality = quality,
                    canProceed = true
                )
            }
        }
    }

    /**
     * Calculate progress based on current step.
     */
    private fun calculateProgress(step: CalibrationStep): Float {
        return when (step) {
            CalibrationStep.INTRODUCTION -> 0.0f
            CalibrationStep.CAMERA_SETUP -> 1f / 7f
            CalibrationStep.CHECKERBOARD_CAPTURE -> 2f / 7f
            CalibrationStep.CAMERA_PROCESSING -> 3f / 7f
            CalibrationStep.IMU_STATIC -> 4f / 7f
            CalibrationStep.IMU_MOTION -> 5f / 7f
            CalibrationStep.RESULTS -> 1.0f
        }
    }

    /**
     * Check if can proceed to the next step.
     */
    private fun canProceedToNextStep(step: CalibrationStep): Boolean {
        return when (step) {
            CalibrationStep.CHECKERBOARD_CAPTURE -> _uiState.value.checkerboardCaptureCount >= MINIMUM_CAPTURES
            else -> true
        }
    }

    /**
     * Calculate camera calibration quality.
     * In production, this would analyze actual calibration data.
     */
    private fun calculateCameraQuality(): CalibrationQuality {
        val captureCount = _uiState.value.checkerboardCaptureCount
        return when {
            captureCount >= 15 -> CalibrationQuality.EXCELLENT
            captureCount >= 12 -> CalibrationQuality.GOOD
            captureCount >= 10 -> CalibrationQuality.FAIR
            else -> CalibrationQuality.POOR
        }
    }

    /**
     * Calculate IMU calibration quality.
     * In production, this would analyze actual IMU data.
     */
    private fun calculateImuQuality(): CalibrationQuality {
        // Simulate quality calculation
        return CalibrationQuality.GOOD
    }

    override fun onCleared() {
        super.onCleared()
        countdownJob?.cancel()
    }

    companion object {
        private const val MINIMUM_CAPTURES = 10
        private const val STATIC_CALIBRATION_DURATION = 30
        private const val MOTION_CALIBRATION_DURATION = 60
    }
}

/**
 * UI state for calibration screen.
 */
data class CalibrationUiState(
    val currentStep: CalibrationStep = CalibrationStep.INTRODUCTION,
    val progress: Float = 0.0f,
    val canProceed: Boolean = true,
    val isProcessing: Boolean = false,
    val checkerboardCaptureCount: Int = 0,
    val staticCalibrationRemaining: Int = 0,
    val motionCalibrationRemaining: Int = 0,
    val cameraCalibrationQuality: CalibrationQuality = CalibrationQuality.POOR,
    val imuCalibrationQuality: CalibrationQuality = CalibrationQuality.POOR
)

/**
 * Calibration steps enum.
 */
enum class CalibrationStep {
    INTRODUCTION,
    CAMERA_SETUP,
    CHECKERBOARD_CAPTURE,
    CAMERA_PROCESSING,
    IMU_STATIC,
    IMU_MOTION,
    RESULTS
}

/**
 * Calibration quality levels.
 */
enum class CalibrationQuality(val label: String) {
    EXCELLENT("Excellent"),
    GOOD("Good"),
    FAIR("Fair"),
    POOR("Poor")
}
