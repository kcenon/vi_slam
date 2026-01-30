package com.vi.slam.android.ui

import android.app.Application
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.*
import org.junit.After
import org.junit.Before
import org.junit.Test
import org.junit.Assert.*
import org.mockito.Mockito.mock
import org.mockito.Mockito.`when`

/**
 * Unit tests for CalibrationViewModel.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class CalibrationViewModelTest {

    private lateinit var viewModel: CalibrationViewModel
    private lateinit var mockApplication: Application
    private val testDispatcher = StandardTestDispatcher()

    @Before
    fun setup() {
        Dispatchers.setMain(testDispatcher)
        mockApplication = mock(Application::class.java)
        `when`(mockApplication.applicationContext).thenReturn(mockApplication)
        viewModel = CalibrationViewModel(mockApplication)
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun `initial state should be INTRODUCTION step`() = runTest {
        advanceUntilIdle()
        val state = viewModel.uiState.value

        assertEquals(CalibrationStep.INTRODUCTION, state.currentStep)
        assertEquals(0.0f, state.progress, 0.01f)
        assertTrue(state.canProceed)
        assertFalse(state.isProcessing)
    }

    @Test
    fun `nextStep should progress from INTRODUCTION to CAMERA_SETUP`() = runTest {
        viewModel.nextStep()
        advanceUntilIdle()

        val state = viewModel.uiState.value
        assertEquals(CalibrationStep.CAMERA_SETUP, state.currentStep)
        assertEquals(1f / 7f, state.progress, 0.01f)
    }

    @Test
    fun `nextStep should progress through all steps`() = runTest {
        val expectedSteps = listOf(
            CalibrationStep.INTRODUCTION,
            CalibrationStep.CAMERA_SETUP,
            CalibrationStep.CHECKERBOARD_CAPTURE,
            CalibrationStep.CAMERA_PROCESSING,
            CalibrationStep.IMU_STATIC,
            CalibrationStep.IMU_MOTION,
            CalibrationStep.RESULTS
        )

        for (i in 0 until expectedSteps.size - 1) {
            assertEquals(expectedSteps[i], viewModel.uiState.value.currentStep)
            viewModel.nextStep()
            advanceUntilIdle()
        }

        assertEquals(CalibrationStep.RESULTS, viewModel.uiState.value.currentStep)
    }

    @Test
    fun `nextStep should not progress beyond RESULTS`() = runTest {
        // Navigate to RESULTS
        repeat(6) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        assertEquals(CalibrationStep.RESULTS, viewModel.uiState.value.currentStep)

        // Try to go beyond RESULTS
        viewModel.nextStep()
        advanceUntilIdle()

        assertEquals(CalibrationStep.RESULTS, viewModel.uiState.value.currentStep)
    }

    @Test
    fun `previousStep should navigate backwards`() = runTest {
        // Navigate to CAMERA_SETUP
        viewModel.nextStep()
        advanceUntilIdle()
        assertEquals(CalibrationStep.CAMERA_SETUP, viewModel.uiState.value.currentStep)

        // Go back
        viewModel.previousStep()
        advanceUntilIdle()

        assertEquals(CalibrationStep.INTRODUCTION, viewModel.uiState.value.currentStep)
    }

    @Test
    fun `previousStep should not go back from INTRODUCTION`() = runTest {
        assertEquals(CalibrationStep.INTRODUCTION, viewModel.uiState.value.currentStep)

        viewModel.previousStep()
        advanceUntilIdle()

        assertEquals(CalibrationStep.INTRODUCTION, viewModel.uiState.value.currentStep)
    }

    @Test
    fun `previousStep should cancel ongoing operations`() = runTest {
        // Navigate to CAMERA_PROCESSING
        repeat(3) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        // Start processing
        viewModel.startProcessing()
        assertTrue(viewModel.uiState.value.isProcessing)

        // Go back
        viewModel.previousStep()
        advanceUntilIdle()

        assertFalse(viewModel.uiState.value.isProcessing)
    }

    @Test
    fun `captureCheckerboard should increment capture count`() = runTest {
        // Navigate to CHECKERBOARD_CAPTURE
        repeat(2) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        val initialCount = viewModel.uiState.value.checkerboardCaptureCount
        assertEquals(0, initialCount)

        viewModel.captureCheckerboard()
        advanceUntilIdle()

        assertEquals(1, viewModel.uiState.value.checkerboardCaptureCount)
    }

    @Test
    fun `captureCheckerboard should enable proceed after minimum captures`() = runTest {
        // Navigate to CHECKERBOARD_CAPTURE
        repeat(2) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        assertFalse(viewModel.uiState.value.canProceed)

        // Capture minimum required (10)
        repeat(10) {
            viewModel.captureCheckerboard()
        }
        advanceUntilIdle()

        assertTrue(viewModel.uiState.value.canProceed)
    }

    @Test
    fun `startProcessing should process camera calibration`() = runTest {
        // Navigate to CAMERA_PROCESSING
        repeat(3) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        viewModel.startProcessing()
        assertTrue(viewModel.uiState.value.isProcessing)

        // Wait for processing to complete
        advanceUntilIdle()

        assertFalse(viewModel.uiState.value.isProcessing)
        assertTrue(viewModel.uiState.value.canProceed)
    }

    @Test
    fun `startProcessing should run IMU static calibration`() = runTest {
        // Navigate to IMU_STATIC
        repeat(4) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        viewModel.startProcessing()
        advanceTimeBy(100)

        assertTrue(viewModel.uiState.value.isProcessing)

        // Wait for completion (30 seconds + buffer)
        advanceTimeBy(31000)
        advanceUntilIdle()

        assertFalse(viewModel.uiState.value.isProcessing)
        assertEquals(0, viewModel.uiState.value.staticCalibrationRemaining)
        assertTrue(viewModel.uiState.value.canProceed)
    }

    @Test
    fun `startProcessing should run IMU motion calibration`() = runTest {
        // Navigate to IMU_MOTION
        repeat(5) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        viewModel.startProcessing()
        advanceTimeBy(100)

        assertTrue(viewModel.uiState.value.isProcessing)

        // Wait for completion (60 seconds + buffer)
        advanceTimeBy(61000)
        advanceUntilIdle()

        assertFalse(viewModel.uiState.value.isProcessing)
        assertEquals(0, viewModel.uiState.value.motionCalibrationRemaining)
        assertTrue(viewModel.uiState.value.canProceed)
        assertNotEquals(CalibrationQuality.POOR, viewModel.uiState.value.imuCalibrationQuality)
    }

    @Test
    fun `camera quality should be calculated after processing`() = runTest {
        // Navigate to CHECKERBOARD_CAPTURE
        repeat(2) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        // Capture 12 images
        repeat(12) {
            viewModel.captureCheckerboard()
        }
        advanceUntilIdle()

        // Navigate to processing
        viewModel.nextStep()
        advanceUntilIdle()

        // Process
        viewModel.startProcessing()
        advanceTimeBy(4000)
        advanceUntilIdle()

        val quality = viewModel.uiState.value.cameraCalibrationQuality

        // With 12 captures, should be at least GOOD quality
        assertTrue(quality == CalibrationQuality.GOOD || quality == CalibrationQuality.EXCELLENT)
    }

    @Test
    fun `retryCalibration should reset state`() = runTest {
        // Navigate to CAMERA_SETUP
        viewModel.nextStep()
        advanceUntilIdle()

        assertNotEquals(CalibrationStep.INTRODUCTION, viewModel.uiState.value.currentStep)

        viewModel.retryCalibration()

        assertEquals(CalibrationStep.INTRODUCTION, viewModel.uiState.value.currentStep)
        assertEquals(0, viewModel.uiState.value.checkerboardCaptureCount)
        assertFalse(viewModel.uiState.value.isProcessing)
    }

    @Test
    fun `saveCalibration should complete without error`() = runTest {
        // Navigate to RESULTS
        repeat(6) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        // Should not throw
        viewModel.saveCalibration()
        advanceUntilIdle()
    }

    @Test
    fun `progress should increase from 0 to 1`() = runTest {
        assertEquals(0.0f, viewModel.uiState.value.progress, 0.01f)

        // Navigate through all steps
        for (i in 1..6) {
            viewModel.nextStep()
            advanceUntilIdle()
        }

        assertEquals(1.0f, viewModel.uiState.value.progress, 0.01f)
    }

    @Test
    fun `CalibrationQuality enum should have correct labels`() {
        assertEquals("Excellent", CalibrationQuality.EXCELLENT.label)
        assertEquals("Good", CalibrationQuality.GOOD.label)
        assertEquals("Fair", CalibrationQuality.FAIR.label)
        assertEquals("Poor", CalibrationQuality.POOR.label)
    }

    @Test
    fun `CalibrationStep enum should have all required steps`() {
        val steps = CalibrationStep.values()

        assertEquals(7, steps.size)
        assertTrue(steps.contains(CalibrationStep.INTRODUCTION))
        assertTrue(steps.contains(CalibrationStep.CAMERA_SETUP))
        assertTrue(steps.contains(CalibrationStep.CHECKERBOARD_CAPTURE))
        assertTrue(steps.contains(CalibrationStep.CAMERA_PROCESSING))
        assertTrue(steps.contains(CalibrationStep.IMU_STATIC))
        assertTrue(steps.contains(CalibrationStep.IMU_MOTION))
        assertTrue(steps.contains(CalibrationStep.RESULTS))
    }
}
