package com.vi.slam.android.ui

import android.app.Application
import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.preferencesDataStore
import androidx.test.core.app.ApplicationProvider
import com.vi.slam.android.data.SettingsRepository
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.test.*
import org.junit.After
import org.junit.Before
import org.junit.Test
import org.junit.Assert.*
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

/**
 * Unit tests for SettingsViewModel.
 */
@OptIn(ExperimentalCoroutinesApi::class)
@RunWith(RobolectricTestRunner::class)
class SettingsViewModelTest {

    private lateinit var viewModel: SettingsViewModel
    private lateinit var testApplication: Application
    private val testDispatcher = StandardTestDispatcher()

    @Before
    fun setup() {
        Dispatchers.setMain(testDispatcher)
        testApplication = ApplicationProvider.getApplicationContext()
        viewModel = SettingsViewModel(testApplication)
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun `initial settings should have default values`() = runTest {
        advanceUntilIdle()
        val settings = viewModel.settings.value

        assertEquals(SettingsRepository.DEFAULT_RESOLUTION_WIDTH, settings.resolutionWidth)
        assertEquals(SettingsRepository.DEFAULT_RESOLUTION_HEIGHT, settings.resolutionHeight)
        assertEquals(SettingsRepository.DEFAULT_FPS, settings.fps)
        assertEquals(SettingsRepository.DEFAULT_SERVER_IP, settings.serverIp)
        assertEquals(SettingsRepository.DEFAULT_SERVER_PORT, settings.serverPort)
        assertEquals(SettingsRepository.DEFAULT_ENABLE_IMU, settings.enableImu)
        assertEquals(SettingsRepository.DEFAULT_IMU_RATE, settings.imuRate)
    }

    @Test
    fun `updateResolution should update resolution width and height`() = runTest {
        viewModel.updateResolution(1280, 720)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals(1280, settings.resolutionWidth)
        assertEquals(720, settings.resolutionHeight)
    }

    @Test
    fun `updateFps should update FPS value`() = runTest {
        viewModel.updateFps(60)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals(60, settings.fps)
    }

    @Test
    fun `updateServerIp should update server IP`() = runTest {
        viewModel.updateServerIp("10.0.0.1")
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals("10.0.0.1", settings.serverIp)
    }

    @Test
    fun `updateServerPort should update valid port`() = runTest {
        viewModel.updateServerPort(9090)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals(9090, settings.serverPort)
    }

    @Test
    fun `updateServerPort should reject port below 1`() = runTest {
        val originalPort = viewModel.settings.value.serverPort
        viewModel.updateServerPort(0)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals(originalPort, settings.serverPort)
    }

    @Test
    fun `updateServerPort should reject port above 65535`() = runTest {
        val originalPort = viewModel.settings.value.serverPort
        viewModel.updateServerPort(65536)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals(originalPort, settings.serverPort)
    }

    @Test
    fun `updateEnableImu should update IMU enabled state`() = runTest {
        viewModel.updateEnableImu(false)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertFalse(settings.enableImu)
    }

    @Test
    fun `updateImuRate should update IMU rate`() = runTest {
        viewModel.updateImuRate(400)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals(400, settings.imuRate)
    }

    @Test
    fun `SUPPORTED_RESOLUTIONS should contain common resolutions`() {
        val resolutions = SettingsViewModel.SUPPORTED_RESOLUTIONS

        assertTrue(resolutions.any { it.width == 640 && it.height == 480 })
        assertTrue(resolutions.any { it.width == 1280 && it.height == 720 })
        assertTrue(resolutions.any { it.width == 1920 && it.height == 1080 })
        assertTrue(resolutions.any { it.width == 3840 && it.height == 2160 })
    }

    @Test
    fun `SUPPORTED_FPS should contain common frame rates`() {
        val fps = SettingsViewModel.SUPPORTED_FPS

        assertTrue(fps.contains(15))
        assertTrue(fps.contains(24))
        assertTrue(fps.contains(30))
        assertTrue(fps.contains(60))
    }

    @Test
    fun `SUPPORTED_IMU_RATES should contain common IMU rates`() {
        val rates = SettingsViewModel.SUPPORTED_IMU_RATES

        assertTrue(rates.contains(100))
        assertTrue(rates.contains(200))
        assertTrue(rates.contains(400))
    }

    @Test
    fun `Resolution toString should format correctly`() {
        val resolution = Resolution(1920, 1080, "Full HD")

        assertEquals("Full HD (1920×1080)", resolution.toString())
    }

    @Test
    fun `AppSettings resolutionString should format correctly`() = runTest {
        viewModel.updateResolution(1280, 720)
        advanceUntilIdle()

        val settings = viewModel.settings.value
        assertEquals("1280x720", settings.resolutionString)
    }
}
