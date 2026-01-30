package com.vi.slam.android.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.vi.slam.android.data.AppSettings
import com.vi.slam.android.data.SettingsRepository
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

/**
 * ViewModel for the settings screen.
 * Manages application settings and their persistence.
 *
 * @param application Application context
 * @param sharingStarted Sharing strategy for the StateFlow (defaults to WhileSubscribed for production)
 * @param dataStoreName Name for the DataStore instance (defaults to "settings")
 */
class SettingsViewModel(
    application: Application,
    sharingStarted: SharingStarted = SharingStarted.WhileSubscribed(5000),
    dataStoreName: String = "settings"
) : AndroidViewModel(application) {

    private val repository = SettingsRepository(application, dataStoreName)

    /**
     * Current settings as a StateFlow.
     */
    val settings: StateFlow<AppSettings> = repository.settingsFlow
        .stateIn(
            scope = viewModelScope,
            started = sharingStarted,
            initialValue = AppSettings()
        )

    /**
     * Update camera resolution.
     *
     * @param width Resolution width in pixels
     * @param height Resolution height in pixels
     */
    fun updateResolution(width: Int, height: Int) {
        viewModelScope.launch {
            repository.updateResolution(width, height)
        }
    }

    /**
     * Update camera FPS.
     *
     * @param fps Frames per second (e.g., 30, 60)
     */
    fun updateFps(fps: Int) {
        viewModelScope.launch {
            repository.updateFps(fps)
        }
    }

    /**
     * Update server IP address.
     * Only updates if the IP address format is valid.
     *
     * @param ip Server IP address (e.g., "192.168.1.100")
     */
    fun updateServerIp(ip: String) {
        viewModelScope.launch {
            repository.updateServerIp(ip)
        }
    }

    /**
     * Update server port.
     *
     * @param port Server port number (1-65535)
     */
    fun updateServerPort(port: Int) {
        if (port in 1..65535) {
            viewModelScope.launch {
                repository.updateServerPort(port)
            }
        }
    }

    /**
     * Update IMU enabled state.
     *
     * @param enabled True to enable IMU, false to disable
     */
    fun updateEnableImu(enabled: Boolean) {
        viewModelScope.launch {
            repository.updateEnableImu(enabled)
        }
    }

    /**
     * Update IMU sample rate.
     *
     * @param rate IMU sample rate in Hz (e.g., 100, 200, 400)
     */
    fun updateImuRate(rate: Int) {
        viewModelScope.launch {
            repository.updateImuRate(rate)
        }
    }

    companion object {
        /**
         * Supported camera resolutions.
         */
        val SUPPORTED_RESOLUTIONS = listOf(
            Resolution(640, 480, "VGA"),
            Resolution(1280, 720, "HD"),
            Resolution(1920, 1080, "Full HD"),
            Resolution(3840, 2160, "4K")
        )

        /**
         * Supported FPS values.
         */
        val SUPPORTED_FPS = listOf(15, 24, 30, 60)

        /**
         * Supported IMU sample rates.
         */
        val SUPPORTED_IMU_RATES = listOf(100, 200, 400)
    }
}

/**
 * Camera resolution data class.
 */
data class Resolution(
    val width: Int,
    val height: Int,
    val label: String
) {
    override fun toString(): String = "$label ($width×$height)"
}
