package com.vi.slam.android.data

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

/**
 * Repository for application settings using DataStore.
 * Provides persistent storage for camera configuration and server connection settings.
 */
class SettingsRepository(private val context: Context) {

    companion object {
        private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "settings")

        // Keys for settings
        private val CAMERA_RESOLUTION_WIDTH = intPreferencesKey("camera_resolution_width")
        private val CAMERA_RESOLUTION_HEIGHT = intPreferencesKey("camera_resolution_height")
        private val CAMERA_FPS = intPreferencesKey("camera_fps")
        private val SERVER_IP = stringPreferencesKey("server_ip")
        private val SERVER_PORT = intPreferencesKey("server_port")
        private val ENABLE_IMU = booleanPreferencesKey("enable_imu")
        private val IMU_RATE = intPreferencesKey("imu_rate")

        // Default values
        const val DEFAULT_RESOLUTION_WIDTH = 1920
        const val DEFAULT_RESOLUTION_HEIGHT = 1080
        const val DEFAULT_FPS = 30
        const val DEFAULT_SERVER_IP = "192.168.1.100"
        const val DEFAULT_SERVER_PORT = 8080
        const val DEFAULT_ENABLE_IMU = true
        const val DEFAULT_IMU_RATE = 200
    }

    /**
     * Flow of all settings.
     */
    val settingsFlow: Flow<AppSettings> = context.dataStore.data.map { preferences ->
        AppSettings(
            resolutionWidth = preferences[CAMERA_RESOLUTION_WIDTH] ?: DEFAULT_RESOLUTION_WIDTH,
            resolutionHeight = preferences[CAMERA_RESOLUTION_HEIGHT] ?: DEFAULT_RESOLUTION_HEIGHT,
            fps = preferences[CAMERA_FPS] ?: DEFAULT_FPS,
            serverIp = preferences[SERVER_IP] ?: DEFAULT_SERVER_IP,
            serverPort = preferences[SERVER_PORT] ?: DEFAULT_SERVER_PORT,
            enableImu = preferences[ENABLE_IMU] ?: DEFAULT_ENABLE_IMU,
            imuRate = preferences[IMU_RATE] ?: DEFAULT_IMU_RATE
        )
    }

    /**
     * Update camera resolution.
     */
    suspend fun updateResolution(width: Int, height: Int) {
        context.dataStore.edit { preferences ->
            preferences[CAMERA_RESOLUTION_WIDTH] = width
            preferences[CAMERA_RESOLUTION_HEIGHT] = height
        }
    }

    /**
     * Update camera FPS.
     */
    suspend fun updateFps(fps: Int) {
        context.dataStore.edit { preferences ->
            preferences[CAMERA_FPS] = fps
        }
    }

    /**
     * Update server IP address.
     */
    suspend fun updateServerIp(ip: String) {
        context.dataStore.edit { preferences ->
            preferences[SERVER_IP] = ip
        }
    }

    /**
     * Update server port.
     */
    suspend fun updateServerPort(port: Int) {
        context.dataStore.edit { preferences ->
            preferences[SERVER_PORT] = port
        }
    }

    /**
     * Update IMU enabled state.
     */
    suspend fun updateEnableImu(enabled: Boolean) {
        context.dataStore.edit { preferences ->
            preferences[ENABLE_IMU] = enabled
        }
    }

    /**
     * Update IMU sample rate.
     */
    suspend fun updateImuRate(rate: Int) {
        context.dataStore.edit { preferences ->
            preferences[IMU_RATE] = rate
        }
    }
}

/**
 * Application settings data class.
 */
data class AppSettings(
    val resolutionWidth: Int = SettingsRepository.DEFAULT_RESOLUTION_WIDTH,
    val resolutionHeight: Int = SettingsRepository.DEFAULT_RESOLUTION_HEIGHT,
    val fps: Int = SettingsRepository.DEFAULT_FPS,
    val serverIp: String = SettingsRepository.DEFAULT_SERVER_IP,
    val serverPort: Int = SettingsRepository.DEFAULT_SERVER_PORT,
    val enableImu: Boolean = SettingsRepository.DEFAULT_ENABLE_IMU,
    val imuRate: Int = SettingsRepository.DEFAULT_IMU_RATE
) {
    /**
     * Get resolution as a formatted string (e.g., "1920x1080").
     */
    val resolutionString: String
        get() = "${resolutionWidth}x${resolutionHeight}"
}
