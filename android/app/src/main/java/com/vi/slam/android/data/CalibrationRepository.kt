package com.vi.slam.android.data

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.vi.slam.android.ui.CalibrationQuality
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

/**
 * Repository for calibration data storage.
 * Manages camera and IMU calibration results using DataStore.
 */
class CalibrationRepository(private val context: Context) {

    companion object {
        private val Context.calibrationDataStore: DataStore<Preferences> by preferencesDataStore(name = "calibration")

        private val CAMERA_CALIBRATION_QUALITY = stringPreferencesKey("camera_calibration_quality")
        private val IMU_CALIBRATION_QUALITY = stringPreferencesKey("imu_calibration_quality")
        private val CALIBRATION_TIMESTAMP = stringPreferencesKey("calibration_timestamp")
    }

    /**
     * Flow of calibration data.
     */
    val calibrationFlow: Flow<CalibrationData> = context.calibrationDataStore.data.map { preferences ->
        CalibrationData(
            cameraQuality = preferences[CAMERA_CALIBRATION_QUALITY]?.let {
                CalibrationQuality.valueOf(it)
            } ?: CalibrationQuality.POOR,
            imuQuality = preferences[IMU_CALIBRATION_QUALITY]?.let {
                CalibrationQuality.valueOf(it)
            } ?: CalibrationQuality.POOR,
            timestamp = preferences[CALIBRATION_TIMESTAMP] ?: ""
        )
    }

    /**
     * Save calibration results.
     *
     * @param cameraQuality Camera calibration quality
     * @param imuQuality IMU calibration quality
     */
    suspend fun saveCalibration(
        cameraQuality: CalibrationQuality,
        imuQuality: CalibrationQuality
    ) {
        context.calibrationDataStore.edit { preferences ->
            preferences[CAMERA_CALIBRATION_QUALITY] = cameraQuality.name
            preferences[IMU_CALIBRATION_QUALITY] = imuQuality.name
            preferences[CALIBRATION_TIMESTAMP] = System.currentTimeMillis().toString()
        }
    }

    /**
     * Clear calibration data.
     */
    suspend fun clearCalibration() {
        context.calibrationDataStore.edit { preferences ->
            preferences.remove(CAMERA_CALIBRATION_QUALITY)
            preferences.remove(IMU_CALIBRATION_QUALITY)
            preferences.remove(CALIBRATION_TIMESTAMP)
        }
    }
}

/**
 * Calibration data class.
 */
data class CalibrationData(
    val cameraQuality: CalibrationQuality = CalibrationQuality.POOR,
    val imuQuality: CalibrationQuality = CalibrationQuality.POOR,
    val timestamp: String = ""
) {
    /**
     * Check if calibration is valid (not poor quality).
     */
    val isValid: Boolean
        get() = cameraQuality != CalibrationQuality.POOR &&
                imuQuality != CalibrationQuality.POOR
}
