package com.vi.slam.android.sensor

import android.hardware.SensorEvent

/**
 * IMU sample data container.
 *
 * Contains accelerometer or gyroscope measurement with nanosecond-precision timestamp.
 * This class represents a single sensor reading from either the accelerometer or gyroscope.
 *
 * @property timestampNs Hardware timestamp in nanoseconds
 * @property type Sensor type (TYPE_ACCELEROMETER or TYPE_GYROSCOPE)
 * @property x Measurement along X axis
 * @property y Measurement along Y axis
 * @property z Measurement along Z axis
 */
data class IMUSample(
    val timestampNs: Long,
    val type: SensorType,
    val x: Float,
    val y: Float,
    val z: Float
) {
    companion object {
        /**
         * Create IMUSample from Android SensorEvent.
         *
         * @param event SensorEvent from accelerometer or gyroscope
         * @return IMUSample instance
         * @throws IllegalArgumentException if sensor type is not supported
         */
        fun fromSensorEvent(event: SensorEvent): IMUSample {
            val type = when (event.sensor.type) {
                android.hardware.Sensor.TYPE_ACCELEROMETER -> SensorType.ACCELEROMETER
                android.hardware.Sensor.TYPE_GYROSCOPE -> SensorType.GYROSCOPE
                else -> throw IllegalArgumentException(
                    "Unsupported sensor type: ${event.sensor.type}"
                )
            }

            return IMUSample(
                timestampNs = event.timestamp,
                type = type,
                x = event.values[0],
                y = event.values[1],
                z = event.values[2]
            )
        }
    }

    /**
     * Get measurement values as array.
     *
     * @return [x, y, z]
     */
    fun valuesArray(): FloatArray = floatArrayOf(x, y, z)

    /**
     * Validate sensor data.
     *
     * Checks if the measurement values are within reasonable ranges:
     * - Accelerometer: ±20 g (≈ ±196 m/s²)
     * - Gyroscope: ±35 rad/s (≈ ±2000 deg/s)
     *
     * @return true if data is valid, false otherwise
     */
    fun isValid(): Boolean {
        val maxValue = when (type) {
            SensorType.ACCELEROMETER -> 196.0f  // ±20g in m/s²
            SensorType.GYROSCOPE -> 35.0f       // ±2000 deg/s in rad/s
        }

        return valuesArray().all { value ->
            value.isFinite() && value >= -maxValue && value <= maxValue
        }
    }
}

/**
 * Sensor type enumeration.
 */
enum class SensorType {
    ACCELEROMETER,
    GYROSCOPE
}
