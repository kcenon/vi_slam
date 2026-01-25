package com.vi.slam.android.sensor

/**
 * Exception thrown when IMU sensor operations fail.
 *
 * @property message Error message describing the failure
 * @property cause Original exception that caused this error
 */
class SensorException(
    message: String,
    cause: Throwable? = null
) : Exception(message, cause)
