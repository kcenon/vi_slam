package com.vi.slam.android.data

import com.vi.slam.android.sensor.SynchronizedData

/**
 * Interface for data consumers (destinations).
 *
 * Implementations of this interface receive synchronized camera and IMU data
 * from the Data Manager. Examples include recorders and streamers.
 *
 * Thread Safety: Implementations must be thread-safe as onData() may be called
 * from multiple threads or high-frequency callbacks.
 */
interface IDataDestination {
    /**
     * Handle incoming synchronized data.
     *
     * Called for each synchronized frame with associated IMU samples.
     * Implementations should process data quickly to avoid blocking
     * the data pipeline.
     *
     * @param data Synchronized camera and IMU data
     */
    fun onData(data: SynchronizedData)

    /**
     * Check if this destination is currently enabled.
     *
     * When disabled, the Data Manager may skip routing data to this destination.
     *
     * @return True if enabled and ready to receive data, false otherwise
     */
    fun isEnabled(): Boolean
}
