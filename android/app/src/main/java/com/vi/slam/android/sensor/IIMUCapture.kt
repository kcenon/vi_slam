package com.vi.slam.android.sensor

/**
 * Callback for IMU data events.
 *
 * @param sample The IMU sample data
 */
typealias IMUCallback = (sample: IMUSample) -> Unit

/**
 * Interface for IMU capture operations.
 *
 * This interface defines the contract for managing IMU sensor lifecycle,
 * including starting/stopping capture and retrieving historical samples.
 */
interface IIMUCapture {
    /**
     * Start IMU capture.
     *
     * Registers sensor listeners with SENSOR_DELAY_FASTEST and begins
     * buffering IMU samples. Samples will be delivered via registered callbacks.
     *
     * @throws SensorException if sensor registration fails
     * @throws SecurityException if sensor permission is not granted
     */
    fun startCapture()

    /**
     * Stop IMU capture and release resources.
     *
     * Unregisters sensor listeners and stops buffering.
     * Buffered data remains available until cleared.
     *
     * @throws SensorException if stopping fails
     */
    fun stopCapture()

    /**
     * Get IMU samples within a time range.
     *
     * Retrieves buffered samples with timestamps in the range [startTime, endTime].
     * Samples are returned in chronological order.
     *
     * @param startTime Start timestamp in nanoseconds (inclusive)
     * @param endTime End timestamp in nanoseconds (inclusive)
     * @return List of IMU samples in the time range
     */
    fun getSamples(startTime: Long, endTime: Long): List<IMUSample>

    /**
     * Register a callback to receive IMU samples.
     *
     * Multiple callbacks can be registered. All registered callbacks will
     * be invoked for each IMU sample.
     *
     * @param callback Function to be called when an IMU sample is available
     */
    fun registerCallback(callback: IMUCallback)

    /**
     * Unregister a previously registered IMU callback.
     *
     * @param callback The callback to remove
     */
    fun unregisterCallback(callback: IMUCallback)
}
