package com.vi.slam.android.sensor

import java.util.concurrent.locks.ReentrantReadWriteLock
import kotlin.concurrent.read
import kotlin.concurrent.write

/**
 * Thread-safe circular buffer for IMU samples.
 *
 * Stores a fixed duration of IMU samples using a circular buffer strategy.
 * When the buffer is full, the oldest samples are overwritten by new ones.
 *
 * This implementation is thread-safe and supports concurrent reads and writes.
 *
 * @property capacitySamples Maximum number of samples to store
 */
class IMUCircularBuffer(
    private val capacitySamples: Int = DEFAULT_CAPACITY
) {
    companion object {
        /**
         * Default buffer capacity.
         * Stores 5 seconds of data at 500Hz = 2500 samples.
         */
        const val DEFAULT_CAPACITY = 2500

        /**
         * Create buffer with time-based capacity.
         *
         * @param durationSeconds Buffer duration in seconds
         * @param samplingRate Expected sampling rate in Hz
         * @return IMUCircularBuffer instance
         */
        fun withDuration(
            durationSeconds: Double = 5.0,
            samplingRate: Int = 500
        ): IMUCircularBuffer {
            val capacity = (durationSeconds * samplingRate).toInt()
            return IMUCircularBuffer(capacity)
        }
    }

    private val buffer = arrayOfNulls<IMUSample>(capacitySamples)
    private var writeIndex = 0
    private var size = 0
    private val lock = ReentrantReadWriteLock()

    /**
     * Add a sample to the buffer.
     *
     * If the buffer is full, overwrites the oldest sample.
     * Thread-safe: can be called concurrently with other operations.
     *
     * @param sample IMU sample to add
     */
    fun add(sample: IMUSample) = lock.write {
        buffer[writeIndex] = sample
        writeIndex = (writeIndex + 1) % capacitySamples
        if (size < capacitySamples) {
            size++
        }
    }

    /**
     * Get samples within a time range.
     *
     * Returns all samples with timestamps in [startTime, endTime] (inclusive).
     * Results are sorted by timestamp in ascending order.
     *
     * Thread-safe: can be called concurrently with add().
     *
     * @param startTime Start timestamp in nanoseconds (inclusive)
     * @param endTime End timestamp in nanoseconds (inclusive)
     * @return List of samples in the time range
     */
    fun getSamples(startTime: Long, endTime: Long): List<IMUSample> = lock.read {
        val result = mutableListOf<IMUSample>()

        if (size == 0) {
            return@read result
        }

        // Calculate the start index (oldest sample)
        val startIndex = if (size < capacitySamples) 0 else writeIndex

        // Iterate through all valid samples
        for (i in 0 until size) {
            val index = (startIndex + i) % capacitySamples
            val sample = buffer[index] ?: continue

            if (sample.timestampNs in startTime..endTime) {
                result.add(sample)
            }
        }

        // Sort by timestamp (should already be sorted, but ensure it)
        result.sortBy { it.timestampNs }
        result
    }

    /**
     * Get all buffered samples.
     *
     * Returns all samples in chronological order (oldest to newest).
     * Thread-safe: can be called concurrently with add().
     *
     * @return List of all buffered samples
     */
    fun getAllSamples(): List<IMUSample> = lock.read {
        val result = mutableListOf<IMUSample>()

        if (size == 0) {
            return@read result
        }

        val startIndex = if (size < capacitySamples) 0 else writeIndex

        for (i in 0 until size) {
            val index = (startIndex + i) % capacitySamples
            buffer[index]?.let { result.add(it) }
        }

        result
    }

    /**
     * Clear all buffered samples.
     *
     * Removes all samples from the buffer and resets internal state.
     * Thread-safe: can be called concurrently with other operations.
     */
    fun clear() = lock.write {
        buffer.fill(null)
        writeIndex = 0
        size = 0
    }

    /**
     * Get the number of samples currently in the buffer.
     *
     * Thread-safe: can be called concurrently with other operations.
     *
     * @return Number of samples
     */
    fun getSize(): Int = lock.read { size }

    /**
     * Check if the buffer is full.
     *
     * Thread-safe: can be called concurrently with other operations.
     *
     * @return true if buffer is at capacity
     */
    fun isFull(): Boolean = lock.read { size == capacitySamples }

    /**
     * Get the oldest sample in the buffer.
     *
     * Thread-safe: can be called concurrently with other operations.
     *
     * @return Oldest sample, or null if buffer is empty
     */
    fun getOldest(): IMUSample? = lock.read {
        if (size == 0) {
            null
        } else {
            val oldestIndex = if (size < capacitySamples) 0 else writeIndex
            buffer[oldestIndex]
        }
    }

    /**
     * Get the newest sample in the buffer.
     *
     * Thread-safe: can be called concurrently with other operations.
     *
     * @return Newest sample, or null if buffer is empty
     */
    fun getNewest(): IMUSample? = lock.read {
        if (size == 0) {
            null
        } else {
            val newestIndex = (writeIndex - 1 + capacitySamples) % capacitySamples
            buffer[newestIndex]
        }
    }
}
