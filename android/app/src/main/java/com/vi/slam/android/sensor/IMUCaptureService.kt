package com.vi.slam.android.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log

/**
 * IMU capture service implementation.
 *
 * Manages accelerometer and gyroscope sensor lifecycle, captures high-frequency
 * IMU data, and maintains a circular buffer of recent samples.
 *
 * Features:
 * - Registers sensors with SENSOR_DELAY_FASTEST for maximum sampling rate
 * - Circular buffer stores 5 seconds of data at 500Hz (2500 samples)
 * - Thread-safe operations with concurrent read/write support
 * - Data validation filters invalid sensor readings
 * - Callback system for real-time data processing
 *
 * @property sensorManager Android SensorManager instance
 * @property bufferCapacity Maximum number of samples to buffer (default: 2500)
 */
class IMUCaptureService(
    private val sensorManager: SensorManager,
    bufferCapacity: Int = IMUCircularBuffer.DEFAULT_CAPACITY
) : IIMUCapture {

    companion object {
        private const val TAG = "IMUCaptureService"

        /**
         * Create IMUCaptureService from Android Context.
         *
         * @param context Android context
         * @param bufferCapacity Maximum number of samples to buffer
         * @return IMUCaptureService instance
         * @throws SensorException if SensorManager is unavailable
         */
        fun create(
            context: Context,
            bufferCapacity: Int = IMUCircularBuffer.DEFAULT_CAPACITY
        ): IMUCaptureService {
            val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
                ?: throw SensorException("SensorManager not available")

            return IMUCaptureService(sensorManager, bufferCapacity)
        }
    }

    private val buffer = IMUCircularBuffer(bufferCapacity)
    private val callbacks = mutableListOf<IMUCallback>()
    private val callbackLock = Any()

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var isCapturing = false

    private val sensorListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            try {
                val sample = IMUSample.fromSensorEvent(event)

                // Validate sensor data
                if (!sample.isValid()) {
                    Log.w(TAG, "Invalid sensor data: type=${sample.type}, values=${sample.valuesArray().contentToString()}")
                    return
                }

                // Add to buffer
                buffer.add(sample)

                // Notify callbacks
                synchronized(callbackLock) {
                    callbacks.forEach { callback ->
                        try {
                            callback(sample)
                        } catch (e: Exception) {
                            Log.e(TAG, "Callback error", e)
                        }
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error processing sensor event", e)
            }
        }

        override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
            Log.d(TAG, "Sensor accuracy changed: ${sensor.name}, accuracy=$accuracy")
        }
    }

    override fun startCapture() {
        if (isCapturing) {
            Log.w(TAG, "Capture already started")
            return
        }

        // Get accelerometer sensor
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
            ?: throw SensorException("Accelerometer not available")

        // Get gyroscope sensor
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
            ?: throw SensorException("Gyroscope not available")

        // Register listeners with SENSOR_DELAY_FASTEST
        val accelRegistered = sensorManager.registerListener(
            sensorListener,
            accelerometer,
            SensorManager.SENSOR_DELAY_FASTEST
        )

        val gyroRegistered = sensorManager.registerListener(
            sensorListener,
            gyroscope,
            SensorManager.SENSOR_DELAY_FASTEST
        )

        if (!accelRegistered || !gyroRegistered) {
            stopCapture()
            throw SensorException("Failed to register sensor listeners")
        }

        isCapturing = true
        Log.i(TAG, "IMU capture started")
    }

    override fun stopCapture() {
        if (!isCapturing) {
            Log.w(TAG, "Capture not started")
            return
        }

        sensorManager.unregisterListener(sensorListener)
        isCapturing = false
        Log.i(TAG, "IMU capture stopped")
    }

    override fun getSamples(startTime: Long, endTime: Long): List<IMUSample> {
        require(endTime >= startTime) {
            "endTime ($endTime) must be >= startTime ($startTime)"
        }

        return buffer.getSamples(startTime, endTime)
    }

    override fun registerCallback(callback: IMUCallback) {
        synchronized(callbackLock) {
            if (callback !in callbacks) {
                callbacks.add(callback)
                Log.d(TAG, "Callback registered (total: ${callbacks.size})")
            }
        }
    }

    override fun unregisterCallback(callback: IMUCallback) {
        synchronized(callbackLock) {
            if (callbacks.remove(callback)) {
                Log.d(TAG, "Callback unregistered (remaining: ${callbacks.size})")
            }
        }
    }

    /**
     * Clear all buffered samples.
     *
     * Removes all samples from the buffer while keeping the capture running.
     * Useful for resetting state between recording sessions.
     */
    fun clearBuffer() {
        buffer.clear()
        Log.d(TAG, "Buffer cleared")
    }

    /**
     * Get buffer statistics.
     *
     * @return Map of buffer statistics
     */
    fun getBufferStats(): Map<String, Any> {
        val oldest = buffer.getOldest()
        val newest = buffer.getNewest()

        return mapOf(
            "size" to buffer.getSize(),
            "capacity" to IMUCircularBuffer.DEFAULT_CAPACITY,
            "isFull" to buffer.isFull(),
            "oldestTimestamp" to (oldest?.timestampNs ?: 0L),
            "newestTimestamp" to (newest?.timestampNs ?: 0L),
            "durationMs" to if (oldest != null && newest != null) {
                (newest.timestampNs - oldest.timestampNs) / 1_000_000
            } else {
                0L
            }
        )
    }

    /**
     * Check if accelerometer is available.
     *
     * @return true if accelerometer sensor exists
     */
    fun hasAccelerometer(): Boolean {
        return sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null
    }

    /**
     * Check if gyroscope is available.
     *
     * @return true if gyroscope sensor exists
     */
    fun hasGyroscope(): Boolean {
        return sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null
    }

    /**
     * Check if currently capturing.
     *
     * @return true if capture is active
     */
    fun isCapturing(): Boolean = isCapturing
}
