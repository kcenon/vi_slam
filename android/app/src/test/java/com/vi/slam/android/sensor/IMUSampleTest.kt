package com.vi.slam.android.sensor

import org.junit.Assert.*
import org.junit.Test

class IMUSampleTest {

    @Test
    fun `valuesArray returns correct array`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.ACCELEROMETER,
            x = 1.0f,
            y = 2.0f,
            z = 3.0f
        )

        val values = sample.valuesArray()

        assertArrayEquals(floatArrayOf(1.0f, 2.0f, 3.0f), values, 0.001f)
    }

    @Test
    fun `isValid returns true for valid accelerometer data`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.ACCELEROMETER,
            x = 9.8f,
            y = 0.0f,
            z = 0.0f
        )

        assertTrue(sample.isValid())
    }

    @Test
    fun `isValid returns true for valid gyroscope data`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.GYROSCOPE,
            x = 1.0f,
            y = 2.0f,
            z = 3.0f
        )

        assertTrue(sample.isValid())
    }

    @Test
    fun `isValid returns false for out-of-range accelerometer data`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.ACCELEROMETER,
            x = 200.0f,  // Exceeds ±196 m/s²
            y = 0.0f,
            z = 0.0f
        )

        assertFalse(sample.isValid())
    }

    @Test
    fun `isValid returns false for out-of-range gyroscope data`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.GYROSCOPE,
            x = 50.0f,  // Exceeds ±35 rad/s
            y = 0.0f,
            z = 0.0f
        )

        assertFalse(sample.isValid())
    }

    @Test
    fun `isValid returns false for infinite values`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.ACCELEROMETER,
            x = Float.POSITIVE_INFINITY,
            y = 0.0f,
            z = 0.0f
        )

        assertFalse(sample.isValid())
    }

    @Test
    fun `isValid returns false for NaN values`() {
        val sample = IMUSample(
            timestampNs = 0L,
            type = SensorType.ACCELEROMETER,
            x = Float.NaN,
            y = 0.0f,
            z = 0.0f
        )

        assertFalse(sample.isValid())
    }
}
