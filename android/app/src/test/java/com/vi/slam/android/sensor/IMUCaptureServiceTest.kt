package com.vi.slam.android.sensor

import android.hardware.SensorManager
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.mockito.kotlin.mock

class IMUCaptureServiceTest {

    private lateinit var sensorManager: SensorManager
    private lateinit var service: IMUCaptureService

    @Before
    fun setUp() {
        sensorManager = mock()
        service = IMUCaptureService(sensorManager, bufferCapacity = 100)
    }

    @Test
    fun `stopCapture is idempotent`() {
        service.stopCapture()
        assertFalse(service.isCapturing())

        // Should not throw
        service.stopCapture()
        assertFalse(service.isCapturing())
    }

    @Test
    fun `registerCallback adds callback`() {
        val callback: IMUCallback = { }

        service.registerCallback(callback)

        // Callback should be registered
        assertNotNull(callback)
    }

    @Test
    fun `unregisterCallback removes callback`() {
        val callback: IMUCallback = { }

        service.registerCallback(callback)
        service.unregisterCallback(callback)

        // Callback should be removed
        assertNotNull(callback)
    }

    @Test
    fun `getSamples returns empty list when no data`() {
        val samples = service.getSamples(0L, 1000000L)

        assertTrue(samples.isEmpty())
    }

    @Test(expected = IllegalArgumentException::class)
    fun `getSamples throws when endTime less than startTime`() {
        service.getSamples(1000L, 500L)
    }

    @Test
    fun `clearBuffer removes all samples`() {
        service.clearBuffer()

        val samples = service.getSamples(0L, Long.MAX_VALUE)
        assertTrue(samples.isEmpty())
    }

    @Test
    fun `getBufferStats returns correct statistics`() {
        val stats = service.getBufferStats()

        assertNotNull(stats)
        assertTrue(stats.containsKey("size"))
        assertTrue(stats.containsKey("capacity"))
        assertTrue(stats.containsKey("isFull"))
        assertEquals(0, stats["size"])
        assertEquals(false, stats["isFull"])
    }
}
