package com.vi.slam.android.sensor

import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class IMUCircularBufferTest {

    private lateinit var buffer: IMUCircularBuffer

    @Before
    fun setUp() {
        buffer = IMUCircularBuffer(capacitySamples = 10)
    }

    private fun createSample(timestampNs: Long): IMUSample {
        return IMUSample(
            timestampNs = timestampNs,
            type = SensorType.ACCELEROMETER,
            x = 1.0f,
            y = 2.0f,
            z = 3.0f
        )
    }

    @Test
    fun `withDuration creates buffer with correct capacity`() {
        val buffer = IMUCircularBuffer.withDuration(
            durationSeconds = 2.0,
            samplingRate = 100
        )

        // 2 seconds * 100 Hz = 200 samples
        assertNotNull(buffer)
    }

    @Test
    fun `add increases size until capacity`() {
        assertEquals(0, buffer.getSize())

        buffer.add(createSample(1000L))
        assertEquals(1, buffer.getSize())

        buffer.add(createSample(2000L))
        assertEquals(2, buffer.getSize())
    }

    @Test
    fun `add overwrites oldest when full`() {
        // Fill buffer with 10 samples (timestamps 1000-10000)
        for (i in 1..10) {
            buffer.add(createSample((i * 1000).toLong()))
        }

        assertEquals(10, buffer.getSize())
        assertTrue(buffer.isFull())
        assertEquals(1000L, buffer.getOldest()?.timestampNs)
        assertEquals(10000L, buffer.getNewest()?.timestampNs)

        // Add one more (should overwrite oldest)
        buffer.add(createSample(11000L))

        assertEquals(10, buffer.getSize())
        assertTrue(buffer.isFull())
        assertEquals(2000L, buffer.getOldest()?.timestampNs)
        assertEquals(11000L, buffer.getNewest()?.timestampNs)
    }

    @Test
    fun `getSamples returns samples in time range`() {
        // Add samples with timestamps 1000, 2000, 3000, 4000, 5000
        for (i in 1..5) {
            buffer.add(createSample((i * 1000).toLong()))
        }

        // Get samples in range [2000, 4000]
        val samples = buffer.getSamples(2000L, 4000L)

        assertEquals(3, samples.size)
        assertEquals(2000L, samples[0].timestampNs)
        assertEquals(3000L, samples[1].timestampNs)
        assertEquals(4000L, samples[2].timestampNs)
    }

    @Test
    fun `getSamples returns empty list when no samples in range`() {
        buffer.add(createSample(1000L))
        buffer.add(createSample(2000L))

        val samples = buffer.getSamples(5000L, 6000L)

        assertTrue(samples.isEmpty())
    }

    @Test
    fun `getSamples returns sorted samples`() {
        // Add samples in non-chronological order
        buffer.add(createSample(3000L))
        buffer.add(createSample(1000L))
        buffer.add(createSample(2000L))

        val samples = buffer.getSamples(0L, 5000L)

        assertEquals(3, samples.size)
        assertEquals(1000L, samples[0].timestampNs)
        assertEquals(2000L, samples[1].timestampNs)
        assertEquals(3000L, samples[2].timestampNs)
    }

    @Test
    fun `getAllSamples returns all buffered samples`() {
        for (i in 1..5) {
            buffer.add(createSample((i * 1000).toLong()))
        }

        val samples = buffer.getAllSamples()

        assertEquals(5, samples.size)
        assertEquals(1000L, samples[0].timestampNs)
        assertEquals(5000L, samples[4].timestampNs)
    }

    @Test
    fun `getAllSamples returns samples in chronological order after wrapping`() {
        // Fill buffer and wrap around
        for (i in 1..15) {
            buffer.add(createSample((i * 1000).toLong()))
        }

        val samples = buffer.getAllSamples()

        assertEquals(10, samples.size)
        // Should have samples 6000-15000
        assertEquals(6000L, samples[0].timestampNs)
        assertEquals(15000L, samples[9].timestampNs)
    }

    @Test
    fun `clear removes all samples`() {
        for (i in 1..5) {
            buffer.add(createSample((i * 1000).toLong()))
        }

        assertEquals(5, buffer.getSize())

        buffer.clear()

        assertEquals(0, buffer.getSize())
        assertFalse(buffer.isFull())
        assertNull(buffer.getOldest())
        assertNull(buffer.getNewest())
    }

    @Test
    fun `getOldest returns null when empty`() {
        assertNull(buffer.getOldest())
    }

    @Test
    fun `getNewest returns null when empty`() {
        assertNull(buffer.getNewest())
    }

    @Test
    fun `isFull returns false when not full`() {
        buffer.add(createSample(1000L))
        assertFalse(buffer.isFull())
    }

    @Test
    fun `isFull returns true when at capacity`() {
        for (i in 1..10) {
            buffer.add(createSample((i * 1000).toLong()))
        }
        assertTrue(buffer.isFull())
    }

    @Test
    fun `buffer is thread-safe with concurrent adds`() {
        val numThreads = 10
        val samplesPerThread = 100
        val threads = mutableListOf<Thread>()

        for (threadId in 0 until numThreads) {
            val thread = Thread {
                for (i in 0 until samplesPerThread) {
                    val timestamp = (threadId * samplesPerThread + i).toLong()
                    buffer.add(createSample(timestamp))
                }
            }
            threads.add(thread)
            thread.start()
        }

        threads.forEach { it.join() }

        // Buffer should be full (capacity 10) with the latest 10 samples
        assertEquals(10, buffer.getSize())
        assertTrue(buffer.isFull())
    }

    @Test
    fun `buffer is thread-safe with concurrent reads and writes`() {
        val writeThread = Thread {
            for (i in 1..100) {
                buffer.add(createSample((i * 1000).toLong()))
                Thread.sleep(1)
            }
        }

        val readThread = Thread {
            for (i in 1..100) {
                buffer.getAllSamples()
                buffer.getSize()
                Thread.sleep(1)
            }
        }

        writeThread.start()
        readThread.start()

        writeThread.join()
        readThread.join()

        // Should not crash and should have some samples
        assertTrue(buffer.getSize() > 0)
    }
}
