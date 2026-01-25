package com.vi.slam.android.camera

import android.graphics.ImageFormat
import android.hardware.camera2.CaptureResult
import android.media.Image
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.Mockito.*
import org.mockito.junit.MockitoJUnitRunner
import java.nio.ByteBuffer

@RunWith(MockitoJUnitRunner::class)
class FrameProcessorTest {

    @Mock
    private lateinit var mockImage: Image

    @Mock
    private lateinit var mockCaptureResult: CaptureResult

    @Mock
    private lateinit var mockYPlane: Image.Plane

    @Mock
    private lateinit var mockUPlane: Image.Plane

    @Mock
    private lateinit var mockVPlane: Image.Plane

    private lateinit var processor: FrameProcessor

    // Test image dimensions
    private val testWidth = 640
    private val testHeight = 480
    private val testTimestamp = 1234567890123456L
    private val testExposureTime = 33333333L
    private val testIso = 800

    @Before
    fun setup() {
        processor = FrameProcessor()

        // Configure mock image
        `when`(mockImage.format).thenReturn(ImageFormat.YUV_420_888)
        `when`(mockImage.width).thenReturn(testWidth)
        `when`(mockImage.height).thenReturn(testHeight)
        `when`(mockImage.timestamp).thenReturn(testTimestamp)

        // Configure mock capture result
        // Note: CaptureResult.get() is generic, but the actual Android implementation
        // may return different types. Mock returns exact types.
        doReturn(testExposureTime).`when`(mockCaptureResult).get(CaptureResult.SENSOR_EXPOSURE_TIME)
        doReturn(testIso).`when`(mockCaptureResult).get(CaptureResult.SENSOR_SENSITIVITY)

        // Configure mock planes for YUV image
        setupMockPlanes()
    }

    private fun setupMockPlanes() {
        // Create test data
        val yData = ByteArray(testWidth * testHeight) { 128.toByte() }
        val uvData = ByteArray((testWidth / 2) * (testHeight / 2)) { 128.toByte() }

        val yBuffer = ByteBuffer.wrap(yData)
        val uBuffer = ByteBuffer.wrap(uvData)
        val vBuffer = ByteBuffer.wrap(uvData)

        // Configure Y plane
        `when`(mockYPlane.buffer).thenReturn(yBuffer)
        `when`(mockYPlane.rowStride).thenReturn(testWidth)
        `when`(mockYPlane.pixelStride).thenReturn(1)

        // Configure U plane
        `when`(mockUPlane.buffer).thenReturn(uBuffer)
        `when`(mockUPlane.rowStride).thenReturn(testWidth / 2)
        `when`(mockUPlane.pixelStride).thenReturn(1)

        // Configure V plane
        `when`(mockVPlane.buffer).thenReturn(vBuffer)
        `when`(mockVPlane.rowStride).thenReturn(testWidth / 2)
        `when`(mockVPlane.pixelStride).thenReturn(1)

        `when`(mockImage.planes).thenReturn(arrayOf(mockYPlane, mockUPlane, mockVPlane))
    }

    @Test
    fun testProcessFrameGrayscale() {
        val result = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)

        assertEquals(OutputFormat.GRAYSCALE, result.format)
        assertEquals(testWidth * testHeight, result.data.remaining())
        assertEquals(0, result.metadata.sequenceNumber) // First frame
        assertEquals(testTimestamp, result.metadata.timestampNs)
        assertEquals(testWidth, result.metadata.width)
        assertEquals(testHeight, result.metadata.height)
        assertEquals(testExposureTime, result.metadata.exposureTimeNs)
        assertEquals(testIso, result.metadata.iso)
    }

    @Test
    fun testProcessFrameRgb() {
        val result = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.RGB)

        assertEquals(OutputFormat.RGB, result.format)
        assertEquals(testWidth * testHeight * 3, result.data.remaining())
        assertEquals(testTimestamp, result.metadata.timestampNs)
        assertEquals(testExposureTime, result.metadata.exposureTimeNs)
        assertEquals(testIso, result.metadata.iso)
    }

    @Test
    fun testSequenceNumberIncrement() {
        processor.resetSequence()

        val result1 = processor.processFrame(mockImage, mockCaptureResult)
        val result2 = processor.processFrame(mockImage, mockCaptureResult)
        val result3 = processor.processFrame(mockImage, mockCaptureResult)

        assertEquals(0, result1.metadata.sequenceNumber)
        assertEquals(1, result2.metadata.sequenceNumber)
        assertEquals(2, result3.metadata.sequenceNumber)
    }

    @Test
    fun testResetSequence() {
        processor.processFrame(mockImage, mockCaptureResult)
        processor.processFrame(mockImage, mockCaptureResult)

        processor.resetSequence()

        val result = processor.processFrame(mockImage, mockCaptureResult)
        assertEquals(0, result.metadata.sequenceNumber)
    }

    @Test
    fun testProcessFrameWithoutCaptureResult() {
        val result = processor.processFrame(mockImage, null, OutputFormat.GRAYSCALE)

        assertEquals(0L, result.metadata.exposureTimeNs)
        assertEquals(0, result.metadata.iso)
        assertEquals(testTimestamp, result.metadata.timestampNs)
    }

    @Test(expected = IllegalArgumentException::class)
    fun testProcessFrameInvalidFormat() {
        `when`(mockImage.format).thenReturn(ImageFormat.JPEG)
        processor.processFrame(mockImage, mockCaptureResult)
    }

    @Test
    fun testBufferReuse() {
        // Process multiple frames of same resolution
        val result1 = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)
        val result2 = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)

        // Both should succeed
        assertNotNull(result1.data)
        assertNotNull(result2.data)
        assertEquals(result1.data.capacity(), result2.data.capacity())
    }

    @Test
    fun testClearBuffers() {
        processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)
        processor.processFrame(mockImage, mockCaptureResult, OutputFormat.RGB)

        // Should not throw
        processor.clearBuffers()

        // Should still work after clearing
        val result = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)
        assertNotNull(result.data)
    }

    @Test
    fun testGrayscaleOutputSize() {
        val result = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)

        // Grayscale: 1 byte per pixel
        val expectedSize = testWidth * testHeight
        assertEquals(expectedSize, result.data.remaining())
    }

    @Test
    fun testRgbOutputSize() {
        val result = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.RGB)

        // RGB: 3 bytes per pixel
        val expectedSize = testWidth * testHeight * 3
        assertEquals(expectedSize, result.data.remaining())
    }

    @Test
    fun testMetadataAccuracy() {
        val testTimestamp2 = 9999999999999999L
        val testExposure2 = 16666666L
        val testIso2 = 1600

        `when`(mockImage.timestamp).thenReturn(testTimestamp2)
        doReturn(testExposure2).`when`(mockCaptureResult).get(CaptureResult.SENSOR_EXPOSURE_TIME)
        doReturn(testIso2).`when`(mockCaptureResult).get(CaptureResult.SENSOR_SENSITIVITY)

        val result = processor.processFrame(mockImage, mockCaptureResult)

        assertEquals(testTimestamp2, result.metadata.timestampNs)
        assertEquals(testExposure2, result.metadata.exposureTimeNs)
        assertEquals(testIso2, result.metadata.iso)
    }

    @Test
    fun testDifferentResolutions() {
        val resolutions = listOf(
            320 to 240,
            640 to 480,
            1280 to 720,
            1920 to 1080
        )

        resolutions.forEach { (width, height) ->
            `when`(mockImage.width).thenReturn(width)
            `when`(mockImage.height).thenReturn(height)

            // Re-setup planes for new resolution
            setupMockPlanesForResolution(width, height)

            val result = processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)

            assertEquals(width, result.metadata.width)
            assertEquals(height, result.metadata.height)
            assertEquals(width * height, result.data.remaining())
        }
    }

    private fun setupMockPlanesForResolution(width: Int, height: Int) {
        val yData = ByteArray(width * height) { 128.toByte() }
        val uvData = ByteArray((width / 2) * (height / 2)) { 128.toByte() }

        val yBuffer = ByteBuffer.wrap(yData)
        val uBuffer = ByteBuffer.wrap(uvData)
        val vBuffer = ByteBuffer.wrap(uvData)

        `when`(mockYPlane.buffer).thenReturn(yBuffer)
        `when`(mockYPlane.rowStride).thenReturn(width)
        `when`(mockYPlane.pixelStride).thenReturn(1)

        `when`(mockUPlane.buffer).thenReturn(uBuffer)
        `when`(mockUPlane.rowStride).thenReturn(width / 2)
        `when`(mockUPlane.pixelStride).thenReturn(1)

        `when`(mockVPlane.buffer).thenReturn(vBuffer)
        `when`(mockVPlane.rowStride).thenReturn(width / 2)
        `when`(mockVPlane.pixelStride).thenReturn(1)

        `when`(mockImage.planes).thenReturn(arrayOf(mockYPlane, mockUPlane, mockVPlane))
    }

    @Test
    fun testProcessingPerformance() {
        // Process a frame and measure time
        val startTime = System.nanoTime()
        processor.processFrame(mockImage, mockCaptureResult, OutputFormat.GRAYSCALE)
        val duration = (System.nanoTime() - startTime) / 1_000_000.0

        // Should be fast (< 10ms for unit test with mock data)
        assertTrue("Processing took ${duration}ms", duration < 10.0)
    }

    @Test
    fun testExposureTimeRange() {
        val exposureTimes = listOf(
            1_000_000L,    // 1ms
            10_000_000L,   // 10ms
            33_333_333L,   // ~30fps
            100_000_000L   // 100ms
        )

        exposureTimes.forEach { exposure ->
            doReturn(exposure).`when`(mockCaptureResult).get(CaptureResult.SENSOR_EXPOSURE_TIME)

            val result = processor.processFrame(mockImage, mockCaptureResult)
            assertEquals(exposure, result.metadata.exposureTimeNs)
        }
    }

    @Test
    fun testIsoRange() {
        val isoValues = listOf(100, 200, 400, 800, 1600, 3200)

        isoValues.forEach { iso ->
            doReturn(iso).`when`(mockCaptureResult).get(CaptureResult.SENSOR_SENSITIVITY)

            val result = processor.processFrame(mockImage, mockCaptureResult)
            assertEquals(iso, result.metadata.iso)
        }
    }
}
