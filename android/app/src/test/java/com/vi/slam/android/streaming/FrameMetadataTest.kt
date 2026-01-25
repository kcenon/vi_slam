package com.vi.slam.android.streaming

import org.json.JSONException
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class FrameMetadataTest {

    @Test
    fun testToJson() {
        val metadata = FrameMetadata(
            sequenceNumber = 12345,
            timestampNs = 1234567890123456L,
            width = 1920,
            height = 1080,
            exposureTimeNs = 33333333L,
            iso = 800
        )

        val json = metadata.toJson()

        assertTrue(json.contains("\"seq\":12345"))
        assertTrue(json.contains("\"ts\":1234567890123456"))
        assertTrue(json.contains("\"w\":1920"))
        assertTrue(json.contains("\"h\":1080"))
        assertTrue(json.contains("\"exp\":33333333"))
        assertTrue(json.contains("\"iso\":800"))
    }

    @Test
    fun testFromJson() {
        val json = """{"seq":99999,"ts":9876543210987654,"w":3840,"h":2160,"exp":16666666,"iso":400}"""

        val metadata = FrameMetadata.fromJson(json)

        assertEquals(99999, metadata.sequenceNumber)
        assertEquals(9876543210987654L, metadata.timestampNs)
        assertEquals(3840, metadata.width)
        assertEquals(2160, metadata.height)
        assertEquals(16666666L, metadata.exposureTimeNs)
        assertEquals(400, metadata.iso)
    }

    @Test
    fun testRoundTrip() {
        val original = FrameMetadata(
            sequenceNumber = 54321,
            timestampNs = System.nanoTime(),
            width = 1280,
            height = 720,
            exposureTimeNs = 20000000L,
            iso = 1600
        )

        val json = original.toJson()
        val decoded = FrameMetadata.fromJson(json)

        assertEquals(original.sequenceNumber, decoded.sequenceNumber)
        assertEquals(original.timestampNs, decoded.timestampNs)
        assertEquals(original.width, decoded.width)
        assertEquals(original.height, decoded.height)
        assertEquals(original.exposureTimeNs, decoded.exposureTimeNs)
        assertEquals(original.iso, decoded.iso)
    }

    @Test(expected = JSONException::class)
    fun testFromJsonInvalidFormat() {
        val invalidJson = "{invalid json}"
        FrameMetadata.fromJson(invalidJson)
    }

    @Test(expected = JSONException::class)
    fun testFromJsonMissingFields() {
        val incompleteJson = """{"seq":123}"""
        FrameMetadata.fromJson(incompleteJson)
    }

    @Test
    fun testToStringFormat() {
        val metadata = FrameMetadata(
            sequenceNumber = 100,
            timestampNs = 1000000000,
            width = 640,
            height = 480,
            exposureTimeNs = 10000000,
            iso = 200
        )

        val str = metadata.toString()

        assertTrue(str.contains("seq=100"))
        assertTrue(str.contains("ts=1000000000"))
        assertTrue(str.contains("640x480"))
        assertTrue(str.contains("exp=10000000ns"))
        assertTrue(str.contains("iso=200"))
    }

    @Test
    fun testCompactJsonFormat() {
        val metadata = FrameMetadata(
            sequenceNumber = 1,
            timestampNs = 1000,
            width = 800,
            height = 600,
            exposureTimeNs = 5000000,
            iso = 100
        )

        val json = metadata.toJson()

        // Verify compact field names
        assertTrue(json.contains("seq"))
        assertTrue(json.contains("ts"))
        assertTrue(json.contains("w"))
        assertTrue(json.contains("h"))
        assertTrue(json.contains("exp"))
        assertTrue(json.contains("iso"))

        // Verify no verbose field names
        assertFalse(json.contains("sequenceNumber"))
        assertFalse(json.contains("timestampNs"))
        assertFalse(json.contains("width"))
        assertFalse(json.contains("height"))
        assertFalse(json.contains("exposureTimeNs"))
    }

    @Test
    fun testZeroSequenceNumber() {
        val metadata = FrameMetadata(
            sequenceNumber = 0,
            timestampNs = 1000,
            width = 1920,
            height = 1080
        )

        val json = metadata.toJson()
        val decoded = FrameMetadata.fromJson(json)

        assertEquals(0, decoded.sequenceNumber)
    }

    @Test
    fun testLargeSequenceNumber() {
        val metadata = FrameMetadata(
            sequenceNumber = Long.MAX_VALUE,
            timestampNs = 1000,
            width = 1920,
            height = 1080
        )

        val json = metadata.toJson()
        val decoded = FrameMetadata.fromJson(json)

        assertEquals(Long.MAX_VALUE, decoded.sequenceNumber)
    }

    @Test
    fun testCommonResolutions() {
        val resolutions = listOf(
            640 to 480,    // VGA
            1280 to 720,   // HD
            1920 to 1080,  // Full HD
            3840 to 2160   // 4K
        )

        resolutions.forEach { (width, height) ->
            val metadata = FrameMetadata(
                sequenceNumber = 1,
                timestampNs = 1000,
                width = width,
                height = height
            )

            val json = metadata.toJson()
            val decoded = FrameMetadata.fromJson(json)

            assertEquals(width, decoded.width)
            assertEquals(height, decoded.height)
        }
    }
}
