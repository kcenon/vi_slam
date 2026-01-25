package com.vi.slam.android.streaming

import org.json.JSONObject

/**
 * Frame metadata for synchronization with video stream.
 *
 * Contains essential information for frame identification and synchronization:
 * - Frame sequence number for ordering
 * - Hardware timestamp from camera sensor
 * - Frame dimensions
 *
 * Sent via WebRTC DataChannel to ensure reliable delivery and
 * synchronization with video frames.
 *
 * @property sequenceNumber Monotonically increasing frame number
 * @property timestampNs Hardware timestamp in nanoseconds (SENSOR_TIMESTAMP)
 * @property width Frame width in pixels
 * @property height Frame height in pixels
 */
data class FrameMetadata(
    val sequenceNumber: Long,
    val timestampNs: Long,
    val width: Int,
    val height: Int
) {
    companion object {
        /**
         * Create FrameMetadata from JSON string.
         *
         * Expected JSON format:
         * {
         *   "seq": 12345,
         *   "ts": 1234567890123456,
         *   "w": 1920,
         *   "h": 1080
         * }
         *
         * @param json JSON string representation
         * @return Decoded FrameMetadata instance
         * @throws org.json.JSONException if JSON is malformed
         */
        fun fromJson(json: String): FrameMetadata {
            val obj = JSONObject(json)
            return FrameMetadata(
                sequenceNumber = obj.getLong("seq"),
                timestampNs = obj.getLong("ts"),
                width = obj.getInt("w"),
                height = obj.getInt("h")
            )
        }
    }

    /**
     * Convert FrameMetadata to compact JSON string.
     *
     * Uses short field names to minimize bandwidth:
     * - seq: sequence number
     * - ts: timestamp
     * - w: width
     * - h: height
     *
     * @return JSON string representation
     */
    fun toJson(): String {
        return JSONObject().apply {
            put("seq", sequenceNumber)
            put("ts", timestampNs)
            put("w", width)
            put("h", height)
        }.toString()
    }

    /**
     * Convert to human-readable string for logging.
     *
     * @return Human-readable description
     */
    override fun toString(): String {
        return "FrameMetadata(seq=$sequenceNumber, ts=$timestampNs, ${width}x${height})"
    }
}
