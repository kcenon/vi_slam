package com.vi.slam.android.streaming

import android.util.Log
import org.webrtc.DataChannel
import java.nio.ByteBuffer
import java.nio.charset.StandardCharsets
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * WebRTC DataChannel-based metadata sender.
 *
 * Sends frame metadata over a reliable WebRTC DataChannel to ensure
 * synchronization with video frames. DataChannel provides:
 * - Reliable, ordered delivery
 * - Automatic retransmission
 * - NAT/firewall traversal
 *
 * @property dataChannel WebRTC DataChannel instance (must be configured as reliable)
 */
class DataChannelMetadataSender(
    private val dataChannel: DataChannel
) {
    companion object {
        private const val TAG = "DataChannelMetadata"
        private const val MAX_MESSAGE_SIZE = 16384 // 16KB per WebRTC spec
    }

    private val isReady = AtomicBoolean(false)
    private val messagesSent = AtomicLong(0)
    private val bytesSent = AtomicLong(0)

    init {
        // Verify DataChannel is configured correctly
        require(dataChannel.state() != DataChannel.State.CLOSED) {
            "DataChannel must not be closed"
        }

        // Monitor DataChannel state
        observeDataChannelState()
    }

    /**
     * Send frame metadata over DataChannel.
     *
     * Converts metadata to compact JSON and sends as UTF-8 text message.
     * The message size is validated against WebRTC limits.
     *
     * @param metadata Frame metadata to send
     * @throws IllegalStateException if DataChannel is not open
     * @throws IllegalArgumentException if message exceeds size limit
     */
    fun sendMetadata(metadata: FrameMetadata) {
        if (!isReady.get()) {
            throw IllegalStateException(
                "DataChannel not ready (state: ${dataChannel.state()})"
            )
        }

        val json = metadata.toJson()
        val bytes = json.toByteArray(StandardCharsets.UTF_8)

        if (bytes.size > MAX_MESSAGE_SIZE) {
            throw IllegalArgumentException(
                "Metadata message too large: ${bytes.size} bytes (max $MAX_MESSAGE_SIZE)"
            )
        }

        val buffer = ByteBuffer.wrap(bytes)
        val dataBuffer = DataChannel.Buffer(buffer, false) // false = text message

        val success = dataChannel.send(dataBuffer)
        if (!success) {
            Log.w(TAG, "Failed to send metadata: buffer full or channel busy")
        } else {
            messagesSent.incrementAndGet()
            bytesSent.addAndGet(bytes.size.toLong())

            Log.v(TAG, "Sent metadata: $metadata (${bytes.size} bytes)")
        }
    }

    /**
     * Check if DataChannel is ready for sending.
     *
     * @return true if DataChannel is open and ready
     */
    fun isChannelReady(): Boolean {
        return isReady.get() && dataChannel.state() == DataChannel.State.OPEN
    }

    /**
     * Get current sending statistics.
     *
     * @return Statistics snapshot
     */
    fun getStatistics(): MetadataStatistics {
        return MetadataStatistics(
            messagesSent = messagesSent.get(),
            bytesSent = bytesSent.get()
        )
    }

    /**
     * Observe DataChannel state changes.
     */
    private fun observeDataChannelState() {
        when (dataChannel.state()) {
            DataChannel.State.CONNECTING -> {
                Log.d(TAG, "DataChannel connecting...")
                isReady.set(false)
            }
            DataChannel.State.OPEN -> {
                Log.i(TAG, "DataChannel open and ready")
                isReady.set(true)
            }
            DataChannel.State.CLOSING -> {
                Log.w(TAG, "DataChannel closing...")
                isReady.set(false)
            }
            DataChannel.State.CLOSED -> {
                Log.w(TAG, "DataChannel closed")
                isReady.set(false)
            }
        }
    }

    /**
     * Register callback for DataChannel state changes.
     *
     * @param callback Function called when state changes
     */
    fun registerStateCallback(callback: (DataChannel.State) -> Unit) {
        dataChannel.registerObserver(object : DataChannel.Observer {
            override fun onBufferedAmountChange(previousAmount: Long) {
                // Not used
            }

            override fun onStateChange() {
                val state = dataChannel.state()
                Log.d(TAG, "DataChannel state changed: $state")
                isReady.set(state == DataChannel.State.OPEN)
                callback(state)
            }

            override fun onMessage(buffer: DataChannel.Buffer) {
                // Not used - this is a sender only
            }
        })
    }
}

/**
 * Metadata sending statistics.
 *
 * @property messagesSent Total metadata messages sent
 * @property bytesSent Total bytes sent
 */
data class MetadataStatistics(
    val messagesSent: Long,
    val bytesSent: Long
)
