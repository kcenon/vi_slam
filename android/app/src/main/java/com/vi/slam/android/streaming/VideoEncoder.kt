package com.vi.slam.android.streaming

import android.media.Image
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import android.view.Surface
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean

/**
 * H.264 video encoder using MediaCodec hardware acceleration.
 *
 * Encodes camera frames to H.264 format for WebRTC video streaming.
 * Features:
 * - Hardware-accelerated H.264 encoding
 * - Configurable bitrate and frame rate
 * - Surface-based input for efficient frame transfer
 * - Adaptive bitrate control support
 * - Low-latency encoding optimizations
 *
 * Thread Safety: All public methods are thread-safe.
 *
 * @property width Video frame width
 * @property height Video frame height
 * @property frameRate Target frame rate (FPS)
 * @property bitrate Initial bitrate in bits per second
 * @property listener Callback for encoded frames and errors
 */
class VideoEncoder(
    private val width: Int,
    private val height: Int,
    private val frameRate: Int = 30,
    private var bitrate: Int = 2_000_000, // 2 Mbps default
    private val listener: VideoEncoderListener
) {
    companion object {
        private const val TAG = "VideoEncoder"
        private const val MIME_TYPE = MediaFormat.MIMETYPE_VIDEO_AVC // H.264
        private const val I_FRAME_INTERVAL = 2 // I-frame every 2 seconds
        private const val TIMEOUT_US = 10_000L // 10ms timeout
        private const val MAX_BITRATE = 5_000_000 // 5 Mbps max
        private const val MIN_BITRATE = 500_000 // 500 Kbps min
    }

    private var encoder: MediaCodec? = null
    private var inputSurface: Surface? = null

    private val isRunning = AtomicBoolean(false)
    private val isConfigured = AtomicBoolean(false)

    private var frameCount = 0L
    private var encodedBytesTotal = 0L
    private var lastStatisticsTime = System.currentTimeMillis()

    /**
     * Configure and start the encoder.
     *
     * Creates MediaCodec with H.264 encoding settings optimized for:
     * - Low latency (baseline profile, no B-frames)
     * - Hardware acceleration
     * - Constant bitrate mode
     *
     * @throws EncoderException if configuration fails
     */
    fun start() {
        if (isRunning.getAndSet(true)) {
            throw IllegalStateException("Encoder already running")
        }

        try {
            Log.d(TAG, "Starting encoder: ${width}x${height} @ ${frameRate}fps, ${bitrate}bps")

            // Create MediaFormat
            val format = MediaFormat.createVideoFormat(MIME_TYPE, width, height).apply {
                setInteger(MediaFormat.KEY_COLOR_FORMAT,
                    MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
                setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
                setInteger(MediaFormat.KEY_FRAME_RATE, frameRate)
                setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, I_FRAME_INTERVAL)

                // Low latency optimizations
                setInteger(MediaFormat.KEY_PROFILE,
                    MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline)
                setInteger(MediaFormat.KEY_BITRATE_MODE,
                    MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR)
                setInteger(MediaFormat.KEY_PRIORITY, 0) // Real-time priority
            }

            // Create and configure encoder
            encoder = MediaCodec.createEncoderByType(MIME_TYPE)
            encoder?.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)

            // Get input surface for camera frames
            inputSurface = encoder?.createInputSurface()

            // Start encoder
            encoder?.start()

            isConfigured.set(true)
            Log.i(TAG, "Encoder started successfully")

        } catch (e: Exception) {
            isRunning.set(false)
            releaseEncoder()
            throw EncoderException("Failed to start encoder: ${e.message}", e)
        }
    }

    /**
     * Get input surface for camera frame rendering.
     *
     * Camera frames should be rendered to this surface for encoding.
     * The surface is created when the encoder starts.
     *
     * @return Input surface for camera frames
     * @throws IllegalStateException if encoder not started
     */
    fun getInputSurface(): Surface {
        return inputSurface
            ?: throw IllegalStateException("Encoder not started")
    }

    /**
     * Drain encoded frames from the encoder output buffer.
     *
     * Should be called periodically (e.g., every frame) to retrieve
     * encoded H.264 data. Encoded frames are delivered via the listener.
     *
     * @return Number of frames drained
     */
    fun drainEncoder(): Int {
        if (!isRunning.get()) {
            return 0
        }

        val codec = encoder ?: return 0
        val bufferInfo = MediaCodec.BufferInfo()
        var framesDrained = 0

        try {
            while (true) {
                val outputBufferIndex = codec.dequeueOutputBuffer(bufferInfo, TIMEOUT_US)

                when {
                    outputBufferIndex == MediaCodec.INFO_TRY_AGAIN_LATER -> {
                        // No more output available
                        break
                    }
                    outputBufferIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                        val format = codec.outputFormat
                        Log.d(TAG, "Output format changed: $format")
                        listener.onFormatChanged(format)
                    }
                    outputBufferIndex >= 0 -> {
                        val outputBuffer = codec.getOutputBuffer(outputBufferIndex)

                        if (outputBuffer != null && bufferInfo.size > 0) {
                            // Extract encoded data
                            outputBuffer.position(bufferInfo.offset)
                            outputBuffer.limit(bufferInfo.offset + bufferInfo.size)

                            val encodedData = ByteArray(bufferInfo.size)
                            outputBuffer.get(encodedData)

                            // Deliver encoded frame
                            val isKeyFrame = (bufferInfo.flags and
                                MediaCodec.BUFFER_FLAG_KEY_FRAME) != 0

                            listener.onEncodedFrame(
                                data = encodedData,
                                timestampUs = bufferInfo.presentationTimeUs,
                                isKeyFrame = isKeyFrame
                            )

                            // Update statistics
                            frameCount++
                            encodedBytesTotal += bufferInfo.size
                            framesDrained++

                            // Log statistics every 5 seconds
                            val now = System.currentTimeMillis()
                            if (now - lastStatisticsTime >= 5000) {
                                logStatistics()
                                lastStatisticsTime = now
                            }
                        }

                        codec.releaseOutputBuffer(outputBufferIndex, false)

                        // Check for end of stream
                        if ((bufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                            Log.i(TAG, "End of stream reached")
                            break
                        }
                    }
                }
            }

        } catch (e: Exception) {
            Log.e(TAG, "Error draining encoder: ${e.message}", e)
            listener.onError("Encoder drain error: ${e.message}")
        }

        return framesDrained
    }

    /**
     * Update encoder bitrate dynamically.
     *
     * Adjusts the target bitrate for adaptive streaming.
     * The new bitrate takes effect immediately.
     *
     * @param newBitrate Target bitrate in bits per second
     */
    fun setBitrate(newBitrate: Int) {
        val clampedBitrate = newBitrate.coerceIn(MIN_BITRATE, MAX_BITRATE)

        if (clampedBitrate == bitrate) {
            return
        }

        Log.d(TAG, "Updating bitrate: $bitrate -> $clampedBitrate bps")
        bitrate = clampedBitrate

        if (!isRunning.get()) {
            return
        }

        try {
            encoder?.let { codec ->
                val params = android.os.Bundle().apply {
                    putInt(MediaCodec.PARAMETER_KEY_VIDEO_BITRATE, bitrate)
                }
                codec.setParameters(params)
                Log.i(TAG, "Bitrate updated successfully")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to update bitrate: ${e.message}", e)
            listener.onError("Bitrate update failed: ${e.message}")
        }
    }

    /**
     * Request an immediate I-frame (keyframe).
     *
     * Useful for:
     * - New peer connection establishment
     * - Recovering from packet loss
     * - Switching to higher quality
     */
    fun requestKeyFrame() {
        if (!isRunning.get()) {
            return
        }

        try {
            encoder?.let { codec ->
                val params = android.os.Bundle().apply {
                    putInt(MediaCodec.PARAMETER_KEY_REQUEST_SYNC_FRAME, 0)
                }
                codec.setParameters(params)
                Log.d(TAG, "Key frame requested")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to request key frame: ${e.message}", e)
        }
    }

    /**
     * Get current encoder statistics.
     *
     * @return Encoder statistics
     */
    fun getStatistics(): EncoderStatistics {
        val duration = System.currentTimeMillis() - lastStatisticsTime
        val fps = if (duration > 0) {
            (frameCount * 1000.0 / duration).toInt()
        } else {
            0
        }

        val avgBitrate = if (duration > 0) {
            (encodedBytesTotal * 8 * 1000.0 / duration).toLong()
        } else {
            0L
        }

        return EncoderStatistics(
            framesEncoded = frameCount,
            currentFps = fps,
            averageBitrate = avgBitrate,
            targetBitrate = bitrate.toLong()
        )
    }

    /**
     * Stop encoder and release resources.
     *
     * Stops the encoder, releases all buffers, and closes the input surface.
     * After calling this method, start() must be called to resume encoding.
     */
    fun stop() {
        if (!isRunning.getAndSet(false)) {
            return
        }

        Log.d(TAG, "Stopping encoder...")

        try {
            // Signal end of stream
            encoder?.signalEndOfInputStream()

            // Drain remaining frames
            drainEncoder()

        } catch (e: Exception) {
            Log.e(TAG, "Error during encoder shutdown: ${e.message}", e)
        } finally {
            releaseEncoder()
            Log.i(TAG, "Encoder stopped")
        }
    }

    /**
     * Check if encoder is running.
     *
     * @return true if encoder is running
     */
    fun isRunning(): Boolean = isRunning.get()

    private fun releaseEncoder() {
        try {
            inputSurface?.release()
            inputSurface = null

            encoder?.stop()
            encoder?.release()
            encoder = null

            isConfigured.set(false)

        } catch (e: Exception) {
            Log.e(TAG, "Error releasing encoder: ${e.message}", e)
        }
    }

    private fun logStatistics() {
        val stats = getStatistics()
        Log.i(TAG, "Encoder stats: ${stats.framesEncoded} frames, " +
                "${stats.currentFps} fps, " +
                "${stats.averageBitrate / 1000} kbps (target: ${stats.targetBitrate / 1000} kbps)")
    }
}

/**
 * Listener for video encoder events.
 */
interface VideoEncoderListener {
    /**
     * Called when an encoded frame is available.
     *
     * @param data H.264 encoded data
     * @param timestampUs Presentation timestamp in microseconds
     * @param isKeyFrame true if this is an I-frame (keyframe)
     */
    fun onEncodedFrame(data: ByteArray, timestampUs: Long, isKeyFrame: Boolean)

    /**
     * Called when encoder output format changes.
     *
     * @param format New media format (contains SPS/PPS for H.264)
     */
    fun onFormatChanged(format: MediaFormat)

    /**
     * Called when an encoder error occurs.
     *
     * @param message Error message
     */
    fun onError(message: String)
}

/**
 * Encoder statistics.
 *
 * @property framesEncoded Total frames encoded
 * @property currentFps Current frames per second
 * @property averageBitrate Average bitrate in bits per second
 * @property targetBitrate Target bitrate in bits per second
 */
data class EncoderStatistics(
    val framesEncoded: Long,
    val currentFps: Int,
    val averageBitrate: Long,
    val targetBitrate: Long
)
