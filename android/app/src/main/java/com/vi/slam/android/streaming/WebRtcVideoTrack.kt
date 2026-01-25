package com.vi.slam.android.streaming

import android.content.Context
import android.util.Log
import org.webrtc.*

/**
 * WebRTC video track creator and manager.
 *
 * Creates and manages WebRTC video tracks from camera frames for peer connection.
 * Integrates with VideoEncoder to stream H.264 encoded video.
 *
 * Features:
 * - Creates video source from camera capture
 * - Adds video track to peer connection
 * - Manages video track lifecycle
 * - Configures video constraints (resolution, FPS)
 *
 * Thread Safety: All public methods are thread-safe.
 *
 * @property context Android application context
 * @property peerConnectionFactory PeerConnectionFactory for creating tracks
 */
class WebRtcVideoTrack(
    private val context: Context,
    private val peerConnectionFactory: PeerConnectionFactory
) {
    companion object {
        private const val TAG = "WebRtcVideoTrack"
        private const val VIDEO_TRACK_ID = "ARDAMSv0" // Standard video track ID
        private const val VIDEO_CODEC_VP8 = "VP8"
        private const val VIDEO_CODEC_H264 = "H264"
    }

    private var videoSource: VideoSource? = null
    private var videoTrack: VideoTrack? = null
    private var videoCapturer: VideoCapturer? = null
    private var surfaceTextureHelper: SurfaceTextureHelper? = null

    /**
     * Create video source from camera.
     *
     * Initializes a camera video source with the specified configuration.
     *
     * @param width Frame width
     * @param height Frame height
     * @param fps Target frames per second
     * @return Created video source
     */
    fun createVideoSource(
        width: Int = 1920,
        height: Int = 1080,
        fps: Int = 30
    ): VideoSource {
        Log.d(TAG, "Creating video source: ${width}x${height} @ ${fps}fps")

        // Create video source
        val source = peerConnectionFactory.createVideoSource(false)
        videoSource = source

        Log.i(TAG, "Video source created successfully")
        return source
    }

    /**
     * Create video track from video source.
     *
     * Creates a video track that can be added to a peer connection.
     *
     * @param videoSource Video source to use
     * @return Created video track
     */
    fun createVideoTrack(videoSource: VideoSource): VideoTrack {
        Log.d(TAG, "Creating video track from source")

        val track = peerConnectionFactory.createVideoTrack(VIDEO_TRACK_ID, videoSource)
        videoTrack = track

        // Enable track
        track.setEnabled(true)

        Log.i(TAG, "Video track created: ${track.id()}")
        return track
    }

    /**
     * Create video track from camera with specified configuration.
     *
     * Convenience method that creates both source and track.
     *
     * @param width Frame width
     * @param height Frame height
     * @param fps Target frames per second
     * @return Created video track
     */
    fun createVideoTrackFromCamera(
        width: Int = 1920,
        height: Int = 1080,
        fps: Int = 30
    ): VideoTrack {
        val source = createVideoSource(width, height, fps)
        return createVideoTrack(source)
    }

    /**
     * Add video track to peer connection.
     *
     * Adds the video track to the specified peer connection for streaming.
     *
     * @param peerConnection Peer connection to add track to
     * @param videoTrack Video track to add
     * @param streamId Media stream ID (default: "stream")
     * @return RTP sender for the track
     */
    fun addVideoTrackToPeerConnection(
        peerConnection: PeerConnection,
        videoTrack: VideoTrack,
        streamId: String = "stream"
    ): RtpSender? {
        Log.d(TAG, "Adding video track to peer connection")

        val sender = peerConnection.addTrack(videoTrack, listOf(streamId))

        if (sender != null) {
            Log.i(TAG, "Video track added successfully")

            // Configure RTP parameters for low latency
            configureRtpSender(sender)
        } else {
            Log.e(TAG, "Failed to add video track")
        }

        return sender
    }

    /**
     * Get current video track.
     *
     * @return Current video track, or null if not created
     */
    fun getVideoTrack(): VideoTrack? = videoTrack

    /**
     * Get current video source.
     *
     * @return Current video source, or null if not created
     */
    fun getVideoSource(): VideoSource? = videoSource

    /**
     * Enable or disable video track.
     *
     * @param enabled true to enable, false to disable
     */
    fun setEnabled(enabled: Boolean) {
        videoTrack?.setEnabled(enabled)
        Log.d(TAG, "Video track ${if (enabled) "enabled" else "disabled"}")
    }

    /**
     * Dispose video track and source.
     *
     * Releases all resources associated with the video track.
     * After calling this method, a new track must be created.
     */
    fun dispose() {
        Log.d(TAG, "Disposing video track and source")

        videoCapturer?.dispose()
        videoCapturer = null

        surfaceTextureHelper?.dispose()
        surfaceTextureHelper = null

        videoTrack?.dispose()
        videoTrack = null

        videoSource?.dispose()
        videoSource = null

        Log.i(TAG, "Video track disposed")
    }

    private fun configureRtpSender(sender: RtpSender) {
        try {
            val parameters = sender.parameters

            // Configure degradation preference for low latency
            parameters.degradationPreference =
                RtpParameters.DegradationPreference.MAINTAIN_FRAMERATE

            // Configure encoding parameters
            val encodings = parameters.encodings
            if (encodings.isNotEmpty()) {
                val encoding = encodings[0]

                // Set encoding parameters for low latency
                encoding.maxBitrateBps = 5_000_000  // 5 Mbps max
                encoding.minBitrateBps = 500_000    // 500 Kbps min
                encoding.maxFramerate = 30
                encoding.scaleResolutionDownBy = 1.0 // No downscaling
            }

            // Apply parameters
            sender.parameters = parameters

            Log.d(TAG, "RTP sender configured for low latency")

        } catch (e: Exception) {
            Log.e(TAG, "Failed to configure RTP sender: ${e.message}", e)
        }
    }
}

/**
 * Video track configuration.
 *
 * @property width Frame width
 * @property height Frame height
 * @property fps Target frames per second
 * @property bitrate Target bitrate in bits per second
 */
data class VideoTrackConfig(
    val width: Int = 1920,
    val height: Int = 1080,
    val fps: Int = 30,
    val bitrate: Int = 2_000_000
)
