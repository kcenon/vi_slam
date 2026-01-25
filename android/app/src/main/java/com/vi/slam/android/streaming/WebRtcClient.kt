package com.vi.slam.android.streaming

import android.content.Context
import android.util.Log
import org.webrtc.*
import java.util.concurrent.atomic.AtomicBoolean

/**
 * WebRTC peer connection client for video/audio streaming and DataChannel communication.
 *
 * Manages the lifecycle of a WebRTC PeerConnection, including:
 * - PeerConnectionFactory initialization
 * - Offer/Answer SDP exchange
 * - ICE candidate exchange
 * - DataChannel creation
 * - Connection state monitoring
 *
 * Thread Safety: All methods are thread-safe and can be called from any thread.
 * WebRTC callbacks are executed on the signaling thread.
 *
 * @property context Android application context
 * @property iceServers List of ICE servers for NAT traversal
 * @property listener Callback for connection events
 */
class WebRtcClient(
    private val context: Context,
    private val iceServers: List<PeerConnection.IceServer>,
    private val listener: WebRtcClientListener
) {
    companion object {
        private const val TAG = "WebRtcClient"
        private const val DATA_CHANNEL_LABEL = "metadata"
    }

    private var peerConnectionFactory: PeerConnectionFactory? = null
    private var peerConnection: PeerConnection? = null
    private var dataChannel: DataChannel? = null

    private val isInitialized = AtomicBoolean(false)
    private val isConnected = AtomicBoolean(false)

    /**
     * Initialize WebRTC components.
     *
     * Must be called before any other operations.
     * Initializes PeerConnectionFactory with default settings.
     *
     * @throws IllegalStateException if already initialized
     */
    fun initialize() {
        if (isInitialized.getAndSet(true)) {
            throw IllegalStateException("WebRtcClient already initialized")
        }

        Log.d(TAG, "Initializing WebRTC components...")

        // Initialize PeerConnectionFactory global settings
        val options = PeerConnectionFactory.InitializationOptions.builder(context)
            .setEnableInternalTracer(false)
            .createInitializationOptions()

        PeerConnectionFactory.initialize(options)

        // Create PeerConnectionFactory
        val factoryOptions = PeerConnectionFactory.Options()

        peerConnectionFactory = PeerConnectionFactory.builder()
            .setOptions(factoryOptions)
            .createPeerConnectionFactory()

        Log.i(TAG, "WebRTC components initialized successfully")
    }

    /**
     * Create peer connection with configured ICE servers.
     *
     * Sets up a new PeerConnection with RTCConfiguration including:
     * - ICE servers for NAT traversal
     * - Unified Plan SDP semantics
     * - Continual gathering policy for better connectivity
     *
     * @throws IllegalStateException if not initialized or already connected
     */
    fun createPeerConnection() {
        if (!isInitialized.get()) {
            throw IllegalStateException("WebRtcClient not initialized")
        }

        if (peerConnection != null) {
            throw IllegalStateException("PeerConnection already exists")
        }

        Log.d(TAG, "Creating peer connection with ${iceServers.size} ICE servers")

        val rtcConfig = PeerConnection.RTCConfiguration(iceServers).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
            continualGatheringPolicy = PeerConnection.ContinualGatheringPolicy.GATHER_CONTINUALLY
            iceTransportsType = PeerConnection.IceTransportsType.ALL
        }

        peerConnection = peerConnectionFactory?.createPeerConnection(
            rtcConfig,
            object : PeerConnectionObserver() {
                override fun onIceCandidate(candidate: IceCandidate) {
                    Log.d(TAG, "ICE candidate: ${candidate.sdpMid}")
                    listener.onIceCandidate(candidate)
                }

                override fun onConnectionChange(newState: PeerConnection.PeerConnectionState) {
                    Log.i(TAG, "Connection state: $newState")

                    when (newState) {
                        PeerConnection.PeerConnectionState.CONNECTED -> {
                            isConnected.set(true)
                            listener.onConnected()
                        }
                        PeerConnection.PeerConnectionState.FAILED,
                        PeerConnection.PeerConnectionState.DISCONNECTED -> {
                            isConnected.set(false)
                            listener.onDisconnected()
                        }
                        else -> {
                            // Other states (new, connecting, closed)
                        }
                    }
                }

                override fun onIceConnectionChange(newState: PeerConnection.IceConnectionState) {
                    Log.d(TAG, "ICE connection state: $newState")
                }

                override fun onDataChannel(channel: DataChannel) {
                    Log.i(TAG, "Received DataChannel: ${channel.label()}")
                    dataChannel = channel
                    listener.onDataChannelOpened(channel)
                }
            }
        )

        Log.i(TAG, "Peer connection created successfully")
    }

    /**
     * Create DataChannel for metadata transmission.
     *
     * Creates a reliable, ordered DataChannel for sending frame metadata.
     * Configuration:
     * - Reliable: true (guaranteed delivery with retransmission)
     * - Ordered: true (messages delivered in order)
     *
     * @return Created DataChannel instance
     * @throws IllegalStateException if peer connection not created
     */
    fun createDataChannel(): DataChannel {
        val pc = peerConnection
            ?: throw IllegalStateException("PeerConnection not created")

        val init = DataChannel.Init().apply {
            ordered = true
            maxRetransmits = -1 // Infinite retransmits (reliable)
        }

        val channel = pc.createDataChannel(DATA_CHANNEL_LABEL, init)
        dataChannel = channel

        Log.i(TAG, "DataChannel created: ${channel.label()}")
        return channel
    }

    /**
     * Create SDP offer for connection initiation.
     *
     * Generates an SDP offer with default media constraints.
     * The offer is returned asynchronously via the listener callback.
     *
     * @throws IllegalStateException if peer connection not created
     */
    fun createOffer() {
        val pc = peerConnection
            ?: throw IllegalStateException("PeerConnection not created")

        Log.d(TAG, "Creating SDP offer...")

        val constraints = MediaConstraints().apply {
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveVideo", "false"))
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveAudio", "false"))
        }

        pc.createOffer(object : SdpObserver {
            override fun onCreateSuccess(sdp: SessionDescription) {
                Log.i(TAG, "Offer created successfully")
                pc.setLocalDescription(object : SdpObserver {
                    override fun onSetSuccess() {
                        Log.d(TAG, "Local description set")
                        listener.onLocalDescription(sdp)
                    }

                    override fun onSetFailure(error: String) {
                        Log.e(TAG, "Failed to set local description: $error")
                        listener.onError("Failed to set local description: $error")
                    }

                    override fun onCreateSuccess(sdp: SessionDescription) {}
                    override fun onCreateFailure(error: String) {}
                }, sdp)
            }

            override fun onCreateFailure(error: String) {
                Log.e(TAG, "Failed to create offer: $error")
                listener.onError("Failed to create offer: $error")
            }

            override fun onSetSuccess() {}
            override fun onSetFailure(error: String) {}
        }, constraints)
    }

    /**
     * Create SDP answer for connection response.
     *
     * Generates an SDP answer in response to a received offer.
     * The answer is returned asynchronously via the listener callback.
     *
     * @throws IllegalStateException if peer connection not created
     */
    fun createAnswer() {
        val pc = peerConnection
            ?: throw IllegalStateException("PeerConnection not created")

        Log.d(TAG, "Creating SDP answer...")

        val constraints = MediaConstraints()

        pc.createAnswer(object : SdpObserver {
            override fun onCreateSuccess(sdp: SessionDescription) {
                Log.i(TAG, "Answer created successfully")
                pc.setLocalDescription(object : SdpObserver {
                    override fun onSetSuccess() {
                        Log.d(TAG, "Local description set")
                        listener.onLocalDescription(sdp)
                    }

                    override fun onSetFailure(error: String) {
                        Log.e(TAG, "Failed to set local description: $error")
                        listener.onError("Failed to set local description: $error")
                    }

                    override fun onCreateSuccess(sdp: SessionDescription) {}
                    override fun onCreateFailure(error: String) {}
                }, sdp)
            }

            override fun onCreateFailure(error: String) {
                Log.e(TAG, "Failed to create answer: $error")
                listener.onError("Failed to create answer: $error")
            }

            override fun onSetSuccess() {}
            override fun onSetFailure(error: String) {}
        }, constraints)
    }

    /**
     * Set remote session description (offer or answer).
     *
     * @param sdp Remote session description
     */
    fun setRemoteDescription(sdp: SessionDescription) {
        val pc = peerConnection
            ?: throw IllegalStateException("PeerConnection not created")

        Log.d(TAG, "Setting remote description: ${sdp.type}")

        pc.setRemoteDescription(object : SdpObserver {
            override fun onSetSuccess() {
                Log.d(TAG, "Remote description set successfully")
            }

            override fun onSetFailure(error: String) {
                Log.e(TAG, "Failed to set remote description: $error")
                listener.onError("Failed to set remote description: $error")
            }

            override fun onCreateSuccess(sdp: SessionDescription) {}
            override fun onCreateFailure(error: String) {}
        }, sdp)
    }

    /**
     * Add ICE candidate from remote peer.
     *
     * @param candidate ICE candidate to add
     */
    fun addIceCandidate(candidate: IceCandidate) {
        val pc = peerConnection
            ?: throw IllegalStateException("PeerConnection not created")

        Log.d(TAG, "Adding ICE candidate: ${candidate.sdpMid}")
        pc.addIceCandidate(candidate)
    }

    /**
     * Get current DataChannel instance.
     *
     * @return DataChannel if created, null otherwise
     */
    fun getDataChannel(): DataChannel? = dataChannel

    /**
     * Check if peer connection is established.
     *
     * @return true if connected
     */
    fun isConnected(): Boolean = isConnected.get()

    /**
     * Close peer connection and release resources.
     *
     * Closes the DataChannel and PeerConnection, then disposes the factory.
     * After calling this method, the client must be re-initialized before use.
     */
    fun close() {
        Log.d(TAG, "Closing WebRTC client...")

        dataChannel?.close()
        dataChannel = null

        peerConnection?.close()
        peerConnection = null

        peerConnectionFactory?.dispose()
        peerConnectionFactory = null

        isInitialized.set(false)
        isConnected.set(false)

        Log.i(TAG, "WebRTC client closed")
    }

    /**
     * Base PeerConnection observer with no-op implementations.
     */
    private open class PeerConnectionObserver : PeerConnection.Observer {
        override fun onSignalingChange(state: PeerConnection.SignalingState) {}
        override fun onIceConnectionChange(state: PeerConnection.IceConnectionState) {}
        override fun onIceConnectionReceivingChange(receiving: Boolean) {}
        override fun onIceGatheringChange(state: PeerConnection.IceGatheringState) {}
        override fun onIceCandidate(candidate: IceCandidate) {}
        override fun onIceCandidatesRemoved(candidates: Array<out IceCandidate>) {}
        override fun onAddStream(stream: MediaStream) {}
        override fun onRemoveStream(stream: MediaStream) {}
        override fun onDataChannel(channel: DataChannel) {}
        override fun onRenegotiationNeeded() {}
        override fun onAddTrack(receiver: RtpReceiver, streams: Array<out MediaStream>) {}
        override fun onConnectionChange(newState: PeerConnection.PeerConnectionState) {}
    }
}

/**
 * Listener for WebRTC client events.
 */
interface WebRtcClientListener {
    /**
     * Called when local SDP description is created.
     *
     * @param sdp Local session description (offer or answer)
     */
    fun onLocalDescription(sdp: SessionDescription)

    /**
     * Called when ICE candidate is discovered.
     *
     * @param candidate ICE candidate to send to remote peer
     */
    fun onIceCandidate(candidate: IceCandidate)

    /**
     * Called when peer connection is established.
     */
    fun onConnected()

    /**
     * Called when peer connection is disconnected.
     */
    fun onDisconnected()

    /**
     * Called when DataChannel is opened (receiver side).
     *
     * @param channel Opened DataChannel
     */
    fun onDataChannelOpened(channel: DataChannel)

    /**
     * Called when an error occurs.
     *
     * @param message Error message
     */
    fun onError(message: String)
}
