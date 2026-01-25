package com.vi.slam.android.streaming

import android.content.Context
import android.util.Log
import org.webrtc.DataChannel
import org.webrtc.IceCandidate
import org.webrtc.PeerConnection
import org.webrtc.SessionDescription
import java.util.concurrent.atomic.AtomicBoolean

/**
 * High-level manager for WebRTC connection lifecycle.
 *
 * Coordinates WebRTC peer connection and signaling client to establish
 * and maintain a WebRTC connection. Handles:
 * - Connection establishment (offer/answer exchange)
 * - ICE candidate exchange
 * - Connection state monitoring
 * - Automatic reconnection on network transitions
 *
 * Thread Safety: All public methods are thread-safe.
 *
 * @property context Android application context
 * @property signalingServerUrl WebSocket signaling server URL
 * @property iceServers ICE servers for NAT traversal
 * @property listener Callback for connection events
 */
class WebRtcConnectionManager(
    private val context: Context,
    private val signalingServerUrl: String,
    private val iceServers: List<PeerConnection.IceServer>,
    private val listener: ConnectionListener
) {
    companion object {
        private const val TAG = "WebRtcConnectionMgr"
    }

    private lateinit var webRtcClient: WebRtcClient
    private lateinit var signalingClient: SignalingClient

    private val isInitialized = AtomicBoolean(false)
    private val isConnecting = AtomicBoolean(false)
    private val isOfferer = AtomicBoolean(false)

    /**
     * Initialize WebRTC and signaling components.
     *
     * Must be called before starting connection.
     *
     * @throws IllegalStateException if already initialized
     */
    fun initialize() {
        if (isInitialized.getAndSet(true)) {
            throw IllegalStateException("Already initialized")
        }

        Log.d(TAG, "Initializing WebRTC connection manager...")

        // Initialize WebRTC client
        webRtcClient = WebRtcClient(
            context = context,
            iceServers = iceServers,
            listener = object : WebRtcClientListener {
                override fun onLocalDescription(sdp: SessionDescription) {
                    handleLocalDescription(sdp)
                }

                override fun onIceCandidate(candidate: IceCandidate) {
                    handleIceCandidate(candidate)
                }

                override fun onConnected() {
                    Log.i(TAG, "WebRTC peer connection established")
                    listener.onConnected()
                }

                override fun onDisconnected() {
                    Log.w(TAG, "WebRTC peer connection disconnected")
                    listener.onDisconnected("Peer connection lost")
                }

                override fun onDataChannelOpened(channel: DataChannel) {
                    Log.i(TAG, "DataChannel opened: ${channel.label()}")
                    listener.onDataChannelReady(channel)
                }

                override fun onError(message: String) {
                    Log.e(TAG, "WebRTC error: $message")
                    listener.onError(message)
                }
            }
        )

        webRtcClient.initialize()

        // Initialize signaling client
        signalingClient = SignalingClient(
            serverUrl = signalingServerUrl,
            listener = object : SignalingListener {
                override fun onConnected() {
                    Log.i(TAG, "Signaling server connected")
                    handleSignalingConnected()
                }

                override fun onMessageReceived(message: SignalingMessage) {
                    handleSignalingMessage(message)
                }

                override fun onDisconnected(reason: String) {
                    Log.w(TAG, "Signaling disconnected: $reason")
                    listener.onDisconnected("Signaling: $reason")
                }

                override fun onError(message: String) {
                    Log.e(TAG, "Signaling error: $message")
                    listener.onError("Signaling: $message")
                }
            }
        )

        Log.i(TAG, "WebRTC connection manager initialized")
    }

    /**
     * Start connection as offerer (initiator).
     *
     * Creates DataChannel, then connects to signaling server
     * and initiates SDP offer/answer exchange.
     *
     * @throws IllegalStateException if not initialized or already connecting
     */
    fun startAsOfferer() {
        checkInitialized()

        if (isConnecting.getAndSet(true)) {
            throw IllegalStateException("Already connecting")
        }

        isOfferer.set(true)
        Log.d(TAG, "Starting connection as offerer...")

        // Create peer connection
        webRtcClient.createPeerConnection()

        // Create DataChannel (offerer creates it)
        val dataChannel = webRtcClient.createDataChannel()
        Log.d(TAG, "DataChannel created: ${dataChannel.label()}")

        // Connect to signaling server
        signalingClient.connect()
    }

    /**
     * Start connection as answerer (responder).
     *
     * Connects to signaling server and waits for offer from remote peer.
     * DataChannel will be created by the offerer.
     *
     * @throws IllegalStateException if not initialized or already connecting
     */
    fun startAsAnswerer() {
        checkInitialized()

        if (isConnecting.getAndSet(true)) {
            throw IllegalStateException("Already connecting")
        }

        isOfferer.set(false)
        Log.d(TAG, "Starting connection as answerer...")

        // Create peer connection
        webRtcClient.createPeerConnection()

        // Connect to signaling server
        signalingClient.connect()
    }

    /**
     * Get current DataChannel instance.
     *
     * @return DataChannel if available, null otherwise
     */
    fun getDataChannel(): DataChannel? {
        return webRtcClient.getDataChannel()
    }

    /**
     * Check if peer connection is established.
     *
     * @return true if connected
     */
    fun isConnected(): Boolean {
        return webRtcClient.isConnected()
    }

    /**
     * Disconnect and cleanup resources.
     *
     * Closes signaling connection and WebRTC peer connection.
     * After calling this method, initialize() must be called again
     * before starting a new connection.
     */
    fun disconnect() {
        Log.d(TAG, "Disconnecting...")

        isConnecting.set(false)

        signalingClient.shutdown()
        webRtcClient.close()

        isInitialized.set(false)

        Log.i(TAG, "Disconnected and cleaned up")
    }

    /**
     * Handle signaling connection established.
     */
    private fun handleSignalingConnected() {
        if (isOfferer.get()) {
            // Offerer creates SDP offer
            Log.d(TAG, "Creating SDP offer...")
            webRtcClient.createOffer()
        } else {
            // Answerer waits for offer
            Log.d(TAG, "Waiting for SDP offer...")
        }
    }

    /**
     * Handle local SDP description created.
     */
    private fun handleLocalDescription(sdp: SessionDescription) {
        Log.d(TAG, "Local SDP created: ${sdp.type}")

        val message = when (sdp.type) {
            SessionDescription.Type.OFFER -> {
                OfferMessage.fromSessionDescription(sdp)
            }
            SessionDescription.Type.ANSWER -> {
                AnswerMessage.fromSessionDescription(sdp)
            }
            else -> {
                Log.e(TAG, "Unexpected SDP type: ${sdp.type}")
                return
            }
        }

        signalingClient.sendMessage(message)
    }

    /**
     * Handle ICE candidate discovered.
     */
    private fun handleIceCandidate(candidate: IceCandidate) {
        Log.d(TAG, "Local ICE candidate: ${candidate.sdpMid}")

        val message = IceCandidateMessage.fromIceCandidate(candidate)
        signalingClient.sendIceCandidate(message)
    }

    /**
     * Handle signaling message received.
     */
    private fun handleSignalingMessage(message: SignalingMessage) {
        Log.d(TAG, "Received signaling message: ${message.type}")

        when (message) {
            is OfferMessage -> {
                // Answerer receives offer, sets remote description, creates answer
                Log.d(TAG, "Received offer, creating answer...")
                webRtcClient.setRemoteDescription(message.toSessionDescription())
                webRtcClient.createAnswer()
            }

            is AnswerMessage -> {
                // Offerer receives answer, sets remote description
                Log.d(TAG, "Received answer")
                webRtcClient.setRemoteDescription(message.toSessionDescription())
            }

            is IceCandidateMessage -> {
                // Add remote ICE candidate
                Log.d(TAG, "Received ICE candidate: ${message.sdpMid}")
                webRtcClient.addIceCandidate(message.toIceCandidate())
            }

            is ErrorMessage -> {
                Log.e(TAG, "Received error: ${message.message}")
                listener.onError(message.message)
            }
        }
    }

    /**
     * Check if manager is initialized.
     */
    private fun checkInitialized() {
        if (!isInitialized.get()) {
            throw IllegalStateException("Not initialized. Call initialize() first.")
        }
    }
}

/**
 * Listener for connection events.
 */
interface ConnectionListener {
    /**
     * Called when WebRTC peer connection is established.
     */
    fun onConnected()

    /**
     * Called when connection is lost.
     *
     * @param reason Reason for disconnection
     */
    fun onDisconnected(reason: String)

    /**
     * Called when DataChannel is ready for use.
     *
     * @param channel Ready DataChannel instance
     */
    fun onDataChannelReady(channel: DataChannel)

    /**
     * Called when an error occurs.
     *
     * @param message Error message
     */
    fun onError(message: String)
}
