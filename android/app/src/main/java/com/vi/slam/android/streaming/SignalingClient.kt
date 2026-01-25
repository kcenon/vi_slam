package com.vi.slam.android.streaming

import android.util.Log
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

/**
 * WebSocket-based signaling client for WebRTC peer connection establishment.
 *
 * Manages WebSocket connection to signaling server and handles message exchange
 * for SDP (offer/answer) and ICE candidate negotiation.
 *
 * Protocol:
 * - Text-based messages using JSON format
 * - Message types: offer, answer, ice_candidate, error
 * - Connection lifecycle: connect → ready → exchange → disconnect
 *
 * Thread Safety: All methods are thread-safe.
 * Callbacks are invoked on OkHttp's WebSocket thread.
 *
 * @property serverUrl WebSocket server URL (e.g., "ws://192.168.1.100:8080/signaling")
 * @property listener Callback for signaling events
 */
class SignalingClient(
    private val serverUrl: String,
    private val listener: SignalingListener
) {
    companion object {
        private const val TAG = "SignalingClient"
        private const val CONNECT_TIMEOUT_SEC = 10L
        private const val PING_INTERVAL_SEC = 30L
        private const val CLOSE_TIMEOUT_MS = 1000L
    }

    private val httpClient = OkHttpClient.Builder()
        .connectTimeout(CONNECT_TIMEOUT_SEC, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.SECONDS) // No timeout for long-lived connection
        .pingInterval(PING_INTERVAL_SEC, TimeUnit.SECONDS)
        .build()

    private var webSocket: WebSocket? = null
    private val isConnected = AtomicBoolean(false)

    /**
     * Connect to signaling server.
     *
     * Establishes WebSocket connection. Connection result is delivered
     * asynchronously via listener callbacks.
     *
     * @throws IllegalStateException if already connected
     */
    fun connect() {
        if (isConnected.get()) {
            throw IllegalStateException("Already connected to signaling server")
        }

        Log.d(TAG, "Connecting to signaling server: $serverUrl")

        val request = Request.Builder()
            .url(serverUrl)
            .build()

        webSocket = httpClient.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                Log.i(TAG, "WebSocket connected: ${response.code}")
                isConnected.set(true)
                listener.onConnected()
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                Log.d(TAG, "Received message: ${text.take(100)}...")

                try {
                    val message = SignalingMessage.fromJson(text)
                    listener.onMessageReceived(message)
                } catch (e: Exception) {
                    Log.e(TAG, "Failed to parse signaling message", e)
                    listener.onError("Invalid message format: ${e.message}")
                }
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                Log.e(TAG, "WebSocket failure: ${t.message}", t)
                isConnected.set(false)
                listener.onDisconnected("Connection failed: ${t.message}")
            }

            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                Log.d(TAG, "WebSocket closing: $code - $reason")
                webSocket.close(code, reason)
            }

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                Log.i(TAG, "WebSocket closed: $code - $reason")
                isConnected.set(false)
                listener.onDisconnected(reason)
            }
        })
    }

    /**
     * Send signaling message to server.
     *
     * @param message Signaling message to send
     * @throws IllegalStateException if not connected
     */
    fun sendMessage(message: SignalingMessage) {
        val ws = webSocket
            ?: throw IllegalStateException("Not connected to signaling server")

        if (!isConnected.get()) {
            throw IllegalStateException("WebSocket connection not ready")
        }

        val json = message.toJson()
        Log.d(TAG, "Sending message: ${json.take(100)}...")

        val success = ws.send(json)
        if (!success) {
            Log.w(TAG, "Failed to send message: buffer full or connection closing")
        }
    }

    /**
     * Send SDP offer.
     *
     * @param offer Offer message
     */
    fun sendOffer(offer: OfferMessage) {
        Log.d(TAG, "Sending offer")
        sendMessage(offer)
    }

    /**
     * Send SDP answer.
     *
     * @param answer Answer message
     */
    fun sendAnswer(answer: AnswerMessage) {
        Log.d(TAG, "Sending answer")
        sendMessage(answer)
    }

    /**
     * Send ICE candidate.
     *
     * @param candidate ICE candidate message
     */
    fun sendIceCandidate(candidate: IceCandidateMessage) {
        Log.d(TAG, "Sending ICE candidate: ${candidate.sdpMid}")
        sendMessage(candidate)
    }

    /**
     * Check if connected to signaling server.
     *
     * @return true if connected
     */
    fun isConnected(): Boolean = isConnected.get()

    /**
     * Disconnect from signaling server.
     *
     * Closes WebSocket connection gracefully.
     */
    fun disconnect() {
        Log.d(TAG, "Disconnecting from signaling server...")

        webSocket?.close(1000, "Normal closure")
        webSocket = null

        isConnected.set(false)

        Log.i(TAG, "Disconnected from signaling server")
    }

    /**
     * Shutdown signaling client and release resources.
     *
     * Closes WebSocket and shuts down HTTP client thread pool.
     * After calling this method, the client cannot be reused.
     */
    fun shutdown() {
        disconnect()

        httpClient.dispatcher.executorService.shutdown()
        httpClient.connectionPool.evictAll()

        Log.i(TAG, "Signaling client shutdown complete")
    }
}

/**
 * Listener for signaling events.
 */
interface SignalingListener {
    /**
     * Called when WebSocket connection is established.
     */
    fun onConnected()

    /**
     * Called when signaling message is received.
     *
     * @param message Received signaling message
     */
    fun onMessageReceived(message: SignalingMessage)

    /**
     * Called when WebSocket connection is closed.
     *
     * @param reason Reason for disconnection
     */
    fun onDisconnected(reason: String)

    /**
     * Called when an error occurs.
     *
     * @param message Error message
     */
    fun onError(message: String)
}
