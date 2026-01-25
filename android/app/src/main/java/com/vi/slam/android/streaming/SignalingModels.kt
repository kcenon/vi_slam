package com.vi.slam.android.streaming

import org.json.JSONObject
import org.webrtc.IceCandidate
import org.webrtc.SessionDescription

/**
 * Signaling message types for WebRTC peer connection establishment.
 */
enum class SignalingMessageType(val value: String) {
    OFFER("offer"),
    ANSWER("answer"),
    ICE_CANDIDATE("ice_candidate"),
    ERROR("error");

    companion object {
        fun fromString(value: String): SignalingMessageType {
            return values().find { it.value == value }
                ?: throw IllegalArgumentException("Unknown message type: $value")
        }
    }
}

/**
 * Base signaling message.
 */
sealed class SignalingMessage {
    abstract val type: SignalingMessageType

    /**
     * Convert message to JSON for transmission.
     */
    abstract fun toJson(): String

    companion object {
        /**
         * Parse signaling message from JSON.
         *
         * @param json JSON string
         * @return Parsed signaling message
         * @throws IllegalArgumentException if JSON is invalid
         */
        fun fromJson(json: String): SignalingMessage {
            val jsonObject = JSONObject(json)
            val typeString = jsonObject.getString("type")
            val type = SignalingMessageType.fromString(typeString)

            return when (type) {
                SignalingMessageType.OFFER -> {
                    val sdp = jsonObject.getString("sdp")
                    OfferMessage(sdp)
                }
                SignalingMessageType.ANSWER -> {
                    val sdp = jsonObject.getString("sdp")
                    AnswerMessage(sdp)
                }
                SignalingMessageType.ICE_CANDIDATE -> {
                    val candidate = jsonObject.getString("candidate")
                    val sdpMid = jsonObject.getString("sdpMid")
                    val sdpMLineIndex = jsonObject.getInt("sdpMLineIndex")
                    IceCandidateMessage(candidate, sdpMid, sdpMLineIndex)
                }
                SignalingMessageType.ERROR -> {
                    val message = jsonObject.getString("message")
                    ErrorMessage(message)
                }
            }
        }
    }
}

/**
 * SDP offer message.
 *
 * @property sdp Session description protocol string
 */
data class OfferMessage(val sdp: String) : SignalingMessage() {
    override val type = SignalingMessageType.OFFER

    override fun toJson(): String {
        return JSONObject().apply {
            put("type", type.value)
            put("sdp", sdp)
        }.toString()
    }

    /**
     * Convert to WebRTC SessionDescription.
     */
    fun toSessionDescription(): SessionDescription {
        return SessionDescription(SessionDescription.Type.OFFER, sdp)
    }

    companion object {
        /**
         * Create from WebRTC SessionDescription.
         */
        fun fromSessionDescription(sdp: SessionDescription): OfferMessage {
            require(sdp.type == SessionDescription.Type.OFFER) {
                "Expected OFFER, got ${sdp.type}"
            }
            return OfferMessage(sdp.description)
        }
    }
}

/**
 * SDP answer message.
 *
 * @property sdp Session description protocol string
 */
data class AnswerMessage(val sdp: String) : SignalingMessage() {
    override val type = SignalingMessageType.ANSWER

    override fun toJson(): String {
        return JSONObject().apply {
            put("type", type.value)
            put("sdp", sdp)
        }.toString()
    }

    /**
     * Convert to WebRTC SessionDescription.
     */
    fun toSessionDescription(): SessionDescription {
        return SessionDescription(SessionDescription.Type.ANSWER, sdp)
    }

    companion object {
        /**
         * Create from WebRTC SessionDescription.
         */
        fun fromSessionDescription(sdp: SessionDescription): AnswerMessage {
            require(sdp.type == SessionDescription.Type.ANSWER) {
                "Expected ANSWER, got ${sdp.type}"
            }
            return AnswerMessage(sdp.description)
        }
    }
}

/**
 * ICE candidate message.
 *
 * @property candidate ICE candidate SDP string
 * @property sdpMid Media stream identification tag
 * @property sdpMLineIndex Media line index in SDP
 */
data class IceCandidateMessage(
    val candidate: String,
    val sdpMid: String,
    val sdpMLineIndex: Int
) : SignalingMessage() {
    override val type = SignalingMessageType.ICE_CANDIDATE

    override fun toJson(): String {
        return JSONObject().apply {
            put("type", type.value)
            put("candidate", candidate)
            put("sdpMid", sdpMid)
            put("sdpMLineIndex", sdpMLineIndex)
        }.toString()
    }

    /**
     * Convert to WebRTC IceCandidate.
     */
    fun toIceCandidate(): IceCandidate {
        return IceCandidate(sdpMid, sdpMLineIndex, candidate)
    }

    companion object {
        /**
         * Create from WebRTC IceCandidate.
         */
        fun fromIceCandidate(candidate: IceCandidate): IceCandidateMessage {
            return IceCandidateMessage(
                candidate = candidate.sdp,
                sdpMid = candidate.sdpMid,
                sdpMLineIndex = candidate.sdpMLineIndex
            )
        }
    }
}

/**
 * Error message.
 *
 * @property message Error description
 */
data class ErrorMessage(val message: String) : SignalingMessage() {
    override val type = SignalingMessageType.ERROR

    override fun toJson(): String {
        return JSONObject().apply {
            put("type", type.value)
            put("message", message)
        }.toString()
    }
}
