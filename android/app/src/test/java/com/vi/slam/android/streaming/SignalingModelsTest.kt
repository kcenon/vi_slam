package com.vi.slam.android.streaming

import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.webrtc.IceCandidate
import org.webrtc.SessionDescription

@RunWith(RobolectricTestRunner::class)
class SignalingModelsTest {

    @Test
    fun `OfferMessage serialization and deserialization`() {
        val originalSdp = "v=0\r\no=- 123456789 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n"
        val offer = OfferMessage(originalSdp)

        // Serialize to JSON
        val json = offer.toJson()
        val jsonObject = JSONObject(json)

        assertEquals("offer", jsonObject.getString("type"))
        assertEquals(originalSdp, jsonObject.getString("sdp"))

        // Deserialize from JSON
        val parsed = SignalingMessage.fromJson(json) as OfferMessage
        assertEquals(originalSdp, parsed.sdp)
        assertEquals(SignalingMessageType.OFFER, parsed.type)
    }

    @Test
    fun `OfferMessage from SessionDescription`() {
        val sdp = "v=0\r\no=- 123456789 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n"
        val sessionDescription = SessionDescription(SessionDescription.Type.OFFER, sdp)

        val offer = OfferMessage.fromSessionDescription(sessionDescription)

        assertEquals(sdp, offer.sdp)
        assertEquals(SessionDescription.Type.OFFER, offer.toSessionDescription().type)
    }

    @Test
    fun `AnswerMessage serialization and deserialization`() {
        val originalSdp = "v=0\r\no=- 987654321 3 IN IP4 192.168.1.100\r\ns=-\r\nt=0 0\r\n"
        val answer = AnswerMessage(originalSdp)

        // Serialize to JSON
        val json = answer.toJson()
        val jsonObject = JSONObject(json)

        assertEquals("answer", jsonObject.getString("type"))
        assertEquals(originalSdp, jsonObject.getString("sdp"))

        // Deserialize from JSON
        val parsed = SignalingMessage.fromJson(json) as AnswerMessage
        assertEquals(originalSdp, parsed.sdp)
        assertEquals(SignalingMessageType.ANSWER, parsed.type)
    }

    @Test
    fun `AnswerMessage from SessionDescription`() {
        val sdp = "v=0\r\no=- 987654321 3 IN IP4 192.168.1.100\r\ns=-\r\nt=0 0\r\n"
        val sessionDescription = SessionDescription(SessionDescription.Type.ANSWER, sdp)

        val answer = AnswerMessage.fromSessionDescription(sessionDescription)

        assertEquals(sdp, answer.sdp)
        assertEquals(SessionDescription.Type.ANSWER, answer.toSessionDescription().type)
    }

    @Test
    fun `IceCandidateMessage serialization and deserialization`() {
        val candidateSdp = "candidate:1 1 UDP 2130706431 192.168.1.100 54321 typ host"
        val sdpMid = "0"
        val sdpMLineIndex = 0

        val candidateMessage = IceCandidateMessage(candidateSdp, sdpMid, sdpMLineIndex)

        // Serialize to JSON
        val json = candidateMessage.toJson()
        val jsonObject = JSONObject(json)

        assertEquals("ice_candidate", jsonObject.getString("type"))
        assertEquals(candidateSdp, jsonObject.getString("candidate"))
        assertEquals(sdpMid, jsonObject.getString("sdpMid"))
        assertEquals(sdpMLineIndex, jsonObject.getInt("sdpMLineIndex"))

        // Deserialize from JSON
        val parsed = SignalingMessage.fromJson(json) as IceCandidateMessage
        assertEquals(candidateSdp, parsed.candidate)
        assertEquals(sdpMid, parsed.sdpMid)
        assertEquals(sdpMLineIndex, parsed.sdpMLineIndex)
        assertEquals(SignalingMessageType.ICE_CANDIDATE, parsed.type)
    }

    @Test
    fun `IceCandidateMessage from IceCandidate`() {
        val candidateSdp = "candidate:1 1 UDP 2130706431 192.168.1.100 54321 typ host"
        val sdpMid = "audio"
        val sdpMLineIndex = 1

        val iceCandidate = IceCandidate(sdpMid, sdpMLineIndex, candidateSdp)
        val candidateMessage = IceCandidateMessage.fromIceCandidate(iceCandidate)

        assertEquals(candidateSdp, candidateMessage.candidate)
        assertEquals(sdpMid, candidateMessage.sdpMid)
        assertEquals(sdpMLineIndex, candidateMessage.sdpMLineIndex)

        // Convert back
        val converted = candidateMessage.toIceCandidate()
        assertEquals(candidateSdp, converted.sdp)
        assertEquals(sdpMid, converted.sdpMid)
        assertEquals(sdpMLineIndex, converted.sdpMLineIndex)
    }

    @Test
    fun `ErrorMessage serialization and deserialization`() {
        val errorText = "Connection timeout"
        val error = ErrorMessage(errorText)

        // Serialize to JSON
        val json = error.toJson()
        val jsonObject = JSONObject(json)

        assertEquals("error", jsonObject.getString("type"))
        assertEquals(errorText, jsonObject.getString("message"))

        // Deserialize from JSON
        val parsed = SignalingMessage.fromJson(json) as ErrorMessage
        assertEquals(errorText, parsed.message)
        assertEquals(SignalingMessageType.ERROR, parsed.type)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `Invalid message type throws exception`() {
        val invalidJson = """{"type": "invalid_type", "data": "something"}"""
        SignalingMessage.fromJson(invalidJson)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `OfferMessage from wrong SessionDescription type throws exception`() {
        val sdp = "v=0\r\no=- 123 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n"
        val answerDescription = SessionDescription(SessionDescription.Type.ANSWER, sdp)

        OfferMessage.fromSessionDescription(answerDescription)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `AnswerMessage from wrong SessionDescription type throws exception`() {
        val sdp = "v=0\r\no=- 123 2 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n"
        val offerDescription = SessionDescription(SessionDescription.Type.OFFER, sdp)

        AnswerMessage.fromSessionDescription(offerDescription)
    }

    @Test
    fun `SignalingMessageType fromString with valid types`() {
        assertEquals(SignalingMessageType.OFFER, SignalingMessageType.fromString("offer"))
        assertEquals(SignalingMessageType.ANSWER, SignalingMessageType.fromString("answer"))
        assertEquals(SignalingMessageType.ICE_CANDIDATE, SignalingMessageType.fromString("ice_candidate"))
        assertEquals(SignalingMessageType.ERROR, SignalingMessageType.fromString("error"))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `SignalingMessageType fromString with invalid type throws exception`() {
        SignalingMessageType.fromString("invalid")
    }

    @Test
    fun `Multiple message round-trip test`() {
        val messages = listOf(
            OfferMessage("offer_sdp_content"),
            AnswerMessage("answer_sdp_content"),
            IceCandidateMessage("candidate_content", "video", 2),
            ErrorMessage("Test error message")
        )

        messages.forEach { original ->
            val json = original.toJson()
            val parsed = SignalingMessage.fromJson(json)

            when {
                original is OfferMessage && parsed is OfferMessage -> {
                    assertEquals(original.sdp, parsed.sdp)
                }
                original is AnswerMessage && parsed is AnswerMessage -> {
                    assertEquals(original.sdp, parsed.sdp)
                }
                original is IceCandidateMessage && parsed is IceCandidateMessage -> {
                    assertEquals(original.candidate, parsed.candidate)
                    assertEquals(original.sdpMid, parsed.sdpMid)
                    assertEquals(original.sdpMLineIndex, parsed.sdpMLineIndex)
                }
                original is ErrorMessage && parsed is ErrorMessage -> {
                    assertEquals(original.message, parsed.message)
                }
                else -> fail("Type mismatch: ${original::class} vs ${parsed::class}")
            }
        }
    }
}
