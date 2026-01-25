package com.vi.slam.android.streaming

/**
 * Base exception for streaming-related errors.
 *
 * @param message Error description
 * @param cause Root cause exception
 */
open class StreamingException(
    message: String,
    cause: Throwable? = null
) : Exception(message, cause)

/**
 * Exception thrown when UDP streaming fails.
 *
 * @param message Error description
 * @param cause Root cause exception
 */
class UdpStreamingException(
    message: String,
    cause: Throwable? = null
) : StreamingException(message, cause)

/**
 * Exception thrown when DataChannel operations fail.
 *
 * @param message Error description
 * @param cause Root cause exception
 */
class DataChannelException(
    message: String,
    cause: Throwable? = null
) : StreamingException(message, cause)

/**
 * Exception thrown when packet encoding/decoding fails.
 *
 * @param message Error description
 * @param cause Root cause exception
 */
class PacketFormatException(
    message: String,
    cause: Throwable? = null
) : StreamingException(message, cause)

/**
 * Exception thrown when video encoder operations fail.
 *
 * @param message Error description
 * @param cause Root cause exception
 */
class EncoderException(
    message: String,
    cause: Throwable? = null
) : StreamingException(message, cause)
