package com.vi.slam.android.streaming

import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketException
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * UDP-based IMU data streamer.
 *
 * Sends IMU measurements over UDP to a specified destination.
 * Features:
 * - Non-blocking I/O using coroutines
 * - Packet loss monitoring
 * - Sequence number tracking
 * - Thread-safe start/stop
 *
 * Target: 200Hz IMU streaming with <1% packet loss
 *
 * @property destinationAddress Target IP address
 * @property destinationPort Target UDP port
 */
class UdpImuStreamer(
    private val destinationAddress: String,
    private val destinationPort: Int
) {
    companion object {
        private const val TAG = "UdpImuStreamer"
        private const val SOCKET_TIMEOUT_MS = 1000
    }

    private var socket: DatagramSocket? = null
    private val isRunning = AtomicBoolean(false)
    private val packetsSent = AtomicLong(0)
    private val bytesSent = AtomicLong(0)

    // Statistics for monitoring
    private var startTimeNs: Long = 0
    private var lastStatsLogTimeNs: Long = 0
    private val statsLogIntervalNs = 5_000_000_000L // 5 seconds

    /**
     * Start the UDP streamer.
     *
     * Opens a UDP socket bound to any available local port.
     * The socket is configured with a timeout to prevent blocking indefinitely.
     *
     * @throws SocketException if socket cannot be created
     */
    suspend fun start() = withContext(Dispatchers.IO) {
        if (isRunning.getAndSet(true)) {
            Log.w(TAG, "Streamer already running")
            return@withContext
        }

        try {
            socket = DatagramSocket().apply {
                soTimeout = SOCKET_TIMEOUT_MS
            }

            startTimeNs = System.nanoTime()
            lastStatsLogTimeNs = startTimeNs
            packetsSent.set(0)
            bytesSent.set(0)

            Log.i(TAG, "Started UDP IMU streamer: $destinationAddress:$destinationPort")
        } catch (e: SocketException) {
            isRunning.set(false)
            Log.e(TAG, "Failed to start UDP streamer", e)
            throw e
        }
    }

    /**
     * Stop the UDP streamer and release socket resources.
     */
    suspend fun stop() = withContext(Dispatchers.IO) {
        if (!isRunning.getAndSet(false)) {
            Log.w(TAG, "Streamer not running")
            return@withContext
        }

        socket?.close()
        socket = null

        logFinalStatistics()
        Log.i(TAG, "Stopped UDP IMU streamer")
    }

    /**
     * Send IMU data packet over UDP.
     *
     * Converts ImuData to binary packet and sends via UDP.
     * Non-blocking operation using coroutines.
     *
     * @param data IMU measurement to send
     * @throws IOException if send fails
     * @throws IllegalStateException if streamer is not started
     */
    suspend fun sendImuData(data: ImuData) = withContext(Dispatchers.IO) {
        if (!isRunning.get()) {
            throw IllegalStateException("Streamer not started")
        }

        val currentSocket = socket ?: throw IllegalStateException("Socket not initialized")

        try {
            val packet = data.toPacket()
            val datagramPacket = DatagramPacket(
                packet,
                packet.size,
                InetAddress.getByName(destinationAddress),
                destinationPort
            )

            currentSocket.send(datagramPacket)

            // Update statistics
            packetsSent.incrementAndGet()
            bytesSent.addAndGet(packet.size.toLong())

            // Periodic statistics logging
            val now = System.nanoTime()
            if (now - lastStatsLogTimeNs >= statsLogIntervalNs) {
                logStatistics()
                lastStatsLogTimeNs = now
            }
        } catch (e: IOException) {
            Log.e(TAG, "Failed to send IMU packet", e)
            throw e
        }
    }

    /**
     * Get current streaming statistics.
     *
     * @return Statistics snapshot
     */
    fun getStatistics(): StreamingStatistics {
        val elapsedNs = System.nanoTime() - startTimeNs
        val elapsedSeconds = elapsedNs / 1_000_000_000.0
        val packets = packetsSent.get()
        val bytes = bytesSent.get()

        return StreamingStatistics(
            packetsSent = packets,
            bytesSent = bytes,
            elapsedSeconds = elapsedSeconds,
            averagePacketsPerSecond = if (elapsedSeconds > 0) packets / elapsedSeconds else 0.0,
            averageBytesPerSecond = if (elapsedSeconds > 0) bytes / elapsedSeconds else 0.0
        )
    }

    /**
     * Check if streamer is currently running.
     *
     * @return true if streaming is active
     */
    fun isStreaming(): Boolean = isRunning.get()

    /**
     * Log current streaming statistics.
     */
    private fun logStatistics() {
        val stats = getStatistics()
        Log.d(
            TAG,
            "Stats: ${stats.packetsSent} packets " +
                    "(${String.format("%.1f", stats.averagePacketsPerSecond)} pps), " +
                    "${stats.bytesSent} bytes " +
                    "(${String.format("%.1f", stats.averageBytesPerSecond / 1024)} KB/s)"
        )
    }

    /**
     * Log final statistics when stopping.
     */
    private fun logFinalStatistics() {
        val stats = getStatistics()
        Log.i(
            TAG,
            "Final stats: ${stats.packetsSent} packets in " +
                    "${String.format("%.2f", stats.elapsedSeconds)} seconds " +
                    "(avg ${String.format("%.1f", stats.averagePacketsPerSecond)} pps)"
        )
    }
}

/**
 * UDP streaming statistics.
 *
 * @property packetsSent Total packets transmitted
 * @property bytesSent Total bytes transmitted
 * @property elapsedSeconds Time since start in seconds
 * @property averagePacketsPerSecond Average packet rate
 * @property averageBytesPerSecond Average byte rate
 */
data class StreamingStatistics(
    val packetsSent: Long,
    val bytesSent: Long,
    val elapsedSeconds: Double,
    val averagePacketsPerSecond: Double,
    val averageBytesPerSecond: Double
)
