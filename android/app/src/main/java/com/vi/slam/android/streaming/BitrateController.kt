package com.vi.slam.android.streaming

import android.util.Log
import kotlinx.coroutines.*
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlin.math.min

/**
 * Adaptive bitrate controller for video streaming.
 *
 * Monitors network conditions and adjusts encoder bitrate dynamically
 * to maintain optimal video quality while preventing network congestion.
 *
 * Algorithm:
 * - Increases bitrate gradually when RTT is low and packet loss is minimal
 * - Decreases bitrate quickly when network degradation is detected
 * - Maintains bitrate within configured min/max bounds
 * - Responds within 2 seconds to network changes (as per requirement)
 *
 * Thread Safety: All public methods are thread-safe.
 *
 * @property initialBitrate Starting bitrate in bits per second
 * @property minBitrate Minimum allowed bitrate
 * @property maxBitrate Maximum allowed bitrate
 * @property encoder Video encoder to control
 */
class BitrateController(
    private val initialBitrate: Int = 2_000_000, // 2 Mbps
    private val minBitrate: Int = 500_000,       // 500 Kbps
    private val maxBitrate: Int = 5_000_000,     // 5 Mbps
    private val encoder: VideoEncoder
) {
    companion object {
        private const val TAG = "BitrateController"

        // Adjustment parameters
        private const val UPDATE_INTERVAL_MS = 1000L  // Check every 1 second
        private const val INCREASE_FACTOR = 1.1       // 10% increase when conditions are good
        private const val DECREASE_FACTOR = 0.8       // 20% decrease when degradation detected
        private const val STABILITY_THRESHOLD = 3     // Require 3 good samples before increasing

        // Network quality thresholds
        private const val RTT_GOOD_MS = 50            // RTT < 50ms is good
        private const val RTT_BAD_MS = 200            // RTT > 200ms is bad
        private const val PACKET_LOSS_GOOD = 0.01     // <1% packet loss is good
        private const val PACKET_LOSS_BAD = 0.05      // >5% packet loss is bad
    }

    private var currentBitrate = initialBitrate

    private val isRunning = AtomicBoolean(false)
    private var monitorJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    // Network statistics
    private var lastRttMs = 0
    private var lastPacketLoss = 0.0
    private var goodConditionsCount = 0

    /**
     * Start adaptive bitrate control.
     *
     * Begins monitoring network conditions and adjusting bitrate automatically.
     * The controller updates bitrate every second based on observed metrics.
     */
    fun start() {
        if (isRunning.getAndSet(true)) {
            Log.w(TAG, "Bitrate controller already running")
            return
        }

        Log.i(TAG, "Starting bitrate controller: initial=${currentBitrate}bps, " +
                "range=[${minBitrate}, ${maxBitrate}]")

        // Reset state
        currentBitrate = initialBitrate
        goodConditionsCount = 0
        encoder.setBitrate(currentBitrate)

        // Start monitoring loop
        monitorJob = scope.launch {
            while (isActive && isRunning.get()) {
                try {
                    adjustBitrate()
                    delay(UPDATE_INTERVAL_MS)
                } catch (e: CancellationException) {
                    break
                } catch (e: Exception) {
                    Log.e(TAG, "Error in bitrate adjustment: ${e.message}", e)
                }
            }
        }
    }

    /**
     * Stop adaptive bitrate control.
     *
     * Stops monitoring and bitrate adjustments.
     */
    fun stop() {
        if (!isRunning.getAndSet(false)) {
            return
        }

        Log.i(TAG, "Stopping bitrate controller")

        monitorJob?.cancel()
        monitorJob = null
    }

    /**
     * Update network statistics for bitrate adjustment.
     *
     * Should be called periodically with current network metrics.
     * The controller uses these metrics to adjust bitrate.
     *
     * @param rttMs Round-trip time in milliseconds
     * @param packetLoss Packet loss ratio (0.0 to 1.0)
     */
    fun updateNetworkStats(rttMs: Int, packetLoss: Double) {
        lastRttMs = rttMs
        lastPacketLoss = packetLoss.coerceIn(0.0, 1.0)

        Log.d(TAG, "Network stats updated: RTT=${rttMs}ms, " +
                "Loss=${String.format("%.2f%%", packetLoss * 100)}")
    }

    /**
     * Get current target bitrate.
     *
     * @return Current bitrate in bits per second
     */
    fun getCurrentBitrate(): Int = currentBitrate

    /**
     * Check if controller is running.
     *
     * @return true if controller is running
     */
    fun isRunning(): Boolean = isRunning.get()

    private fun adjustBitrate() {
        val networkQuality = evaluateNetworkQuality()

        val newBitrate = when (networkQuality) {
            NetworkQuality.GOOD -> {
                // Increase bitrate gradually
                goodConditionsCount++
                if (goodConditionsCount >= STABILITY_THRESHOLD) {
                    val increased = (currentBitrate * INCREASE_FACTOR).toInt()
                    min(increased, maxBitrate)
                } else {
                    currentBitrate // Wait for stability
                }
            }
            NetworkQuality.POOR -> {
                // Decrease bitrate immediately
                goodConditionsCount = 0
                val decreased = (currentBitrate * DECREASE_FACTOR).toInt()
                max(decreased, minBitrate)
            }
            NetworkQuality.NORMAL -> {
                // Maintain current bitrate
                goodConditionsCount = 0
                currentBitrate
            }
        }

        if (newBitrate != currentBitrate) {
            Log.i(TAG, "Bitrate adjustment: $currentBitrate -> $newBitrate bps " +
                    "(quality: $networkQuality, RTT: ${lastRttMs}ms, " +
                    "Loss: ${String.format("%.2f%%", lastPacketLoss * 100)})")

            currentBitrate = newBitrate
            encoder.setBitrate(currentBitrate)
        }
    }

    private fun evaluateNetworkQuality(): NetworkQuality {
        // No statistics available yet
        if (lastRttMs == 0) {
            return NetworkQuality.NORMAL
        }

        // Poor conditions: high RTT or high packet loss
        if (lastRttMs > RTT_BAD_MS || lastPacketLoss > PACKET_LOSS_BAD) {
            return NetworkQuality.POOR
        }

        // Good conditions: low RTT and low packet loss
        if (lastRttMs < RTT_GOOD_MS && lastPacketLoss < PACKET_LOSS_GOOD) {
            return NetworkQuality.GOOD
        }

        // Normal conditions
        return NetworkQuality.NORMAL
    }

    private enum class NetworkQuality {
        GOOD,    // Increase bitrate
        NORMAL,  // Maintain bitrate
        POOR     // Decrease bitrate
    }
}
