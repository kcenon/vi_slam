package com.vi.slam.android.recorder

import android.media.MediaExtractor
import android.media.MediaFormat
import android.util.Log
import java.io.File

/**
 * Utility for recovering incomplete MP4 video files.
 *
 * When recording is interrupted, MP4 files may not be properly finalized,
 * resulting in unplayable files. This utility attempts to:
 * 1. Check if MP4 file is playable
 * 2. Extract metadata (frame count, duration) from partial files
 * 3. Mark files as recovered (future: implement actual MP4 repair using MediaMuxer)
 *
 * Note: Full MP4 repair (re-muxing) requires more complex logic and is
 * deferred to future iterations. Current implementation focuses on detection
 * and metadata extraction.
 */
object Mp4Recovery {

    private const val TAG = "Mp4Recovery"

    /**
     * Result of MP4 recovery attempt.
     */
    data class Mp4RecoveryResult(
        val success: Boolean,
        val playable: Boolean,
        val estimatedFrameCount: Long,
        val durationMs: Long,
        val errorMessage: String? = null
    )

    /**
     * Attempt to recover an incomplete MP4 file.
     *
     * Current implementation:
     * 1. Check if file is playable using MediaExtractor
     * 2. Extract available metadata (track count, format, duration)
     * 3. Estimate frame count from track samples
     *
     * Future enhancement:
     * - Implement full MP4 repair using MediaMuxer
     * - Extract raw H.264 frames if MP4 structure is corrupt
     *
     * @param videoFile MP4 file to recover
     * @return Mp4RecoveryResult with recovery status
     */
    fun recoverMp4(videoFile: File): Mp4RecoveryResult {
        if (!videoFile.exists()) {
            return Mp4RecoveryResult(
                success = false,
                playable = false,
                estimatedFrameCount = 0,
                durationMs = 0,
                errorMessage = "Video file does not exist: ${videoFile.absolutePath}"
            )
        }

        if (videoFile.length() == 0L) {
            return Mp4RecoveryResult(
                success = false,
                playable = false,
                estimatedFrameCount = 0,
                durationMs = 0,
                errorMessage = "Video file is empty"
            )
        }

        return try {
            val extractor = MediaExtractor()
            extractor.setDataSource(videoFile.absolutePath)

            val trackCount = extractor.trackCount
            Log.d(TAG, "MP4 file has $trackCount tracks")

            if (trackCount == 0) {
                extractor.release()
                return Mp4RecoveryResult(
                    success = false,
                    playable = false,
                    estimatedFrameCount = 0,
                    durationMs = 0,
                    errorMessage = "No tracks found in MP4 file"
                )
            }

            // Find video track
            var videoTrackIndex = -1
            var videoFormat: MediaFormat? = null

            for (i in 0 until trackCount) {
                val format = extractor.getTrackFormat(i)
                val mime = format.getString(MediaFormat.KEY_MIME) ?: continue

                if (mime.startsWith("video/")) {
                    videoTrackIndex = i
                    videoFormat = format
                    break
                }
            }

            if (videoTrackIndex == -1 || videoFormat == null) {
                extractor.release()
                return Mp4RecoveryResult(
                    success = false,
                    playable = false,
                    estimatedFrameCount = 0,
                    durationMs = 0,
                    errorMessage = "No video track found in MP4 file"
                )
            }

            // Select video track
            extractor.selectTrack(videoTrackIndex)

            // Extract duration
            val durationUs = videoFormat.getLong(MediaFormat.KEY_DURATION)
            val durationMs = durationUs / 1000

            // Count frames by iterating through samples
            var frameCount = 0L
            while (extractor.advance()) {
                frameCount++
            }

            extractor.release()

            Log.i(
                TAG,
                "MP4 recovery successful: frames=$frameCount, duration=${durationMs}ms"
            )

            Mp4RecoveryResult(
                success = true,
                playable = true,
                estimatedFrameCount = frameCount,
                durationMs = durationMs
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to recover MP4 file", e)

            // File exists but is not playable
            Mp4RecoveryResult(
                success = false,
                playable = false,
                estimatedFrameCount = 0,
                durationMs = 0,
                errorMessage = "MP4 file is corrupt or incomplete: ${e.message}"
            )
        }
    }

    /**
     * Check if MP4 file is playable.
     *
     * @param videoFile MP4 file to check
     * @return True if file can be opened and has valid tracks
     */
    fun isPlayable(videoFile: File): Boolean {
        if (!videoFile.exists() || videoFile.length() == 0L) {
            return false
        }

        return try {
            val extractor = MediaExtractor()
            extractor.setDataSource(videoFile.absolutePath)

            val trackCount = extractor.trackCount
            extractor.release()

            trackCount > 0
        } catch (e: Exception) {
            Log.d(TAG, "MP4 file is not playable: ${e.message}")
            false
        }
    }
}
