package com.vi.slam.android.data

/**
 * Recording session data model.
 * Represents metadata for a single recording session.
 */
data class Session(
    val id: String,
    val startTime: Long,
    val duration: Long,
    val fileSize: Long,
    val frameCount: Int,
    val imuSampleCount: Int,
    val thumbnailPath: String? = null,
    val dataPath: String
) {
    /**
     * Format duration as HH:MM:SS.
     */
    val formattedDuration: String
        get() {
            val hours = duration / 3600000
            val minutes = (duration % 3600000) / 60000
            val seconds = (duration % 60000) / 1000
            return if (hours > 0) {
                String.format("%02d:%02d:%02d", hours, minutes, seconds)
            } else {
                String.format("%02d:%02d", minutes, seconds)
            }
        }

    /**
     * Format file size in human-readable format.
     */
    val formattedFileSize: String
        get() {
            return when {
                fileSize >= 1_000_000_000 -> String.format("%.2f GB", fileSize / 1_000_000_000.0)
                fileSize >= 1_000_000 -> String.format("%.2f MB", fileSize / 1_000_000.0)
                fileSize >= 1_000 -> String.format("%.2f KB", fileSize / 1_000.0)
                else -> "$fileSize B"
            }
        }
}
