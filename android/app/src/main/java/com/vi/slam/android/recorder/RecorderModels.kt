package com.vi.slam.android.recorder

import java.io.File

/**
 * Video encoding formats supported by the recorder.
 */
enum class VideoFormat {
    /**
     * H.264 (AVC) encoded video in MP4 container.
     *
     * Widely supported format with good compression ratio.
     * Compatible with most video players and SLAM tools.
     */
    H264_MP4
}

/**
 * IMU data output formats supported by the recorder.
 */
enum class ImuFormat {
    /**
     * Comma-Separated Values (CSV) format with nanosecond timestamps.
     *
     * Format: timestamp_ns,sensor_type,x,y,z
     * Example: 1234567890000000,accel,0.1,0.2,9.8
     *          1234567890500000,gyro,0.01,-0.02,0.00
     *
     * Compatible with EuRoC dataset format and most SLAM processing tools.
     */
    CSV
}

/**
 * Configuration for the recorder.
 *
 * Specifies output directory, file formats, duration limits, and metadata options.
 *
 * @property outputDirectory Base directory for recording output (must exist and be writable)
 * @property videoFormat Video encoding format (default: H264_MP4)
 * @property imuFormat IMU data format (default: CSV)
 * @property maxDurationMs Maximum recording duration in milliseconds (default: 60 minutes)
 * @property enableMetadata Whether to generate metadata.json (default: true)
 * @property videoWidth Video frame width in pixels (default: 640)
 * @property videoHeight Video frame height in pixels (default: 480)
 * @property videoFps Target video frame rate (default: 30 fps)
 * @property videoBitrate Video encoding bitrate in bits per second (default: 2 Mbps)
 */
data class RecorderConfig(
    val outputDirectory: File,
    val videoFormat: VideoFormat = VideoFormat.H264_MP4,
    val imuFormat: ImuFormat = ImuFormat.CSV,
    val maxDurationMs: Long = 60 * 60 * 1000,  // 60 minutes
    val enableMetadata: Boolean = true,
    val videoWidth: Int = 640,
    val videoHeight: Int = 480,
    val videoFps: Int = 30,
    val videoBitrate: Int = 2_000_000  // 2 Mbps
) {
    init {
        require(outputDirectory.exists() && outputDirectory.isDirectory) {
            "Output directory must exist: ${outputDirectory.absolutePath}"
        }
        require(outputDirectory.canWrite()) {
            "Output directory must be writable: ${outputDirectory.absolutePath}"
        }
        require(maxDurationMs > 0) {
            "Max duration must be positive: $maxDurationMs"
        }
        require(videoWidth > 0 && videoHeight > 0) {
            "Video dimensions must be positive: ${videoWidth}x$videoHeight"
        }
        require(videoFps > 0 && videoFps <= 120) {
            "Video FPS must be in range (0, 120]: $videoFps"
        }
        require(videoBitrate > 0) {
            "Video bitrate must be positive: $videoBitrate"
        }
    }
}

/**
 * Information about an active recording session.
 *
 * Returned by startRecording() to provide session metadata.
 *
 * @property recordingId Unique identifier for this recording session
 * @property startTime Recording start time in milliseconds (System.currentTimeMillis())
 * @property outputPath Full path to the recording output directory
 * @property config Recorder configuration used for this session
 */
data class RecordingInfo(
    val recordingId: String,
    val startTime: Long,
    val outputPath: String,
    val config: RecorderConfig
)

/**
 * Summary of a completed recording session.
 *
 * Returned by stopRecording() to provide statistics and output file information.
 *
 * @property recordingId Unique identifier for this recording session
 * @property success True if recording completed without errors
 * @property frameCount Total number of video frames recorded
 * @property imuSampleCount Total number of IMU samples recorded
 * @property durationMs Actual recording duration in milliseconds
 * @property outputFiles List of generated output files (video, IMU, metadata)
 * @property videoFile Path to the recorded video file (MP4)
 * @property imuFile Path to the recorded IMU data file (CSV)
 * @property metadataFile Path to the metadata JSON file (if enabled)
 * @property errorMessage Error description if success is false, null otherwise
 */
data class RecordingSummary(
    val recordingId: String,
    val success: Boolean,
    val frameCount: Long,
    val imuSampleCount: Long,
    val durationMs: Long,
    val outputFiles: List<String>,
    val videoFile: String? = null,
    val imuFile: String? = null,
    val metadataFile: String? = null,
    val errorMessage: String? = null
) {
    /**
     * Calculate average frame rate (FPS) for the recording.
     *
     * @return Average FPS, or 0.0 if duration is zero
     */
    fun averageFps(): Double {
        return if (durationMs > 0) {
            (frameCount * 1000.0) / durationMs
        } else {
            0.0
        }
    }

    /**
     * Calculate average IMU sampling rate (Hz) for the recording.
     *
     * @return Average IMU rate in Hz, or 0.0 if duration is zero
     */
    fun averageImuRate(): Double {
        return if (durationMs > 0) {
            (imuSampleCount * 1000.0) / durationMs
        } else {
            0.0
        }
    }
}

/**
 * Recording state for internal state machine management.
 *
 * Tracks the current state of the recorder throughout its lifecycle.
 */
enum class RecorderState {
    /**
     * Recorder is not initialized or has been reset.
     */
    UNINITIALIZED,

    /**
     * Recorder is initialized but no recording session is active.
     */
    IDLE,

    /**
     * Recording session is starting (transitional state).
     */
    STARTING,

    /**
     * Recording session is active and accepting data.
     */
    RECORDING,

    /**
     * Recording session is stopping (transitional state).
     */
    STOPPING,

    /**
     * An error occurred, recorder must be reinitialized.
     */
    ERROR
}
