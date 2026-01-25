package com.vi.slam.android.camera

import android.util.Size

/**
 * Camera configuration parameters
 */
data class CameraConfig(
    val resolution: Size,
    val fps: Int,
    val format: ImageFormat,
    val exposureMode: ExposureMode
)

/**
 * Supported image formats
 */
enum class ImageFormat {
    YUV_420_888,
    JPEG
}

/**
 * Exposure control modes
 */
enum class ExposureMode {
    AUTO,
    MANUAL,
    CONTINUOUS
}

/**
 * Camera hardware capability levels
 */
enum class HardwareLevel {
    LEGACY,
    LIMITED,
    FULL,
    LEVEL_3,
    EXTERNAL
}
