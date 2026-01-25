package com.vi.slam.android.camera

import android.media.Image

/**
 * Callback for camera frame capture events.
 *
 * @param image The captured image frame
 * @param timestamp The hardware timestamp (SENSOR_TIMESTAMP) in nanoseconds
 */
typealias FrameCallback = (image: Image, timestamp: Long) -> Unit

/**
 * Camera capture configuration.
 *
 * @property resolution Target resolution for frame capture
 * @property fps Target frames per second
 * @property format Image format (YUV_420_888 recommended for processing)
 */
data class CaptureConfig(
    val resolution: android.util.Size,
    val fps: Int,
    val format: ImageFormat = ImageFormat.YUV_420_888
)

/**
 * Interface for camera capture operations.
 *
 * This interface defines the contract for managing camera capture lifecycle,
 * including starting/stopping capture sessions and handling frame callbacks.
 */
interface ICameraCapture {
    /**
     * Start camera capture with the specified configuration.
     *
     * Initializes the camera session and begins capturing frames at the
     * configured rate. Frames will be delivered via registered callbacks.
     *
     * @param config Capture configuration parameters
     * @throws CameraException if camera fails to start
     * @throws SecurityException if camera permission is not granted
     */
    suspend fun startCapture(config: CaptureConfig)

    /**
     * Stop camera capture and release resources.
     *
     * Stops the capture session and closes the camera device.
     * After calling this method, startCapture() must be called to resume.
     *
     * @throws CameraException if stopping fails
     */
    suspend fun stopCapture()

    /**
     * Register a callback to receive captured frames.
     *
     * Multiple callbacks can be registered. All registered callbacks will
     * be invoked for each captured frame.
     *
     * @param callback Function to be called when a frame is captured
     */
    fun registerFrameCallback(callback: FrameCallback)

    /**
     * Unregister a previously registered frame callback.
     *
     * @param callback The callback to remove
     */
    fun unregisterFrameCallback(callback: FrameCallback)
}
