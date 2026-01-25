package com.vi.slam.android.camera

import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.TotalCaptureResult
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

/**
 * Implementation of ICameraCapture using Camera2 API.
 *
 * This service manages the camera session lifecycle, configures capture
 * parameters, and handles frame callbacks with accurate hardware timestamps.
 *
 * Features:
 * - Double-buffering via ImageReader to prevent frame drops
 * - Hardware timestamp (SENSOR_TIMESTAMP) extraction
 * - Configurable resolution and FPS
 * - Thread-safe callback management
 *
 * @property cameraWrapper Camera2 wrapper for device access
 */
class CameraCaptureService(
    private val cameraWrapper: Camera2Wrapper
) : ICameraCapture {

    companion object {
        private const val TAG = "CameraCaptureService"
        private const val IMAGE_READER_MAX_IMAGES = 2 // Double buffering
    }

    // Camera session state
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null

    // Background thread for camera operations
    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    // Registered frame callbacks
    private val frameCallbacks = mutableListOf<FrameCallback>()
    private val callbackLock = Any()

    // Current capture configuration
    private var currentConfig: CaptureConfig? = null

    /**
     * Start camera capture with the specified configuration.
     *
     * This method:
     * 1. Opens the camera device
     * 2. Creates an ImageReader for frame capture
     * 3. Configures capture session with target FPS
     * 4. Starts repeating capture requests
     *
     * @param config Capture configuration
     * @throws CameraException if camera initialization fails
     * @throws SecurityException if camera permission not granted
     */
    override suspend fun startCapture(config: CaptureConfig) {
        Log.d(TAG, "Starting capture: ${config.resolution}, ${config.fps} FPS")

        // Stop existing session if any
        stopCapture()

        // Save configuration
        currentConfig = config

        // Start background thread
        startBackgroundThread()

        // Open camera device
        val cameraIdList = cameraWrapper.getCameraIdList()
        if (cameraIdList.isEmpty()) {
            throw CameraException("No camera devices available")
        }

        val cameraId = cameraIdList.first() // Use first available camera
        val result = cameraWrapper.openCamera(cameraId, backgroundHandler)

        result.fold(
            onSuccess = { device ->
                cameraDevice = device
                createCaptureSession(config)
            },
            onFailure = { error ->
                stopBackgroundThread()
                throw CameraException("Failed to open camera", error)
            }
        )
    }

    /**
     * Stop camera capture and release all resources.
     *
     * This method:
     * 1. Stops the capture session
     * 2. Closes the camera device
     * 3. Releases the ImageReader
     * 4. Stops the background thread
     */
    override suspend fun stopCapture() {
        Log.d(TAG, "Stopping capture")

        // Close capture session
        captureSession?.close()
        captureSession = null

        // Close camera device
        cameraDevice?.close()
        cameraDevice = null

        // Close image reader
        imageReader?.close()
        imageReader = null

        // Stop background thread
        stopBackgroundThread()

        currentConfig = null
    }

    /**
     * Register a callback to receive captured frames.
     *
     * @param callback Function to be called with each frame
     */
    override fun registerFrameCallback(callback: FrameCallback) {
        synchronized(callbackLock) {
            frameCallbacks.add(callback)
            Log.d(TAG, "Registered frame callback (total: ${frameCallbacks.size})")
        }
    }

    /**
     * Unregister a previously registered callback.
     *
     * @param callback The callback to remove
     */
    override fun unregisterFrameCallback(callback: FrameCallback) {
        synchronized(callbackLock) {
            frameCallbacks.remove(callback)
            Log.d(TAG, "Unregistered frame callback (remaining: ${frameCallbacks.size})")
        }
    }

    /**
     * Create and configure the capture session.
     */
    private suspend fun createCaptureSession(config: CaptureConfig) {
        val device = cameraDevice ?: throw CameraException("Camera device not initialized")

        // Create ImageReader for frame capture
        val format = when (config.format) {
            ImageFormat.YUV_420_888 -> android.graphics.ImageFormat.YUV_420_888
            ImageFormat.JPEG -> android.graphics.ImageFormat.JPEG
        }

        imageReader = ImageReader.newInstance(
            config.resolution.width,
            config.resolution.height,
            format,
            IMAGE_READER_MAX_IMAGES
        ).apply {
            setOnImageAvailableListener({ reader ->
                handleImageAvailable(reader)
            }, backgroundHandler)
        }

        val surface = imageReader!!.surface

        // Create capture session
        suspendCancellableCoroutine<Unit> { continuation ->
            device.createCaptureSession(
                listOf(surface),
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        if (continuation.isActive) {
                            captureSession = session
                            startRepeatingRequest(session, device, surface, config)
                            continuation.resume(Unit)
                        }
                    }

                    override fun onConfigureFailed(session: CameraCaptureSession) {
                        if (continuation.isActive) {
                            continuation.resumeWithException(
                                CameraException("Failed to configure capture session")
                            )
                        }
                    }
                },
                backgroundHandler
            )
        }
    }

    /**
     * Start repeating capture requests for continuous frame capture.
     */
    private fun startRepeatingRequest(
        session: CameraCaptureSession,
        device: CameraDevice,
        surface: Surface,
        config: CaptureConfig
    ) {
        val captureRequestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
            addTarget(surface)

            // Configure FPS
            set(
                CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
                android.util.Range(config.fps, config.fps)
            )

            // Enable auto exposure
            set(
                CaptureRequest.CONTROL_AE_MODE,
                CaptureRequest.CONTROL_AE_MODE_ON
            )

            // Enable auto focus
            set(
                CaptureRequest.CONTROL_AF_MODE,
                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
            )
        }

        session.setRepeatingRequest(
            captureRequestBuilder.build(),
            object : CameraCaptureSession.CaptureCallback() {
                override fun onCaptureCompleted(
                    session: CameraCaptureSession,
                    request: CaptureRequest,
                    result: TotalCaptureResult
                ) {
                    // Capture completed successfully
                }
            },
            backgroundHandler
        )

        Log.d(TAG, "Started repeating capture requests at ${config.fps} FPS")
    }

    /**
     * Handle new image frames from ImageReader.
     */
    private fun handleImageAvailable(reader: ImageReader) {
        val image = reader.acquireLatestImage() ?: return

        try {
            // Extract hardware timestamp (SENSOR_TIMESTAMP)
            val timestamp = image.timestamp

            // Notify all registered callbacks
            synchronized(callbackLock) {
                frameCallbacks.forEach { callback ->
                    try {
                        callback(image, timestamp)
                    } catch (e: Exception) {
                        Log.e(TAG, "Frame callback error", e)
                    }
                }
            }
        } finally {
            // Always close the image to prevent buffer exhaustion
            image.close()
        }
    }

    /**
     * Start background thread for camera operations.
     */
    private fun startBackgroundThread() {
        backgroundThread = HandlerThread("CameraBackground").apply {
            start()
            backgroundHandler = Handler(looper)
        }
    }

    /**
     * Stop background thread.
     */
    private fun stopBackgroundThread() {
        backgroundThread?.quitSafely()
        try {
            backgroundThread?.join()
        } catch (e: InterruptedException) {
            Log.e(TAG, "Background thread interrupted", e)
        }
        backgroundThread = null
        backgroundHandler = null
    }
}
