package com.vi.slam.android.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.camera2.CameraAccessException
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.params.StreamConfigurationMap
import android.os.Handler
import android.util.Size
import androidx.core.content.ContextCompat
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

/**
 * Wrapper for Camera2 API providing simplified camera operations.
 *
 * This class abstracts the complexity of Camera2 API and provides
 * a clean interface for camera device management, configuration querying,
 * and capability detection.
 */
class Camera2Wrapper(
    private val context: Context,
    private val cameraManager: CameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
) {

    /**
     * Get list of available camera device IDs.
     *
     * @return List of camera IDs available on the device
     * @throws CameraAccessException if camera service is unavailable
     */
    fun getCameraIdList(): List<String> {
        return try {
            cameraManager.cameraIdList.toList()
        } catch (e: CameraAccessException) {
            throw CameraException("Failed to get camera ID list", e)
        }
    }

    /**
     * Get supported camera configurations for a specific camera.
     *
     * @param cameraId The ID of the camera to query
     * @return List of supported camera configurations
     * @throws CameraException if camera ID is invalid or inaccessible
     */
    fun getSupportedConfigs(cameraId: String): List<CameraConfig> {
        return try {
            val characteristics = cameraManager.getCameraCharacteristics(cameraId)
            val configMap = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                ?: throw CameraException("Stream configuration map not available for camera $cameraId")

            buildConfigList(configMap)
        } catch (e: CameraAccessException) {
            throw CameraException("Failed to get camera characteristics for $cameraId", e)
        } catch (e: IllegalArgumentException) {
            throw CameraException("Invalid camera ID: $cameraId", e)
        }
    }

    /**
     * Check the hardware capability level of a camera.
     *
     * @param cameraId The ID of the camera to check
     * @return The hardware level of the camera
     * @throws CameraException if camera ID is invalid or inaccessible
     */
    fun checkCameraLevel(cameraId: String): HardwareLevel {
        return try {
            val characteristics = cameraManager.getCameraCharacteristics(cameraId)
            val level = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL)
                ?: CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY

            mapHardwareLevel(level)
        } catch (e: CameraAccessException) {
            throw CameraException("Failed to check camera level for $cameraId", e)
        } catch (e: IllegalArgumentException) {
            throw CameraException("Invalid camera ID: $cameraId", e)
        }
    }

    /**
     * Open a camera device asynchronously.
     *
     * This is a suspend function that returns when the camera is successfully opened
     * or throws an exception on failure.
     *
     * @param cameraId The ID of the camera to open
     * @param handler Optional handler for camera callbacks (default: main thread)
     * @return Result containing the opened CameraDevice or error
     * @throws CameraException if camera cannot be opened
     * @throws SecurityException if camera permission is not granted
     */
    suspend fun openCamera(cameraId: String, handler: Handler? = null): Result<CameraDevice> {
        // Check camera permission
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED) {
            return Result.failure(SecurityException("Camera permission not granted"))
        }

        return suspendCancellableCoroutine { continuation ->
            try {
                cameraManager.openCamera(cameraId, object : CameraDevice.StateCallback() {
                    override fun onOpened(camera: CameraDevice) {
                        if (continuation.isActive) {
                            continuation.resume(Result.success(camera))
                        }
                    }

                    override fun onDisconnected(camera: CameraDevice) {
                        camera.close()
                        if (continuation.isActive) {
                            continuation.resumeWithException(
                                CameraException("Camera $cameraId was disconnected")
                            )
                        }
                    }

                    override fun onError(camera: CameraDevice, error: Int) {
                        camera.close()
                        val errorMessage = when (error) {
                            ERROR_CAMERA_IN_USE -> "Camera is already in use"
                            ERROR_MAX_CAMERAS_IN_USE -> "Maximum number of cameras already open"
                            ERROR_CAMERA_DISABLED -> "Camera is disabled by device policy"
                            ERROR_CAMERA_DEVICE -> "Fatal camera device error"
                            ERROR_CAMERA_SERVICE -> "Fatal camera service error"
                            else -> "Unknown camera error: $error"
                        }
                        if (continuation.isActive) {
                            continuation.resumeWithException(
                                CameraException("Failed to open camera $cameraId: $errorMessage")
                            )
                        }
                    }
                }, handler)

                continuation.invokeOnCancellation {
                    // Cleanup if coroutine is cancelled before camera opens
                }
            } catch (e: CameraAccessException) {
                continuation.resumeWithException(
                    CameraException("Camera access exception for $cameraId", e)
                )
            } catch (e: IllegalArgumentException) {
                continuation.resumeWithException(
                    CameraException("Invalid camera ID: $cameraId", e)
                )
            }
        }
    }

    /**
     * Build list of supported camera configurations from stream configuration map.
     */
    private fun buildConfigList(configMap: StreamConfigurationMap): List<CameraConfig> {
        val configs = mutableListOf<CameraConfig>()

        // Get YUV_420_888 configurations
        val yuvSizes = configMap.getOutputSizes(android.graphics.ImageFormat.YUV_420_888) ?: emptyArray()
        for (size in yuvSizes) {
            val fpsRanges = configMap.getHighSpeedVideoFpsRangesFor(size) ?: emptyArray()
            if (fpsRanges.isNotEmpty()) {
                for (fpsRange in fpsRanges) {
                    configs.add(
                        CameraConfig(
                            resolution = size,
                            fps = fpsRange.upper,
                            format = ImageFormat.YUV_420_888,
                            exposureMode = ExposureMode.AUTO
                        )
                    )
                }
            } else {
                // Default to 30 FPS for standard configurations
                configs.add(
                    CameraConfig(
                        resolution = size,
                        fps = 30,
                        format = ImageFormat.YUV_420_888,
                        exposureMode = ExposureMode.AUTO
                    )
                )
            }
        }

        // Get JPEG configurations
        val jpegSizes = configMap.getOutputSizes(android.graphics.ImageFormat.JPEG) ?: emptyArray()
        for (size in jpegSizes) {
            configs.add(
                CameraConfig(
                    resolution = size,
                    fps = 30,
                    format = ImageFormat.JPEG,
                    exposureMode = ExposureMode.AUTO
                )
            )
        }

        return configs
    }

    /**
     * Map Camera2 hardware level constant to our enum.
     */
    private fun mapHardwareLevel(level: Int): HardwareLevel {
        return when (level) {
            CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY -> HardwareLevel.LEGACY
            CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED -> HardwareLevel.LIMITED
            CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL -> HardwareLevel.FULL
            CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3 -> HardwareLevel.LEVEL_3
            CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL -> HardwareLevel.EXTERNAL
            else -> HardwareLevel.LEGACY
        }
    }
}

/**
 * Exception thrown by Camera2Wrapper operations.
 */
class CameraException(message: String, cause: Throwable? = null) : Exception(message, cause)
