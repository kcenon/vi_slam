package com.vi.slam.android.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.camera2.CameraAccessException
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.params.StreamConfigurationMap
import android.util.Size
import kotlinx.coroutines.test.runTest
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Ignore
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.kotlin.any
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.doThrow
import org.mockito.kotlin.mock
import org.mockito.kotlin.verify
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
@Config(manifest = Config.NONE, sdk = [28])
class Camera2WrapperTest {

    private lateinit var mockContext: Context
    private lateinit var mockCameraManager: CameraManager
    private lateinit var camera2Wrapper: Camera2Wrapper

    @Before
    fun setup() {
        mockContext = mock()
        mockCameraManager = mock()
        camera2Wrapper = Camera2Wrapper(mockContext, mockCameraManager)
    }

    @Test
    fun `getCameraIdList returns list of camera IDs`() {
        // Given
        val expectedIds = arrayOf("0", "1", "2")
        whenever(mockCameraManager.cameraIdList).thenReturn(expectedIds)

        // When
        val result = camera2Wrapper.getCameraIdList()

        // Then
        assertEquals(3, result.size)
        assertEquals(listOf("0", "1", "2"), result)
        verify(mockCameraManager).cameraIdList
    }

    @Test(expected = CameraException::class)
    fun `getCameraIdList throws CameraException on access error`() {
        // Given
        whenever(mockCameraManager.cameraIdList)
            .doThrow(CameraAccessException(CameraAccessException.CAMERA_ERROR))

        // When
        camera2Wrapper.getCameraIdList()

        // Then - expect exception
    }

    @Test
    fun `getSupportedConfigs returns configurations for valid camera`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()
        val mockConfigMap: StreamConfigurationMap = mock()

        val yuvSizes = arrayOf(Size(1920, 1080), Size(1280, 720))
        val jpegSizes = arrayOf(Size(4000, 3000))

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP))
            .thenReturn(mockConfigMap)
        whenever(mockConfigMap.getOutputSizes(android.graphics.ImageFormat.YUV_420_888))
            .thenReturn(yuvSizes)
        whenever(mockConfigMap.getOutputSizes(android.graphics.ImageFormat.JPEG))
            .thenReturn(jpegSizes)
        whenever(mockConfigMap.getHighSpeedVideoFpsRangesFor(any()))
            .thenReturn(null)

        // When
        val configs = camera2Wrapper.getSupportedConfigs(cameraId)

        // Then
        assertNotNull(configs)
        assertTrue(configs.isNotEmpty())
        assertEquals(3, configs.size) // 2 YUV + 1 JPEG

        // Verify YUV configs
        val yuvConfigs = configs.filter { it.format == ImageFormat.YUV_420_888 }
        assertEquals(2, yuvConfigs.size)
        assertTrue(yuvConfigs.any { it.resolution == Size(1920, 1080) })
        assertTrue(yuvConfigs.any { it.resolution == Size(1280, 720) })

        // Verify JPEG config
        val jpegConfigs = configs.filter { it.format == ImageFormat.JPEG }
        assertEquals(1, jpegConfigs.size)
        assertEquals(Size(4000, 3000), jpegConfigs[0].resolution)
    }

    @Test(expected = CameraException::class)
    fun `getSupportedConfigs throws exception for invalid camera ID`() {
        // Given
        val invalidCameraId = "invalid"
        whenever(mockCameraManager.getCameraCharacteristics(invalidCameraId))
            .doThrow(IllegalArgumentException("Invalid camera ID"))

        // When
        camera2Wrapper.getSupportedConfigs(invalidCameraId)

        // Then - expect exception
    }

    @Test(expected = CameraException::class)
    fun `getSupportedConfigs throws exception when config map unavailable`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP))
            .thenReturn(null)

        // When
        camera2Wrapper.getSupportedConfigs(cameraId)

        // Then - expect exception
    }

    @Test
    fun `checkCameraLevel returns correct hardware level for LEGACY`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL))
            .thenReturn(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY)

        // When
        val level = camera2Wrapper.checkCameraLevel(cameraId)

        // Then
        assertEquals(HardwareLevel.LEGACY, level)
    }

    @Test
    fun `checkCameraLevel returns correct hardware level for LIMITED`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL))
            .thenReturn(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED)

        // When
        val level = camera2Wrapper.checkCameraLevel(cameraId)

        // Then
        assertEquals(HardwareLevel.LIMITED, level)
    }

    @Test
    fun `checkCameraLevel returns correct hardware level for FULL`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL))
            .thenReturn(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL)

        // When
        val level = camera2Wrapper.checkCameraLevel(cameraId)

        // Then
        assertEquals(HardwareLevel.FULL, level)
    }

    @Test
    fun `checkCameraLevel returns correct hardware level for LEVEL_3`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL))
            .thenReturn(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3)

        // When
        val level = camera2Wrapper.checkCameraLevel(cameraId)

        // Then
        assertEquals(HardwareLevel.LEVEL_3, level)
    }

    @Test
    fun `checkCameraLevel returns LEGACY when level is null`() {
        // Given
        val cameraId = "0"
        val mockCharacteristics: CameraCharacteristics = mock()

        whenever(mockCameraManager.getCameraCharacteristics(cameraId))
            .thenReturn(mockCharacteristics)
        whenever(mockCharacteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL))
            .thenReturn(null)

        // When
        val level = camera2Wrapper.checkCameraLevel(cameraId)

        // Then
        assertEquals(HardwareLevel.LEGACY, level)
    }

    @Test(expected = CameraException::class)
    fun `checkCameraLevel throws exception for invalid camera ID`() {
        // Given
        val invalidCameraId = "invalid"
        whenever(mockCameraManager.getCameraCharacteristics(invalidCameraId))
            .doThrow(IllegalArgumentException("Invalid camera ID"))

        // When
        camera2Wrapper.checkCameraLevel(invalidCameraId)

        // Then - expect exception
    }

    @Ignore("Permission checking requires ContextCompat static mocking - better tested in instrumentation tests")
    @Test
    fun `openCamera returns failure when permission not granted`() = runTest {
        // This test requires static mocking of ContextCompat which is complex in unit tests.
        // The permission check functionality should be covered by Android instrumentation tests instead.
    }
}
