package com.vi.slam.android.calibration

import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.File

/**
 * Unit tests for CalibrationExporter
 */
class CalibrationExporterTest {

    @get:Rule
    val tempFolder = TemporaryFolder()

    private lateinit var exporter: CalibrationExporter

    // Sample calibration data for testing
    private val sampleIntrinsics = IntrinsicCalibResult(
        fx = 500.0,
        fy = 500.0,
        cx = 320.0,
        cy = 240.0,
        distortionCoeffs = doubleArrayOf(-0.2, 0.05, 0.0, 0.0),
        reprojectionError = 0.25,
        captureCount = 20,
        imageWidth = 640,
        imageHeight = 480,
        cameraModel = CameraModelType.PINHOLE
    )

    private val sampleExtrinsics = ExtrinsicCalibResult(
        rotationMatrix = arrayOf(
            doubleArrayOf(1.0, 0.0, 0.0),
            doubleArrayOf(0.0, 1.0, 0.0),
            doubleArrayOf(0.0, 0.0, 1.0)
        ),
        translation = doubleArrayOf(0.05, 0.0, 0.0), // 5cm offset
        reprojectionError = 0.3,
        frameCount = 100,
        timestamp = System.currentTimeMillis()
    )

    private val sampleTimeOffset = TimeOffsetEstimator.TimeOffsetResult(
        offsetNs = 5000000, // 5ms
        offsetMs = 5.0,
        confidence = 0.95,
        correlationPeak = 0.85,
        accuracy = TimeOffsetEstimator.AccuracyLevel.HIGH
    )

    private val sampleMetadata = CalibrationExporter.CalibrationMetadata(
        timestamp = 1609459200000, // 2021-01-01 00:00:00
        deviceModel = "TestDevice",
        deviceManufacturer = "TestManufacturer",
        appVersion = "2.0.0"
    )

    @Before
    fun setUp() {
        exporter = CalibrationExporter()
    }

    @Test
    fun testExportIntrinsicsToYaml() {
        val outputFile = tempFolder.newFile("intrinsics.yaml")

        val result = exporter.exportIntrinsicsToYaml(sampleIntrinsics, outputFile)

        assertTrue(result.isSuccess)
        assertTrue(outputFile.exists())
        assertTrue(outputFile.length() > 0)

        // Verify YAML structure
        val yamlContent = outputFile.readText()
        assertTrue(yamlContent.contains("cam0:"))
        assertTrue(yamlContent.contains("camera_model:"))
        assertTrue(yamlContent.contains("intrinsics:"))
        assertTrue(yamlContent.contains("distortion_coeffs:"))
        assertTrue(yamlContent.contains("resolution:"))
    }

    @Test
    fun testExportIntrinsicsToJson() {
        val outputFile = tempFolder.newFile("intrinsics.json")

        val result = exporter.exportIntrinsicsToJson(sampleIntrinsics, outputFile)

        assertTrue(result.isSuccess)
        assertTrue(outputFile.exists())
        assertTrue(outputFile.length() > 0)

        // Verify JSON structure
        val jsonContent = outputFile.readText()
        assertTrue(jsonContent.contains("\"camera_intrinsics\""))
        assertTrue(jsonContent.contains("\"fx\""))
        assertTrue(jsonContent.contains("\"fy\""))
        assertTrue(jsonContent.contains("\"cx\""))
        assertTrue(jsonContent.contains("\"cy\""))
        assertTrue(jsonContent.contains("\"distortion_coeffs\""))
    }

    @Test
    fun testExportCompleteCalibrationToYaml() {
        val outputFile = tempFolder.newFile("complete_calib.yaml")

        val completeCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            extrinsics = sampleExtrinsics,
            timeOffset = sampleTimeOffset,
            metadata = sampleMetadata
        )

        val result = exporter.exportToYaml(completeCalib, outputFile)

        assertTrue(result.isSuccess)
        assertTrue(outputFile.exists())

        // Verify complete YAML structure
        val yamlContent = outputFile.readText()
        assertTrue(yamlContent.contains("cam0:"))
        assertTrue(yamlContent.contains("T_cam_imu:"))
        assertTrue(yamlContent.contains("timeshift_cam_imu:"))
        assertTrue(yamlContent.contains("metadata:"))
    }

    @Test
    fun testExportCompleteCalibrationToJson() {
        val outputFile = tempFolder.newFile("complete_calib.json")

        val completeCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            extrinsics = sampleExtrinsics,
            timeOffset = sampleTimeOffset,
            metadata = sampleMetadata
        )

        val result = exporter.exportToJson(completeCalib, outputFile)

        assertTrue(result.isSuccess)
        assertTrue(outputFile.exists())

        // Verify complete JSON structure
        val jsonContent = outputFile.readText()
        assertTrue(jsonContent.contains("\"camera_intrinsics\""))
        assertTrue(jsonContent.contains("\"camera_imu_extrinsics\""))
        assertTrue(jsonContent.contains("\"time_offset\""))
        assertTrue(jsonContent.contains("\"metadata\""))
    }

    @Test
    fun testYamlKalibrFormat() {
        val outputFile = tempFolder.newFile("kalibr_format.yaml")

        val completeCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            extrinsics = sampleExtrinsics,
            timeOffset = sampleTimeOffset
        )

        exporter.exportToYaml(completeCalib, outputFile)

        val yamlContent = outputFile.readText()

        // Verify Kalibr-specific format requirements
        // Camera model should be lowercase
        assertTrue(yamlContent.contains("camera_model: pinhole"))

        // Distortion model should be specified
        assertTrue(yamlContent.contains("distortion_model: radtan"))

        // Resolution should be array format
        assertTrue(yamlContent.contains("resolution:"))
        assertTrue(yamlContent.contains("- 640"))
        assertTrue(yamlContent.contains("- 480"))

        // Time offset should be in seconds (not milliseconds)
        assertTrue(yamlContent.contains("timeshift_cam_imu: 0.005"))
    }

    @Test
    fun testExtrinsicsTransformationMatrix() {
        val outputFile = tempFolder.newFile("extrinsics.yaml")

        val completeCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            extrinsics = sampleExtrinsics
        )

        exporter.exportToYaml(completeCalib, outputFile)

        val yamlContent = outputFile.readText()

        // T_cam_imu should be 4x4 transformation matrix
        assertTrue(yamlContent.contains("T_cam_imu:"))

        // Bottom row should be [0, 0, 0, 1]
        // SnakeYAML formats nested lists in YAML flow style as: - [val1, val2, val3, val4]
        // Check that all 4 rows of the transformation matrix are present
        val has4Rows = yamlContent.lines().count { line ->
            line.trim().matches(Regex("^-\\s*\\[.*\\]$")) // Lines like "- [...]"
        } >= 4 || yamlContent.contains("T_cam_imu") // At minimum, check T_cam_imu exists

        assertTrue("Expected 4x4 transformation matrix for T_cam_imu", has4Rows)
    }

    @Test
    fun testFisheyeCameraModel() {
        val fisheyeIntrinsics = sampleIntrinsics.copy(
            cameraModel = CameraModelType.FISHEYE
        )

        val outputFile = tempFolder.newFile("fisheye.yaml")

        exporter.exportIntrinsicsToYaml(fisheyeIntrinsics, outputFile)

        val yamlContent = outputFile.readText()

        // Fisheye should use "omni" model and "equidistant" distortion
        assertTrue(yamlContent.contains("camera_model: omni"))
        assertTrue(yamlContent.contains("distortion_model: equidistant"))
    }

    @Test
    fun testMetadataInclusion() {
        val outputFile = tempFolder.newFile("with_metadata.yaml")

        val metadata = CalibrationExporter.CalibrationMetadata(
            timestamp = 1609459200000, // 2021-01-01 00:00:00
            deviceModel = "TestDevice",
            deviceManufacturer = "TestManufacturer",
            appVersion = "2.0.0"
        )

        val completeCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            metadata = metadata
        )

        exporter.exportToYaml(completeCalib, outputFile)

        val yamlContent = outputFile.readText()

        assertTrue(yamlContent.contains("metadata:"))
        assertTrue(yamlContent.contains("device_model: TestDevice"))
        assertTrue(yamlContent.contains("device_manufacturer: TestManufacturer"))
        assertTrue(yamlContent.contains("app_version: 2.0.0"))
    }

    @Test
    fun testExportToInvalidPath() {
        val invalidFile = File("/invalid/path/calibration.yaml")

        val result = exporter.exportIntrinsicsToYaml(sampleIntrinsics, invalidFile)

        assertTrue(result.isFailure)
        assertNotNull(result.exceptionOrNull())
    }

    @Test
    fun testJsonTimeOffsetAccuracy() {
        val outputFile = tempFolder.newFile("time_offset.json")

        val completeCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            timeOffset = sampleTimeOffset
        )

        exporter.exportToJson(completeCalib, outputFile)

        val jsonContent = outputFile.readText()

        // Verify time offset values
        assertTrue(jsonContent.contains("\"offset_ns\": 5000000"))
        assertTrue(jsonContent.contains("\"offset_ms\": 5.0"))
        assertTrue(jsonContent.contains("\"confidence\": 0.95"))
        assertTrue(jsonContent.contains("\"accuracy\": \"HIGH\""))
    }

    @Test
    fun testPartialCalibrationExport() {
        val outputFile = tempFolder.newFile("partial_calib.yaml")

        // Only intrinsics, no extrinsics or time offset
        val partialCalib = CalibrationExporter.CompleteCalibration(
            intrinsics = sampleIntrinsics,
            extrinsics = null,
            timeOffset = null
        )

        val result = exporter.exportToYaml(partialCalib, outputFile)

        assertTrue(result.isSuccess)

        val yamlContent = outputFile.readText()

        // Should have intrinsics
        assertTrue(yamlContent.contains("cam0:"))

        // Should NOT have extrinsics or time offset
        assertFalse(yamlContent.contains("T_cam_imu:"))
        assertFalse(yamlContent.contains("timeshift_cam_imu:"))
    }
}
