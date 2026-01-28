package com.vi.slam.android.calibration

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import org.yaml.snakeyaml.DumperOptions
import org.yaml.snakeyaml.Yaml
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.*

/**
 * Exports calibration results to YAML (Kalibr-compatible) and JSON formats
 *
 * Supports exporting:
 * - Camera intrinsic parameters
 * - Camera-IMU extrinsic parameters
 * - Time offset between camera and IMU
 *
 * The YAML format follows Kalibr calibration tool conventions for
 * compatibility with popular VIO/SLAM frameworks (OpenVINS, VINS-Mono, etc.)
 */
class CalibrationExporter {

    private val gson: Gson = GsonBuilder()
        .setPrettyPrinting()
        .serializeNulls()
        .create()

    private val yaml: Yaml = run {
        val options = DumperOptions().apply {
            defaultFlowStyle = DumperOptions.FlowStyle.BLOCK
            isPrettyFlow = true
            indent = 2
        }
        Yaml(options)
    }

    /**
     * Complete calibration data
     */
    data class CompleteCalibration(
        val intrinsics: IntrinsicCalibResult,
        val extrinsics: ExtrinsicCalibResult? = null,
        val timeOffset: TimeOffsetEstimator.TimeOffsetResult? = null,
        val metadata: CalibrationMetadata = CalibrationMetadata()
    )

    /**
     * Calibration metadata
     */
    data class CalibrationMetadata(
        val timestamp: Long = System.currentTimeMillis(),
        val deviceModel: String = android.os.Build.MODEL,
        val deviceManufacturer: String = android.os.Build.MANUFACTURER,
        val appVersion: String = "1.0.0"
    )

    /**
     * Export complete calibration to YAML (Kalibr format)
     *
     * @param calibration Complete calibration data
     * @param outputFile Output file path
     * @return Success or failure result
     */
    fun exportToYaml(
        calibration: CompleteCalibration,
        outputFile: File
    ): Result<Unit> {
        return try {
            val yamlData = buildKalibrYamlData(calibration)

            FileWriter(outputFile).use { writer ->
                yaml.dump(yamlData, writer)
            }

            Result.success(Unit)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Export complete calibration to JSON
     *
     * @param calibration Complete calibration data
     * @param outputFile Output file path
     * @return Success or failure result
     */
    fun exportToJson(
        calibration: CompleteCalibration,
        outputFile: File
    ): Result<Unit> {
        return try {
            val jsonData = buildJsonData(calibration)

            FileWriter(outputFile).use { writer ->
                gson.toJson(jsonData, writer)
            }

            Result.success(Unit)
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    /**
     * Build Kalibr-compatible YAML data structure
     *
     * Kalibr format reference:
     * https://github.com/ethz-asl/kalibr/wiki/yaml-formats
     */
    private fun buildKalibrYamlData(calibration: CompleteCalibration): Map<String, Any> {
        val data = mutableMapOf<String, Any>()

        // Camera intrinsics (cam0)
        data["cam0"] = buildCameraIntrinsicsYaml(calibration.intrinsics)

        // Camera-IMU extrinsics (T_cam_imu)
        if (calibration.extrinsics != null) {
            data["T_cam_imu"] = buildExtrinsicsYaml(calibration.extrinsics)
        }

        // Time offset (camera - IMU)
        if (calibration.timeOffset != null) {
            data["timeshift_cam_imu"] = calibration.timeOffset.offsetMs / 1000.0 // Convert to seconds
        }

        // Metadata
        data["metadata"] = buildMetadataYaml(calibration.metadata)

        return data
    }

    /**
     * Build camera intrinsics section for YAML
     */
    private fun buildCameraIntrinsicsYaml(intrinsics: IntrinsicCalibResult): Map<String, Any> {
        val cameraData = mutableMapOf<String, Any>()

        // Camera model
        cameraData["camera_model"] = when (intrinsics.cameraModel) {
            CameraModelType.PINHOLE -> "pinhole"
            CameraModelType.FISHEYE -> "omni"
        }

        // Intrinsics vector [fx, fy, cx, cy]
        cameraData["intrinsics"] = listOf(
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.cx,
            intrinsics.cy
        )

        // Distortion model and coefficients
        val distortionModel = when (intrinsics.cameraModel) {
            CameraModelType.PINHOLE -> "radtan" // radial-tangential
            CameraModelType.FISHEYE -> "equidistant"
        }
        cameraData["distortion_model"] = distortionModel
        cameraData["distortion_coeffs"] = intrinsics.distortionCoeffs.toList()

        // Image resolution
        cameraData["resolution"] = listOf(intrinsics.imageWidth, intrinsics.imageHeight)

        // Reprojection error
        cameraData["reprojection_error"] = intrinsics.reprojectionError

        return cameraData
    }

    /**
     * Build extrinsics section for YAML
     *
     * T_cam_imu is a 4x4 transformation matrix:
     * [R | t]
     * [0 | 1]
     */
    private fun buildExtrinsicsYaml(extrinsics: ExtrinsicCalibResult): List<List<Double>> {
        val T = mutableListOf<List<Double>>()

        // Add rotation and translation (first 3 rows)
        for (i in 0..2) {
            val row = mutableListOf<Double>()
            row.addAll(extrinsics.rotationMatrix[i].toList())
            row.add(extrinsics.translation[i])
            T.add(row)
        }

        // Add bottom row [0, 0, 0, 1]
        T.add(listOf(0.0, 0.0, 0.0, 1.0))

        return T
    }

    /**
     * Build metadata section for YAML
     */
    private fun buildMetadataYaml(metadata: CalibrationMetadata): Map<String, Any> {
        val dateFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US)

        return mapOf(
            "calibration_date" to dateFormat.format(Date(metadata.timestamp)),
            "device_model" to metadata.deviceModel,
            "device_manufacturer" to metadata.deviceManufacturer,
            "app_version" to metadata.appVersion
        )
    }

    /**
     * Build JSON data structure
     */
    private fun buildJsonData(calibration: CompleteCalibration): Map<String, Any?> {
        val data = mutableMapOf<String, Any?>()

        // Camera intrinsics
        data["camera_intrinsics"] = mapOf(
            "fx" to calibration.intrinsics.fx,
            "fy" to calibration.intrinsics.fy,
            "cx" to calibration.intrinsics.cx,
            "cy" to calibration.intrinsics.cy,
            "distortion_coeffs" to calibration.intrinsics.distortionCoeffs.toList(),
            "camera_model" to calibration.intrinsics.cameraModel.name,
            "image_width" to calibration.intrinsics.imageWidth,
            "image_height" to calibration.intrinsics.imageHeight,
            "reprojection_error" to calibration.intrinsics.reprojectionError,
            "capture_count" to calibration.intrinsics.captureCount
        )

        // Camera-IMU extrinsics
        if (calibration.extrinsics != null) {
            data["camera_imu_extrinsics"] = mapOf(
                "rotation_matrix" to calibration.extrinsics.rotationMatrix.map { it.toList() },
                "translation" to calibration.extrinsics.translation.toList(),
                "reprojection_error" to calibration.extrinsics.reprojectionError,
                "frame_count" to calibration.extrinsics.frameCount,
                "timestamp" to calibration.extrinsics.timestamp
            )
        }

        // Time offset
        if (calibration.timeOffset != null) {
            data["time_offset"] = mapOf(
                "offset_ns" to calibration.timeOffset.offsetNs,
                "offset_ms" to calibration.timeOffset.offsetMs,
                "confidence" to calibration.timeOffset.confidence,
                "correlation_peak" to calibration.timeOffset.correlationPeak,
                "accuracy" to calibration.timeOffset.accuracy.name
            )
        }

        // Metadata
        data["metadata"] = mapOf(
            "timestamp" to calibration.metadata.timestamp,
            "device_model" to calibration.metadata.deviceModel,
            "device_manufacturer" to calibration.metadata.deviceManufacturer,
            "app_version" to calibration.metadata.appVersion
        )

        return data
    }

    /**
     * Export intrinsics only to YAML
     */
    fun exportIntrinsicsToYaml(
        intrinsics: IntrinsicCalibResult,
        outputFile: File
    ): Result<Unit> {
        val calibration = CompleteCalibration(
            intrinsics = intrinsics,
            extrinsics = null,
            timeOffset = null
        )
        return exportToYaml(calibration, outputFile)
    }

    /**
     * Export intrinsics only to JSON
     */
    fun exportIntrinsicsToJson(
        intrinsics: IntrinsicCalibResult,
        outputFile: File
    ): Result<Unit> {
        val calibration = CompleteCalibration(
            intrinsics = intrinsics,
            extrinsics = null,
            timeOffset = null
        )
        return exportToJson(calibration, outputFile)
    }
}
