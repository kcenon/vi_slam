package com.vi.slam.android.recorder

import android.util.Log
import java.io.BufferedReader
import java.io.BufferedWriter
import java.io.File
import java.io.FileReader
import java.io.FileWriter

/**
 * Utility for validating and repairing IMU CSV files.
 *
 * Handles recovery of interrupted CSV writes, including:
 * - Header validation
 * - Truncated line detection and removal
 * - Empty line removal
 * - Data format validation
 */
object CsvRecovery {

    private const val TAG = "CsvRecovery"
    private const val CSV_HEADER = "timestamp_ns,sensor_type,x,y,z"
    private const val EXPECTED_COLUMN_COUNT = 5

    /**
     * Result of CSV validation and repair operation.
     */
    data class CsvRecoveryResult(
        val success: Boolean,
        val totalLines: Long,
        val validLines: Long,
        val removedLines: Long,
        val errorMessage: String? = null
    )

    /**
     * Validate and repair an IMU CSV file.
     *
     * Operations performed:
     * 1. Check header exists and matches expected format
     * 2. Remove truncated lines (incomplete data)
     * 3. Remove empty lines
     * 4. Validate data format (timestamp, sensor type, numeric values)
     *
     * The original file is replaced with the repaired version.
     *
     * @param csvFile CSV file to validate and repair
     * @return CsvRecoveryResult with statistics
     */
    fun validateAndRepair(csvFile: File): CsvRecoveryResult {
        if (!csvFile.exists()) {
            return CsvRecoveryResult(
                success = false,
                totalLines = 0,
                validLines = 0,
                removedLines = 0,
                errorMessage = "CSV file does not exist: ${csvFile.absolutePath}"
            )
        }

        try {
            val tempFile = File.createTempFile("imu_repair_", ".csv", csvFile.parentFile)
            var totalLines = 0L
            var validLines = 0L
            var removedLines = 0L
            var headerValid = false

            BufferedReader(FileReader(csvFile)).use { reader ->
                BufferedWriter(FileWriter(tempFile)).use { writer ->
                    // Process header
                    val header = reader.readLine()
                    totalLines++

                    if (header == null || header.trim() != CSV_HEADER) {
                        Log.w(TAG, "Invalid or missing CSV header, adding correct header")
                        writer.write("$CSV_HEADER\n")
                        headerValid = true
                        validLines++

                        // If header was present but wrong, skip it and continue
                        if (header != null) {
                            removedLines++
                        }
                    } else {
                        // Header is valid
                        writer.write("$header\n")
                        headerValid = true
                        validLines++
                    }

                    // Process data lines
                    reader.forEachLine { line ->
                        totalLines++

                        // Skip empty lines
                        if (line.trim().isEmpty()) {
                            removedLines++
                            return@forEachLine
                        }

                        // Validate line format
                        if (isValidCsvLine(line)) {
                            writer.write("$line\n")
                            validLines++
                        } else {
                            Log.d(TAG, "Removing invalid line: $line")
                            removedLines++
                        }
                    }
                }
            }

            // Replace original file with repaired version
            if (!csvFile.delete()) {
                Log.w(TAG, "Failed to delete original CSV file")
            }

            if (!tempFile.renameTo(csvFile)) {
                Log.e(TAG, "Failed to rename repaired CSV file")
                return CsvRecoveryResult(
                    success = false,
                    totalLines = totalLines,
                    validLines = validLines,
                    removedLines = removedLines,
                    errorMessage = "Failed to replace original file with repaired version"
                )
            }

            Log.i(
                TAG,
                "CSV repair complete: total=$totalLines, valid=$validLines, removed=$removedLines"
            )

            return CsvRecoveryResult(
                success = true,
                totalLines = totalLines,
                validLines = validLines,
                removedLines = removedLines
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to validate and repair CSV file", e)
            return CsvRecoveryResult(
                success = false,
                totalLines = 0,
                validLines = 0,
                removedLines = 0,
                errorMessage = "Exception during repair: ${e.message}"
            )
        }
    }

    /**
     * Validate CSV line format.
     *
     * Expected format: timestamp_ns,sensor_type,x,y,z
     * - timestamp_ns: Long integer
     * - sensor_type: "accel" or "gyro"
     * - x, y, z: Float/Double values
     *
     * @param line CSV line to validate
     * @return True if line is valid, false otherwise
     */
    private fun isValidCsvLine(line: String): Boolean {
        val parts = line.split(",")

        // Check column count
        if (parts.size != EXPECTED_COLUMN_COUNT) {
            return false
        }

        // Validate timestamp (must be Long)
        val timestamp = parts[0].trim().toLongOrNull() ?: return false
        if (timestamp <= 0) {
            return false
        }

        // Validate sensor type
        val sensorType = parts[1].trim()
        if (sensorType != "accel" && sensorType != "gyro") {
            return false
        }

        // Validate x, y, z (must be numeric)
        for (i in 2..4) {
            parts[i].trim().toDoubleOrNull() ?: return false
        }

        return true
    }

    /**
     * Count valid data lines in CSV file (excluding header).
     *
     * @param csvFile CSV file to count
     * @return Number of valid data lines, or 0 if file cannot be read
     */
    fun countValidLines(csvFile: File): Long {
        if (!csvFile.exists()) {
            return 0
        }

        return try {
            BufferedReader(FileReader(csvFile)).use { reader ->
                var count = 0L
                var firstLine = true

                reader.forEachLine { line ->
                    // Skip header
                    if (firstLine) {
                        firstLine = false
                        return@forEachLine
                    }

                    // Count valid lines
                    if (line.trim().isNotEmpty() && isValidCsvLine(line)) {
                        count++
                    }
                }

                count
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to count valid lines in CSV", e)
            0
        }
    }
}
