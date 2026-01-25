package com.vi.slam.android.streaming

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * IMU measurement data container.
 *
 * Contains accelerometer and gyroscope measurements with nanosecond-precision timestamp.
 * Designed to match the binary packet format specification:
 * - 8 bytes: timestamp (int64)
 * - 24 bytes: acceleration (3x double, m/s²)
 * - 24 bytes: angular velocity (3x double, rad/s)
 *
 * Total packet size: 56 bytes
 *
 * @property timestampNs Hardware timestamp in nanoseconds
 * @property accelerationX Acceleration along X axis (m/s²)
 * @property accelerationY Acceleration along Y axis (m/s²)
 * @property accelerationZ Acceleration along Z axis (m/s²)
 * @property gyroX Angular velocity around X axis (rad/s)
 * @property gyroY Angular velocity around Y axis (rad/s)
 * @property gyroZ Angular velocity around Z axis (rad/s)
 */
data class ImuData(
    val timestampNs: Long,
    val accelerationX: Double,
    val accelerationY: Double,
    val accelerationZ: Double,
    val gyroX: Double,
    val gyroY: Double,
    val gyroZ: Double
) {
    companion object {
        /**
         * Size of the binary packet in bytes.
         */
        const val PACKET_SIZE = 56

        /**
         * Create ImuData from binary packet.
         *
         * @param packet Binary packet buffer (must be at least PACKET_SIZE bytes)
         * @return Decoded ImuData instance
         * @throws IllegalArgumentException if packet size is insufficient
         */
        fun fromPacket(packet: ByteArray): ImuData {
            require(packet.size >= PACKET_SIZE) {
                "Packet size must be at least $PACKET_SIZE bytes, got ${packet.size}"
            }

            val buffer = ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN)

            return ImuData(
                timestampNs = buffer.getLong(),
                accelerationX = buffer.getDouble(),
                accelerationY = buffer.getDouble(),
                accelerationZ = buffer.getDouble(),
                gyroX = buffer.getDouble(),
                gyroY = buffer.getDouble(),
                gyroZ = buffer.getDouble()
            )
        }
    }

    /**
     * Convert ImuData to binary packet format.
     *
     * Packet format (little-endian):
     * | Offset | Size | Type      | Field          |
     * |--------|------|-----------|----------------|
     * | 0      | 8    | int64     | timestamp_ns   |
     * | 8      | 24   | double[3] | acc (m/s²)     |
     * | 32     | 24   | double[3] | gyro (rad/s)   |
     *
     * @return 56-byte binary packet
     */
    fun toPacket(): ByteArray {
        val buffer = ByteBuffer.allocate(PACKET_SIZE).order(ByteOrder.LITTLE_ENDIAN)

        buffer.putLong(timestampNs)
        buffer.putDouble(accelerationX)
        buffer.putDouble(accelerationY)
        buffer.putDouble(accelerationZ)
        buffer.putDouble(gyroX)
        buffer.putDouble(gyroY)
        buffer.putDouble(gyroZ)

        return buffer.array()
    }

    /**
     * Get acceleration vector as array.
     *
     * @return [x, y, z] in m/s²
     */
    fun accelerationArray(): DoubleArray = doubleArrayOf(
        accelerationX,
        accelerationY,
        accelerationZ
    )

    /**
     * Get gyroscope vector as array.
     *
     * @return [x, y, z] in rad/s
     */
    fun gyroArray(): DoubleArray = doubleArrayOf(
        gyroX,
        gyroY,
        gyroZ
    )
}
