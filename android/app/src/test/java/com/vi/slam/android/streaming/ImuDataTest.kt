package com.vi.slam.android.streaming

import org.junit.Assert.*
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder

class ImuDataTest {

    @Test
    fun testPacketSize() {
        assertEquals(56, ImuData.PACKET_SIZE)
    }

    @Test
    fun testToPacket() {
        val imuData = ImuData(
            timestampNs = 1234567890123456L,
            accelerationX = 1.0,
            accelerationY = 2.0,
            accelerationZ = 3.0,
            gyroX = 0.1,
            gyroY = 0.2,
            gyroZ = 0.3
        )

        val packet = imuData.toPacket()

        assertEquals(ImuData.PACKET_SIZE, packet.size)

        // Verify packet contents
        val buffer = ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN)

        assertEquals(1234567890123456L, buffer.getLong())
        assertEquals(1.0, buffer.getDouble(), 0.0001)
        assertEquals(2.0, buffer.getDouble(), 0.0001)
        assertEquals(3.0, buffer.getDouble(), 0.0001)
        assertEquals(0.1, buffer.getDouble(), 0.0001)
        assertEquals(0.2, buffer.getDouble(), 0.0001)
        assertEquals(0.3, buffer.getDouble(), 0.0001)
    }

    @Test
    fun testFromPacket() {
        val buffer = ByteBuffer.allocate(ImuData.PACKET_SIZE).order(ByteOrder.LITTLE_ENDIAN)
        buffer.putLong(9876543210987654L)
        buffer.putDouble(5.0)
        buffer.putDouble(6.0)
        buffer.putDouble(7.0)
        buffer.putDouble(0.5)
        buffer.putDouble(0.6)
        buffer.putDouble(0.7)

        val imuData = ImuData.fromPacket(buffer.array())

        assertEquals(9876543210987654L, imuData.timestampNs)
        assertEquals(5.0, imuData.accelerationX, 0.0001)
        assertEquals(6.0, imuData.accelerationY, 0.0001)
        assertEquals(7.0, imuData.accelerationZ, 0.0001)
        assertEquals(0.5, imuData.gyroX, 0.0001)
        assertEquals(0.6, imuData.gyroY, 0.0001)
        assertEquals(0.7, imuData.gyroZ, 0.0001)
    }

    @Test
    fun testRoundTrip() {
        val original = ImuData(
            timestampNs = System.nanoTime(),
            accelerationX = 9.81,
            accelerationY = 0.0,
            accelerationZ = 0.0,
            gyroX = 0.0,
            gyroY = 0.0,
            gyroZ = 1.57
        )

        val packet = original.toPacket()
        val decoded = ImuData.fromPacket(packet)

        assertEquals(original.timestampNs, decoded.timestampNs)
        assertEquals(original.accelerationX, decoded.accelerationX, 0.0001)
        assertEquals(original.accelerationY, decoded.accelerationY, 0.0001)
        assertEquals(original.accelerationZ, decoded.accelerationZ, 0.0001)
        assertEquals(original.gyroX, decoded.gyroX, 0.0001)
        assertEquals(original.gyroY, decoded.gyroY, 0.0001)
        assertEquals(original.gyroZ, decoded.gyroZ, 0.0001)
    }

    @Test(expected = IllegalArgumentException::class)
    fun testFromPacketInvalidSize() {
        val tooSmall = ByteArray(50)
        ImuData.fromPacket(tooSmall)
    }

    @Test
    fun testAccelerationArray() {
        val imuData = ImuData(
            timestampNs = 0,
            accelerationX = 1.0,
            accelerationY = 2.0,
            accelerationZ = 3.0,
            gyroX = 0.0,
            gyroY = 0.0,
            gyroZ = 0.0
        )

        val acc = imuData.accelerationArray()

        assertArrayEquals(doubleArrayOf(1.0, 2.0, 3.0), acc, 0.0001)
    }

    @Test
    fun testGyroArray() {
        val imuData = ImuData(
            timestampNs = 0,
            accelerationX = 0.0,
            accelerationY = 0.0,
            accelerationZ = 0.0,
            gyroX = 0.1,
            gyroY = 0.2,
            gyroZ = 0.3
        )

        val gyro = imuData.gyroArray()

        assertArrayEquals(doubleArrayOf(0.1, 0.2, 0.3), gyro, 0.0001)
    }

    @Test
    fun testLittleEndianByteOrder() {
        val imuData = ImuData(
            timestampNs = 0x0102030405060708L,
            accelerationX = 0.0,
            accelerationY = 0.0,
            accelerationZ = 0.0,
            gyroX = 0.0,
            gyroY = 0.0,
            gyroZ = 0.0
        )

        val packet = imuData.toPacket()

        // Little endian: least significant byte first
        assertEquals(0x08.toByte(), packet[0])
        assertEquals(0x07.toByte(), packet[1])
        assertEquals(0x06.toByte(), packet[2])
        assertEquals(0x05.toByte(), packet[3])
    }

    @Test
    fun testZeroValues() {
        val imuData = ImuData(
            timestampNs = 0,
            accelerationX = 0.0,
            accelerationY = 0.0,
            accelerationZ = 0.0,
            gyroX = 0.0,
            gyroY = 0.0,
            gyroZ = 0.0
        )

        val packet = imuData.toPacket()
        val decoded = ImuData.fromPacket(packet)

        assertEquals(imuData, decoded)
    }

    @Test
    fun testNegativeValues() {
        val imuData = ImuData(
            timestampNs = 0,
            accelerationX = -9.81,
            accelerationY = -1.23,
            accelerationZ = -4.56,
            gyroX = -0.1,
            gyroY = -0.2,
            gyroZ = -0.3
        )

        val packet = imuData.toPacket()
        val decoded = ImuData.fromPacket(packet)

        assertEquals(imuData.accelerationX, decoded.accelerationX, 0.0001)
        assertEquals(imuData.accelerationY, decoded.accelerationY, 0.0001)
        assertEquals(imuData.accelerationZ, decoded.accelerationZ, 0.0001)
        assertEquals(imuData.gyroX, decoded.gyroX, 0.0001)
        assertEquals(imuData.gyroY, decoded.gyroY, 0.0001)
        assertEquals(imuData.gyroZ, decoded.gyroZ, 0.0001)
    }
}
