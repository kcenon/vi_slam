package com.vi.slam.android.streaming

import kotlinx.coroutines.test.runTest
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

class UdpImuStreamerTest {

    private lateinit var streamer: UdpImuStreamer
    private lateinit var receiverSocket: DatagramSocket
    private val testPort = 15000

    @Before
    fun setup() {
        streamer = UdpImuStreamer("127.0.0.1", testPort)
        receiverSocket = DatagramSocket(testPort)
        receiverSocket.soTimeout = 2000 // 2 second timeout
    }

    @After
    fun teardown() = runTest {
        if (streamer.isStreaming()) {
            streamer.stop()
        }
        receiverSocket.close()
    }

    @Test
    fun testStartStop() = runTest {
        assertFalse(streamer.isStreaming())

        streamer.start()
        assertTrue(streamer.isStreaming())

        streamer.stop()
        assertFalse(streamer.isStreaming())
    }

    @Test
    fun testSendImuData() = runTest {
        streamer.start()

        val imuData = ImuData(
            timestampNs = 1234567890L,
            accelerationX = 1.0,
            accelerationY = 2.0,
            accelerationZ = 3.0,
            gyroX = 0.1,
            gyroY = 0.2,
            gyroZ = 0.3
        )

        streamer.sendImuData(imuData)

        // Receive packet
        val buffer = ByteArray(ImuData.PACKET_SIZE)
        val packet = DatagramPacket(buffer, buffer.size)
        receiverSocket.receive(packet)

        assertEquals(ImuData.PACKET_SIZE, packet.length)

        // Decode and verify
        val received = ImuData.fromPacket(buffer)
        assertEquals(imuData.timestampNs, received.timestampNs)
        assertEquals(imuData.accelerationX, received.accelerationX, 0.0001)
        assertEquals(imuData.gyroX, received.gyroX, 0.0001)
    }

    @Test
    fun testMultiplePackets() = runTest {
        streamer.start()

        val packets = 10
        for (i in 0 until packets) {
            val imuData = ImuData(
                timestampNs = i.toLong(),
                accelerationX = i.toDouble(),
                accelerationY = 0.0,
                accelerationZ = 0.0,
                gyroX = 0.0,
                gyroY = 0.0,
                gyroZ = 0.0
            )
            streamer.sendImuData(imuData)
        }

        // Verify all packets received
        val buffer = ByteArray(ImuData.PACKET_SIZE)
        for (i in 0 until packets) {
            val packet = DatagramPacket(buffer, buffer.size)
            receiverSocket.receive(packet)

            val received = ImuData.fromPacket(buffer)
            assertEquals(i.toLong(), received.timestampNs)
        }
    }

    @Test(expected = IllegalStateException::class)
    fun testSendWithoutStart() = runTest {
        val imuData = ImuData(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        streamer.sendImuData(imuData)
    }

    @Test
    fun testStatistics() = runTest {
        streamer.start()

        // Send some data
        val imuData = ImuData(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        streamer.sendImuData(imuData)
        streamer.sendImuData(imuData)
        streamer.sendImuData(imuData)

        val stats = streamer.getStatistics()

        assertEquals(3, stats.packetsSent)
        assertEquals(3L * ImuData.PACKET_SIZE, stats.bytesSent)
        assertTrue(stats.elapsedSeconds >= 0)
    }

    @Test
    fun testRestart() = runTest {
        streamer.start()
        assertTrue(streamer.isStreaming())

        streamer.stop()
        assertFalse(streamer.isStreaming())

        streamer.start()
        assertTrue(streamer.isStreaming())
    }

    @Test
    fun testDoubleStart() = runTest {
        streamer.start()
        assertTrue(streamer.isStreaming())

        // Second start should not throw, just warn
        streamer.start()
        assertTrue(streamer.isStreaming())
    }

    @Test
    fun testDoubleStop() = runTest {
        streamer.start()
        streamer.stop()
        assertFalse(streamer.isStreaming())

        // Second stop should not throw, just warn
        streamer.stop()
        assertFalse(streamer.isStreaming())
    }

    @Test
    fun testHighFrequencySending() = runTest {
        streamer.start()

        val startTime = System.nanoTime()
        val targetPackets = 200 // 200 packets
        val imuData = ImuData(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        repeat(targetPackets) {
            streamer.sendImuData(imuData)
        }

        val elapsed = (System.nanoTime() - startTime) / 1_000_000_000.0
        val actualRate = targetPackets / elapsed

        // Should achieve at least 100 Hz (target is 200 Hz)
        assertTrue("Actual rate: $actualRate Hz", actualRate >= 100.0)

        val stats = streamer.getStatistics()
        assertEquals(targetPackets.toLong(), stats.packetsSent)
    }

    @Test
    fun testStatisticsReset() = runTest {
        streamer.start()

        val imuData = ImuData(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        streamer.sendImuData(imuData)

        val stats1 = streamer.getStatistics()
        assertEquals(1, stats1.packetsSent)

        streamer.stop()
        streamer.start()

        val stats2 = streamer.getStatistics()
        assertEquals(0, stats2.packetsSent) // Should reset after restart
    }
}
