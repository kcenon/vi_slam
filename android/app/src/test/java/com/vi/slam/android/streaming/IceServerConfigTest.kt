package com.vi.slam.android.streaming

import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class IceServerConfigTest {

    @Test
    fun `Default servers contain Google STUN servers`() {
        val servers = IceServerConfig.getDefaultServers()

        assertTrue("Should have at least one server", servers.isNotEmpty())
        assertEquals("Should have exactly 2 default servers", 2, servers.size)

        val uris = servers.flatMap { it.urls }
        assertTrue("Should contain Google STUN server",
            uris.any { it.contains("stun.l.google.com") })
        assertTrue("Should contain Google STUN1 server",
            uris.any { it.contains("stun1.l.google.com") })
    }

    @Test
    fun `Create custom STUN server`() {
        val customUri = "stun:custom.example.com:3478"
        val servers = IceServerConfig.createCustomServers(listOf(customUri))

        assertEquals(1, servers.size)

        val server = servers.first()
        assertTrue("Should contain custom URI", server.urls.contains(customUri))
    }

    @Test
    fun `Create custom TURN server with credentials`() {
        val turnUri = "turn:turn.example.com:3478"
        val username = "testuser"
        val password = "testpass"

        val servers = IceServerConfig.createCustomServers(
            listOf(turnUri),
            username,
            password
        )

        assertEquals(1, servers.size)

        val server = servers.first()
        assertTrue("Should contain TURN URI", server.urls.contains(turnUri))
        assertEquals("Username should match", username, server.username)
        assertEquals("Password should match", password, server.password)
    }

    @Test
    fun `Create multiple custom servers`() {
        val uris = listOf(
            "stun:stun1.example.com:3478",
            "stun:stun2.example.com:3478",
            "turn:turn.example.com:3478"
        )

        val servers = IceServerConfig.createCustomServers(uris)

        assertEquals(1, servers.size)

        val server = servers.first()
        assertEquals("Should contain all URIs", uris.size, server.urls.size)
        uris.forEach { uri ->
            assertTrue("Should contain URI: $uri", server.urls.contains(uri))
        }
    }

    @Test
    fun `Production servers include both STUN and TURN`() {
        val turnUri = "turn:production.turn.com:3478?transport=udp"
        val username = "produser"
        val password = "prodpass"

        val servers = IceServerConfig.getProductionServers(turnUri, username, password)

        assertTrue("Should have multiple servers", servers.size > 2)

        // Check for default STUN servers
        val stunServers = servers.filter { server ->
            server.urls.any { it.contains("stun.l.google.com") }
        }
        assertTrue("Should contain default STUN servers", stunServers.isNotEmpty())

        // Check for custom TURN server
        val turnServers = servers.filter { server ->
            server.urls.contains(turnUri)
        }
        assertEquals("Should have one TURN server", 1, turnServers.size)

        val turnServer = turnServers.first()
        assertEquals("TURN username should match", username, turnServer.username)
        assertEquals("TURN password should match", password, turnServer.password)
    }

    @Test
    fun `Custom servers without credentials have null username and password`() {
        val uri = "stun:stun.example.com:3478"
        val servers = IceServerConfig.createCustomServers(listOf(uri))

        val server = servers.first()
        assertNull("Username should be null when not provided", server.username)
        assertNull("Password should be null when not provided", server.password)
    }

    @Test
    fun `TURN server URI formats are preserved`() {
        val uris = listOf(
            "turn:turn.example.com:3478?transport=udp",
            "turn:turn.example.com:3478?transport=tcp",
            "turns:turn.example.com:5349?transport=tcp" // TLS
        )

        val servers = IceServerConfig.createCustomServers(uris, "user", "pass")

        val server = servers.first()
        uris.forEach { uri ->
            assertTrue("Should preserve URI format: $uri", server.urls.contains(uri))
        }
    }

    @Test
    fun `Default servers use standard STUN port`() {
        val servers = IceServerConfig.getDefaultServers()

        val uris = servers.flatMap { it.urls }
        uris.forEach { uri ->
            assertTrue("STUN URI should use port 19302: $uri",
                uri.contains(":19302"))
        }
    }
}
