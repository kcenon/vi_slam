package com.vi.slam.android.streaming

import org.webrtc.PeerConnection

/**
 * ICE server configuration for NAT traversal.
 *
 * Provides pre-configured ICE servers (STUN/TURN) for establishing
 * peer-to-peer connections through NAT and firewalls.
 */
object IceServerConfig {
    /**
     * Default ICE servers using public Google STUN servers.
     *
     * STUN servers are used for discovering public IP addresses and ports.
     * These are suitable for development and testing but may not work in
     * all network environments (e.g., symmetric NAT).
     *
     * For production, consider using TURN servers with authentication.
     */
    fun getDefaultServers(): List<PeerConnection.IceServer> {
        return listOf(
            PeerConnection.IceServer.builder("stun:stun.l.google.com:19302")
                .createIceServer(),
            PeerConnection.IceServer.builder("stun:stun1.l.google.com:19302")
                .createIceServer()
        )
    }

    /**
     * Create custom ICE server list from URIs.
     *
     * Example URIs:
     * - STUN: "stun:stun.example.com:3478"
     * - TURN (UDP): "turn:turn.example.com:3478?transport=udp"
     * - TURN (TCP): "turn:turn.example.com:3478?transport=tcp"
     *
     * @param uris List of ICE server URIs
     * @param username Optional username for TURN authentication
     * @param password Optional password for TURN authentication
     * @return List of configured ICE servers
     */
    fun createCustomServers(
        uris: List<String>,
        username: String? = null,
        password: String? = null
    ): List<PeerConnection.IceServer> {
        val builder = PeerConnection.IceServer.builder(uris)

        if (username != null && password != null) {
            builder.setUsername(username)
            builder.setPassword(password)
        }

        return listOf(builder.createIceServer())
    }

    /**
     * Production-ready ICE server configuration.
     *
     * Combines public STUN servers with custom TURN server for maximum
     * connectivity across different network configurations.
     *
     * @param turnUri TURN server URI
     * @param username TURN authentication username
     * @param password TURN authentication password
     * @return List of configured ICE servers
     */
    fun getProductionServers(
        turnUri: String,
        username: String,
        password: String
    ): List<PeerConnection.IceServer> {
        return getDefaultServers() + createCustomServers(
            listOf(turnUri),
            username,
            password
        )
    }
}
