#!/usr/bin/env python3
"""
Unit tests for WebRTC Signaling Server
"""

import asyncio
import json
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from signaling_server import SignalingServer, Connection


@pytest.fixture
def server():
    """Create a SignalingServer instance for testing"""
    return SignalingServer(host="localhost", port=8765)


@pytest.fixture
def mock_websocket():
    """Create a mock WebSocket connection"""
    ws = AsyncMock()
    ws.send = AsyncMock()
    return ws


@pytest.mark.asyncio
async def test_handle_register(server, mock_websocket):
    """Test client registration"""
    register_data = {
        "type": "register",
        "client_id": "test_client_1",
        "room_id": "test_room",
        "client_type": "pc",
    }

    client_id = await server._handle_register(mock_websocket, register_data)

    assert client_id == "test_client_1"
    assert client_id in server.connections
    assert server.connections[client_id].room_id == "test_room"
    assert server.connections[client_id].client_type == "pc"
    assert "test_room" in server.rooms
    assert client_id in server.rooms["test_room"]
    assert mock_websocket.send.called


@pytest.mark.asyncio
async def test_handle_offer(server, mock_websocket):
    """Test SDP offer relay"""
    # Register sender and target
    sender_ws = AsyncMock()
    target_ws = AsyncMock()

    await server._handle_register(
        sender_ws,
        {
            "type": "register",
            "client_id": "sender",
            "room_id": "test_room",
            "client_type": "pc",
        },
    )

    await server._handle_register(
        target_ws,
        {
            "type": "register",
            "client_id": "target",
            "room_id": "test_room",
            "client_type": "android",
        },
    )

    # Send offer
    offer_data = {
        "type": "offer",
        "target_id": "target",
        "sdp": {"type": "offer", "sdp": "test_sdp_offer"},
    }

    await server._handle_offer("sender", offer_data)

    # Verify target received the offer
    assert target_ws.send.called
    sent_message = json.loads(target_ws.send.call_args[0][0])
    assert sent_message["type"] == "offer"
    assert sent_message["sender_id"] == "sender"
    assert sent_message["sdp"]["sdp"] == "test_sdp_offer"


@pytest.mark.asyncio
async def test_handle_answer(server, mock_websocket):
    """Test SDP answer relay"""
    # Register sender and target
    sender_ws = AsyncMock()
    target_ws = AsyncMock()

    await server._handle_register(
        sender_ws,
        {
            "type": "register",
            "client_id": "sender",
            "room_id": "test_room",
            "client_type": "android",
        },
    )

    await server._handle_register(
        target_ws,
        {
            "type": "register",
            "client_id": "target",
            "room_id": "test_room",
            "client_type": "pc",
        },
    )

    # Send answer
    answer_data = {
        "type": "answer",
        "target_id": "target",
        "sdp": {"type": "answer", "sdp": "test_sdp_answer"},
    }

    await server._handle_answer("sender", answer_data)

    # Verify target received the answer
    assert target_ws.send.called
    sent_message = json.loads(target_ws.send.call_args[0][0])
    assert sent_message["type"] == "answer"
    assert sent_message["sender_id"] == "sender"
    assert sent_message["sdp"]["sdp"] == "test_sdp_answer"


@pytest.mark.asyncio
async def test_handle_ice_candidate(server, mock_websocket):
    """Test ICE candidate exchange"""
    # Register sender and target
    sender_ws = AsyncMock()
    target_ws = AsyncMock()

    await server._handle_register(
        sender_ws,
        {
            "type": "register",
            "client_id": "sender",
            "room_id": "test_room",
            "client_type": "pc",
        },
    )

    await server._handle_register(
        target_ws,
        {
            "type": "register",
            "client_id": "target",
            "room_id": "test_room",
            "client_type": "android",
        },
    )

    # Send ICE candidate
    ice_data = {
        "type": "ice_candidate",
        "target_id": "target",
        "candidate": {
            "candidate": "test_candidate",
            "sdpMid": "0",
            "sdpMLineIndex": 0,
        },
    }

    await server._handle_ice_candidate("sender", ice_data)

    # Verify target received the ICE candidate
    assert target_ws.send.called
    sent_message = json.loads(target_ws.send.call_args[0][0])
    assert sent_message["type"] == "ice_candidate"
    assert sent_message["sender_id"] == "sender"
    assert sent_message["candidate"]["candidate"] == "test_candidate"


@pytest.mark.asyncio
async def test_broadcast_to_room(server, mock_websocket):
    """Test broadcasting to room"""
    # Register multiple clients
    ws1 = AsyncMock()
    ws2 = AsyncMock()
    ws3 = AsyncMock()

    await server._handle_register(
        ws1,
        {"type": "register", "client_id": "client1", "room_id": "test_room"},
    )
    await server._handle_register(
        ws2,
        {"type": "register", "client_id": "client2", "room_id": "test_room"},
    )
    await server._handle_register(
        ws3,
        {"type": "register", "client_id": "client3", "room_id": "test_room"},
    )

    # Broadcast message, excluding client1
    message = {"type": "test", "data": "test_data"}
    await server._broadcast_to_room("test_room", message, exclude="client1")

    # Verify only client2 and client3 received the message
    assert not ws1.send.called or ws1.send.call_count == 1  # Only registration message
    assert ws2.send.call_count >= 2  # Registration + broadcast
    assert ws3.send.call_count >= 2  # Registration + broadcast


@pytest.mark.asyncio
async def test_cleanup_connection(server, mock_websocket):
    """Test connection cleanup"""
    # Register client
    ws1 = AsyncMock()
    ws2 = AsyncMock()

    await server._handle_register(
        ws1,
        {"type": "register", "client_id": "client1", "room_id": "test_room"},
    )
    await server._handle_register(
        ws2,
        {"type": "register", "client_id": "client2", "room_id": "test_room"},
    )

    # Cleanup client1
    await server._cleanup_connection("client1")

    # Verify client1 is removed
    assert "client1" not in server.connections
    assert "client1" not in server.rooms["test_room"]
    assert "client2" in server.rooms["test_room"]

    # Verify client2 was notified
    assert ws2.send.call_count >= 2  # Registration + peer_left notification


@pytest.mark.asyncio
async def test_multiple_rooms(server, mock_websocket):
    """Test multiple rooms isolation"""
    # Register clients in different rooms
    ws1 = AsyncMock()
    ws2 = AsyncMock()

    await server._handle_register(
        ws1,
        {"type": "register", "client_id": "client1", "room_id": "room1"},
    )
    await server._handle_register(
        ws2,
        {"type": "register", "client_id": "client2", "room_id": "room2"},
    )

    assert "room1" in server.rooms
    assert "room2" in server.rooms
    assert "client1" in server.rooms["room1"]
    assert "client2" in server.rooms["room2"]
    assert "client1" not in server.rooms["room2"]
    assert "client2" not in server.rooms["room1"]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
