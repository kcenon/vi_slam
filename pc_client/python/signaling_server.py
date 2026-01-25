#!/usr/bin/env python3
"""
WebRTC Signaling Server for VI-SLAM

Lightweight WebSocket server for establishing WebRTC peer connections
between PC client and Android device.
"""

import asyncio
import json
import logging
import sys
from typing import Dict, Set
from dataclasses import dataclass, field
from datetime import datetime

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    print("Error: websockets library not found. Install it with: pip install websockets")
    sys.exit(1)


@dataclass
class Connection:
    """Represents a WebSocket connection"""
    websocket: WebSocketServerProtocol
    client_id: str
    room_id: str = ""
    client_type: str = ""  # "pc" or "android"
    connected_at: datetime = field(default_factory=datetime.now)


class SignalingServer:
    """WebRTC signaling server for peer connection establishment"""

    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        """
        Initialize signaling server

        Args:
            host: Server host address
            port: Server port number
        """
        self.host = host
        self.port = port
        self.connections: Dict[str, Connection] = {}
        self.rooms: Dict[str, Set[str]] = {}
        self.logger = logging.getLogger(__name__)

    async def handle_client(
        self, websocket: WebSocketServerProtocol, path: str
    ) -> None:
        """
        Handle WebSocket client connection

        Args:
            websocket: WebSocket connection
            path: Request path
        """
        client_id = None

        try:
            async for message in websocket:
                data = json.loads(message)
                msg_type = data.get("type")

                if msg_type == "register":
                    client_id = await self._handle_register(websocket, data)
                elif msg_type == "offer":
                    await self._handle_offer(client_id, data)
                elif msg_type == "answer":
                    await self._handle_answer(client_id, data)
                elif msg_type == "ice_candidate":
                    await self._handle_ice_candidate(client_id, data)
                else:
                    self.logger.warning(f"Unknown message type: {msg_type}")

        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client {client_id} disconnected")
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON from client {client_id}: {e}")
        except Exception as e:
            self.logger.error(f"Error handling client {client_id}: {e}")
        finally:
            if client_id:
                await self._cleanup_connection(client_id)

    async def _handle_register(
        self, websocket: WebSocketServerProtocol, data: dict
    ) -> str:
        """
        Handle client registration

        Args:
            websocket: WebSocket connection
            data: Registration data

        Returns:
            Client ID
        """
        client_id = data.get("client_id")
        room_id = data.get("room_id", "default")
        client_type = data.get("client_type", "unknown")

        if not client_id:
            self.logger.error("Registration missing client_id")
            await websocket.send(
                json.dumps({"type": "error", "message": "Missing client_id"})
            )
            return None

        # Store connection
        conn = Connection(
            websocket=websocket,
            client_id=client_id,
            room_id=room_id,
            client_type=client_type,
        )
        self.connections[client_id] = conn

        # Add to room
        if room_id not in self.rooms:
            self.rooms[room_id] = set()
        self.rooms[room_id].add(client_id)

        self.logger.info(
            f"Client registered: {client_id} ({client_type}) in room {room_id}"
        )

        # Send registration confirmation
        await websocket.send(
            json.dumps(
                {
                    "type": "registered",
                    "client_id": client_id,
                    "room_id": room_id,
                    "peers": list(self.rooms[room_id] - {client_id}),
                }
            )
        )

        # Notify other peers in the room
        await self._broadcast_to_room(
            room_id,
            {
                "type": "peer_joined",
                "client_id": client_id,
                "client_type": client_type,
            },
            exclude=client_id,
        )

        return client_id

    async def _handle_offer(self, sender_id: str, data: dict) -> None:
        """
        Handle SDP offer

        Args:
            sender_id: Sender client ID
            data: Offer data containing SDP
        """
        target_id = data.get("target_id")
        sdp = data.get("sdp")

        if not target_id or not sdp:
            self.logger.error(f"Invalid offer from {sender_id}")
            return

        if target_id not in self.connections:
            self.logger.warning(f"Target {target_id} not connected")
            return

        target_conn = self.connections[target_id]
        await target_conn.websocket.send(
            json.dumps(
                {
                    "type": "offer",
                    "sender_id": sender_id,
                    "sdp": sdp,
                }
            )
        )

        self.logger.info(f"Relayed offer from {sender_id} to {target_id}")

    async def _handle_answer(self, sender_id: str, data: dict) -> None:
        """
        Handle SDP answer

        Args:
            sender_id: Sender client ID
            data: Answer data containing SDP
        """
        target_id = data.get("target_id")
        sdp = data.get("sdp")

        if not target_id or not sdp:
            self.logger.error(f"Invalid answer from {sender_id}")
            return

        if target_id not in self.connections:
            self.logger.warning(f"Target {target_id} not connected")
            return

        target_conn = self.connections[target_id]
        await target_conn.websocket.send(
            json.dumps(
                {
                    "type": "answer",
                    "sender_id": sender_id,
                    "sdp": sdp,
                }
            )
        )

        self.logger.info(f"Relayed answer from {sender_id} to {target_id}")

    async def _handle_ice_candidate(self, sender_id: str, data: dict) -> None:
        """
        Handle ICE candidate exchange

        Args:
            sender_id: Sender client ID
            data: ICE candidate data
        """
        target_id = data.get("target_id")
        candidate = data.get("candidate")

        if not target_id or not candidate:
            self.logger.error(f"Invalid ICE candidate from {sender_id}")
            return

        if target_id not in self.connections:
            self.logger.warning(f"Target {target_id} not connected")
            return

        target_conn = self.connections[target_id]
        await target_conn.websocket.send(
            json.dumps(
                {
                    "type": "ice_candidate",
                    "sender_id": sender_id,
                    "candidate": candidate,
                }
            )
        )

        self.logger.debug(f"Relayed ICE candidate from {sender_id} to {target_id}")

    async def _broadcast_to_room(
        self, room_id: str, message: dict, exclude: str = None
    ) -> None:
        """
        Broadcast message to all clients in a room

        Args:
            room_id: Room ID
            message: Message to broadcast
            exclude: Client ID to exclude from broadcast
        """
        if room_id not in self.rooms:
            return

        message_str = json.dumps(message)
        for client_id in self.rooms[room_id]:
            if client_id == exclude:
                continue

            if client_id in self.connections:
                try:
                    await self.connections[client_id].websocket.send(message_str)
                except Exception as e:
                    self.logger.error(f"Failed to send to {client_id}: {e}")

    async def _cleanup_connection(self, client_id: str) -> None:
        """
        Clean up client connection

        Args:
            client_id: Client ID to clean up
        """
        if client_id not in self.connections:
            return

        conn = self.connections[client_id]
        room_id = conn.room_id

        # Remove from room
        if room_id in self.rooms:
            self.rooms[room_id].discard(client_id)
            if not self.rooms[room_id]:
                del self.rooms[room_id]

            # Notify other peers
            await self._broadcast_to_room(
                room_id, {"type": "peer_left", "client_id": client_id}
            )

        # Remove connection
        del self.connections[client_id]
        self.logger.info(f"Cleaned up connection: {client_id}")

    async def start(self) -> None:
        """Start the signaling server"""
        self.logger.info(f"Starting signaling server on {self.host}:{self.port}")

        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"Signaling server running on ws://{self.host}:{self.port}")
            await asyncio.Future()  # Run forever


def setup_logging(level: str = "INFO") -> None:
    """
    Setup logging configuration

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR)
    """
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def main() -> None:
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="WebRTC Signaling Server for VI-SLAM")
    parser.add_argument(
        "--host", default="0.0.0.0", help="Server host (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8765, help="Server port (default: 8765)"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )

    args = parser.parse_args()

    setup_logging(args.log_level)

    server = SignalingServer(host=args.host, port=args.port)

    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        logging.info("Shutting down signaling server")


if __name__ == "__main__":
    main()
