# WebRTC Signaling Server

Lightweight WebSocket-based signaling server for establishing WebRTC peer connections between VI-SLAM PC client and Android device.

## Features

- WebSocket server for WebRTC signaling
- Handle SDP offer/answer exchange
- Relay ICE candidates
- Support multiple simultaneous connections
- Room-based connection matching
- Comprehensive logging

## Installation

### Prerequisites

- Python 3.8 or higher
- pip package manager

### Setup

1. Create and activate virtual environment:
```bash
cd pc_client/python
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Starting the Server

Default configuration (host: 0.0.0.0, port: 8765):
```bash
python signaling_server.py
```

Custom host and port:
```bash
python signaling_server.py --host localhost --port 9000
```

With debug logging:
```bash
python signaling_server.py --log-level DEBUG
```

### Command-Line Options

- `--host HOST`: Server host address (default: 0.0.0.0)
- `--port PORT`: Server port number (default: 8765)
- `--log-level LEVEL`: Logging level: DEBUG, INFO, WARNING, ERROR (default: INFO)

### Example

```bash
# Start server on localhost:8765 with info logging
python signaling_server.py --host localhost --port 8765 --log-level INFO
```

## Protocol

### Message Types

#### 1. Register
Client registers with the server:
```json
{
  "type": "register",
  "client_id": "unique_client_id",
  "room_id": "room_name",
  "client_type": "pc" or "android"
}
```

Response:
```json
{
  "type": "registered",
  "client_id": "unique_client_id",
  "room_id": "room_name",
  "peers": ["peer1_id", "peer2_id"]
}
```

#### 2. Offer
Send SDP offer to peer:
```json
{
  "type": "offer",
  "target_id": "peer_client_id",
  "sdp": {
    "type": "offer",
    "sdp": "..."
  }
}
```

#### 3. Answer
Send SDP answer to peer:
```json
{
  "type": "answer",
  "target_id": "peer_client_id",
  "sdp": {
    "type": "answer",
    "sdp": "..."
  }
}
```

#### 4. ICE Candidate
Send ICE candidate to peer:
```json
{
  "type": "ice_candidate",
  "target_id": "peer_client_id",
  "candidate": {
    "candidate": "...",
    "sdpMid": "0",
    "sdpMLineIndex": 0
  }
}
```

### Notifications

#### Peer Joined
Notifies when a peer joins the room:
```json
{
  "type": "peer_joined",
  "client_id": "new_peer_id",
  "client_type": "pc" or "android"
}
```

#### Peer Left
Notifies when a peer leaves the room:
```json
{
  "type": "peer_left",
  "client_id": "departed_peer_id"
}
```

## Testing

Run unit tests:
```bash
pytest test_signaling_server.py -v
```

Run tests with coverage:
```bash
pytest test_signaling_server.py --cov=signaling_server --cov-report=html
```

## Architecture

```
┌─────────────┐                  ┌──────────────────┐                  ┌─────────────┐
│  PC Client  │◄────WebSocket────►│ Signaling Server │◄────WebSocket────►│   Android   │
└─────────────┘                  └──────────────────┘                  └─────────────┘
      │                                    │                                    │
      │  1. Register                       │                                    │
      ├───────────────────────────────────►│                                    │
      │  ◄─ Registered (peers list)        │                                    │
      │                                    │  2. Register                       │
      │                                    │◄───────────────────────────────────┤
      │                                    │  ─► Registered (peers list)        │
      │  ◄─ Peer Joined (Android)          │                                    │
      │                                    │  ─► Peer Joined (PC)               │
      │                                    │                                    │
      │  3. Create Offer                   │                                    │
      ├───────────────────────────────────►│                                    │
      │                                    │  ─► Relay Offer                    │
      │                                    ├───────────────────────────────────►│
      │                                    │  4. Create Answer                  │
      │  ◄─ Relay Answer                   │◄───────────────────────────────────┤
      │◄───────────────────────────────────┤                                    │
      │                                    │                                    │
      │  5. Exchange ICE candidates        │                                    │
      │◄──────────────────────────────────►│◄──────────────────────────────────►│
      │                                    │                                    │
      │                                                                         │
      └─────────────────────────WebRTC Connection────────────────────────────┘
```

## Logging

The server logs the following events:
- Client connections/disconnections
- Registration events
- SDP offer/answer relay
- ICE candidate exchange
- Room management (join/leave)
- Errors and warnings

Log format:
```
YYYY-MM-DD HH:MM:SS [LEVEL] module_name: message
```

Example:
```
2025-01-25 10:30:15 [INFO] __main__: Starting signaling server on 0.0.0.0:8765
2025-01-25 10:30:15 [INFO] __main__: Signaling server running on ws://0.0.0.0:8765
2025-01-25 10:30:20 [INFO] signaling_server: Client registered: pc_client_1 (pc) in room default
2025-01-25 10:30:25 [INFO] signaling_server: Client registered: android_client_1 (android) in room default
2025-01-25 10:30:26 [INFO] signaling_server: Relayed offer from pc_client_1 to android_client_1
2025-01-25 10:30:27 [INFO] signaling_server: Relayed answer from android_client_1 to pc_client_1
```

## Security Considerations

This is a lightweight signaling server for development and testing. For production use:

- Add authentication/authorization
- Use WSS (WebSocket Secure) with TLS
- Implement rate limiting
- Add input validation and sanitization
- Consider using a production-grade WebSocket server

## Troubleshooting

### Connection Refused
- Verify server is running
- Check firewall settings
- Ensure correct host/port configuration

### Peer Not Found
- Verify both peers are registered in the same room
- Check client IDs are unique

### Messages Not Delivered
- Check WebSocket connection status
- Verify JSON message format
- Enable DEBUG logging for detailed information

## License

See main project LICENSE file.
