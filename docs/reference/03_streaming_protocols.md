# Real-time Streaming Protocols

## 1. Overview

This document covers protocols and methods for real-time transmission of camera/IMU data from smartphone to PC.

### Streaming Requirements

| Requirement | Target Value | Importance |
|-------------|--------------|------------|
| Video latency | < 100ms | High |
| IMU latency | < 10ms | Very High |
| Data loss | < 1% | High |
| Synchronization accuracy | < 5ms | Very High |

## 2. Major Streaming Protocol Comparison

### 2.1 WebRTC

#### Features
- **Peer-to-Peer**: Direct connection without server
- **Low latency**: Average ~300-500ms (under 100ms when optimized)
- **Built-in encryption**: DTLS + SRTP
- **Native browser support**

#### Advantages
- Lowest latency
- Easy firewall/NAT traversal (ICE/STUN/TURN)
- Adaptive bitrate

#### Disadvantages
- Complex signaling required
- Overhead exists (ICE negotiation)
- Difficult multi-client scaling

#### Latency
```
Average latency: 384ms (research result)
When optimized: 100-200ms
```

### 2.2 RTSP (Real-Time Streaming Protocol)

#### Features
- **Client-server model**
- **Media transport**: Uses RTP (Real-time Transport Protocol)
- **IP camera standard**: Supported by most IP cameras

#### Advantages
- Simple structure
- Fast connection setup
- Supports more connections per server
- Existing infrastructure compatibility

#### Disadvantages
- Higher latency than WebRTC
- No built-in encryption
- Difficult NAT traversal

#### Latency
```
Typical: 1-3 seconds
When optimized: 500ms-1 second
```

### 2.3 RTMP (Real-Time Messaging Protocol)

#### Features
- Protocol developed by Adobe
- TCP-based
- Requires streaming server

#### Advantages
- Reliable transmission
- Widely used

#### Disadvantages
- High latency (1-3 seconds)
- Flash-based legacy
- Decreasing browser support

### 2.4 SRT (Secure Reliable Transport)

#### Features
- Open-source protocol
- Optimized for unstable networks
- Built-in AES encryption

#### Advantages
- Packet loss recovery
- Low latency (120ms target)
- High reliability

#### Disadvantages
- No native browser support
- Relatively new protocol

## 3. Protocol Comparison Summary

| Protocol | Latency | Implementation Complexity | Scalability | Security | Recommended Use |
|----------|---------|---------------------------|-------------|----------|-----------------|
| WebRTC | Lowest (~300ms) | High | Low | High | 1:1 real-time |
| RTSP | Medium (~1s) | Low | High | Low | IP cameras |
| RTMP | High (~2s) | Medium | High | Medium | Live broadcast |
| SRT | Low (~500ms) | Medium | Medium | High | Unstable networks |

## 4. Implementation Methods

### 4.1 WebRTC-based Implementation

#### Android Client
```kotlin
// WebRTC PeerConnection setup
val peerConnectionFactory = PeerConnectionFactory.builder()
    .setVideoEncoderFactory(DefaultVideoEncoderFactory(...))
    .setVideoDecoderFactory(DefaultVideoDecoderFactory(...))
    .createPeerConnectionFactory()

// Camera capture
val videoCapturer = Camera2Enumerator(context)
    .createCapturer(cameraName, null)
```

#### PC Receiver (Python/aiortc)
```python
from aiortc import RTCPeerConnection, VideoStreamTrack

pc = RTCPeerConnection()

@pc.on("track")
async def on_track(track):
    if track.kind == "video":
        while True:
            frame = await track.recv()
            # Process frame for SLAM
```

### 4.2 RTSP-based Implementation

#### Android Server (libstreaming)
```java
// Start RTSP server
Session session = SessionBuilder.getInstance()
    .setContext(context)
    .setAudioEncoder(SessionBuilder.AUDIO_NONE)
    .setVideoEncoder(SessionBuilder.VIDEO_H264)
    .setCamera(Camera.CameraInfo.CAMERA_FACING_BACK)
    .build();
```

#### PC Receiver (OpenCV)
```python
import cv2

cap = cv2.VideoCapture("rtsp://phone_ip:8554/video")
while True:
    ret, frame = cap.read()
    if ret:
        # Process frame for SLAM
```

### 4.3 Custom TCP/UDP Implementation

#### IMU Data Transmission (UDP - Low Latency)
```python
# Android (Kotlin)
val socket = DatagramSocket()
val buffer = "${timestamp},${ax},${ay},${az},${gx},${gy},${gz}".toByteArray()
val packet = DatagramPacket(buffer, buffer.size, serverAddress, port)
socket.send(packet)

# PC Receiver (Python)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))
while True:
    data, addr = sock.recvfrom(1024)
    timestamp, ax, ay, az, gx, gy, gz = parse_imu(data)
```

## 5. Hybrid Architecture (Recommended)

For VI-SLAM purposes, separating video and IMU transmission is effective:

```
[Smartphone]
    │
    ├─── Camera frames ──→ WebRTC/RTSP (30Hz)
    │                         │
    └─── IMU data ──────→ UDP (200Hz, low latency)
                              │
                              ▼
                         [PC Receiver]
                              │
                    ┌────────┴────────┐
                    ▼                 ▼
              Video Buffer       IMU Buffer
                    │                 │
                    └────────┬────────┘
                             ▼
                    Timestamp Synchronization
                             │
                             ▼
                      VI-SLAM Processing
```

### Synchronization Strategy

1. **Common time reference**: Time synchronization via NTP or PTP
2. **Include timestamps**: Include sensor timestamps in all data
3. **Buffering**: Timestamp-based data alignment on PC
4. **Interpolation**: Interpolate IMU data to camera frame times

## 6. Network Requirements

### Bandwidth Calculation

```
Video (H.264, 720p, 30fps):
- Low quality: 1-2 Mbps
- Medium quality: 2-4 Mbps
- High quality: 4-8 Mbps

IMU (200Hz, 7 float values):
- ~11 KB/s (negligible)

Total required bandwidth: 2-10 Mbps
```

### Recommended Network Environment

| Environment | Characteristics | Suitability |
|-------------|-----------------|-------------|
| 5GHz WiFi | Low latency, high bandwidth | Optimal |
| 2.4GHz WiFi | Possible interference, medium bandwidth | Suitable |
| USB tethering | Very low latency | Excellent |
| 5G mobile | Variable latency | Conditionally suitable |

## 7. Open-Source Solutions

### Ant Media Server
- WebRTC-based streaming server
- ~0.5 second latency
- Android/iOS SDK provided
- [antmedia.io](https://antmedia.io/)

### GStreamer
- Multimedia framework
- RTSP/RTP/WebRTC support
- Cross-platform

### FFmpeg
- Encoding/decoding/streaming
- RTSP/RTMP/UDP support
- Command-line based

## References

- [WebRTC vs RTSP Comparison (Nabto)](https://www.nabto.com/webrtc-vs-rtsp/)
- [RTSP Explained (Ant Media)](https://antmedia.io/rtsp-explained-what-is-rtsp-how-it-works/)
- [Video Streaming Protocols (GetStream)](https://getstream.io/blog/streaming-protocols/)
- [Low Latency Streaming (kaanlabs)](https://kaanlabs.com/low-latency-webcam-to-browser-streaming-with-rtsp-webrtc/)
- [7 Ways WebRTC Solves Ultra-Low Latency (Red5)](https://www.red5.net/blog/7-ways-webrtc-solves-ultra-low-latency-streaming/)
