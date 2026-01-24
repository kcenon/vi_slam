# Real-time Streaming Protocols

## 1. 개요

스마트폰에서 PC로 카메라/IMU 데이터를 실시간 전송하기 위한 프로토콜 및 방법을 다룬다.

### 스트리밍 요구사항

| 요구사항 | 목표값 | 중요도 |
|---------|-------|--------|
| 영상 지연 | < 100ms | 높음 |
| IMU 지연 | < 10ms | 매우 높음 |
| 데이터 손실 | < 1% | 높음 |
| 동기화 정확도 | < 5ms | 매우 높음 |

## 2. 주요 스트리밍 프로토콜 비교

### 2.1 WebRTC

#### 특징
- **Peer-to-Peer**: 서버 없이 직접 연결 가능
- **저지연**: 평균 ~300-500ms (최적화 시 100ms 이하)
- **내장 암호화**: DTLS + SRTP
- **브라우저 네이티브 지원**

#### 장점
- 가장 낮은 지연 시간
- 방화벽/NAT 통과 용이 (ICE/STUN/TURN)
- 적응형 비트레이트

#### 단점
- 복잡한 시그널링 필요
- 오버헤드 존재 (ICE negotiation)
- 다중 클라이언트 확장 어려움

#### 지연 시간
```
평균 지연: 384ms (연구 결과)
최적화 시: 100-200ms
```

### 2.2 RTSP (Real-Time Streaming Protocol)

#### 특징
- **클라이언트-서버 모델**
- **미디어 전송**: RTP (Real-time Transport Protocol) 사용
- **IP 카메라 표준**: 대부분의 IP 카메라 지원

#### 장점
- 간단한 구조
- 빠른 연결 설정
- 서버당 더 많은 연결 지원
- 기존 인프라 호환성

#### 단점
- WebRTC 대비 높은 지연
- 내장 암호화 없음
- NAT 통과 어려움

#### 지연 시간
```
일반적: 1-3초
최적화 시: 500ms-1초
```

### 2.3 RTMP (Real-Time Messaging Protocol)

#### 특징
- Adobe 개발 프로토콜
- TCP 기반
- 스트리밍 서버 필요

#### 장점
- 안정적인 전송
- 널리 사용됨

#### 단점
- 높은 지연 (1-3초)
- Flash 기반 레거시
- 브라우저 지원 감소

### 2.4 SRT (Secure Reliable Transport)

#### 특징
- 오픈소스 프로토콜
- 불안정한 네트워크에 최적화
- AES 암호화 내장

#### 장점
- 패킷 손실 복구
- 저지연 (120ms 목표)
- 신뢰성 높음

#### 단점
- 네이티브 브라우저 미지원
- 상대적으로 새로운 프로토콜

## 3. 프로토콜 비교 요약

| 프로토콜 | 지연 시간 | 구현 복잡도 | 확장성 | 보안 | 권장 용도 |
|---------|----------|------------|--------|------|----------|
| WebRTC | 최저 (~300ms) | 높음 | 낮음 | 높음 | 1:1 실시간 |
| RTSP | 중간 (~1초) | 낮음 | 높음 | 낮음 | IP 카메라 |
| RTMP | 높음 (~2초) | 중간 | 높음 | 중간 | 라이브 방송 |
| SRT | 낮음 (~500ms) | 중간 | 중간 | 높음 | 불안정 네트워크 |

## 4. 구현 방법

### 4.1 WebRTC 기반 구현

#### Android 클라이언트
```kotlin
// WebRTC PeerConnection 설정
val peerConnectionFactory = PeerConnectionFactory.builder()
    .setVideoEncoderFactory(DefaultVideoEncoderFactory(...))
    .setVideoDecoderFactory(DefaultVideoDecoderFactory(...))
    .createPeerConnectionFactory()

// 카메라 캡처
val videoCapturer = Camera2Enumerator(context)
    .createCapturer(cameraName, null)
```

#### PC 수신 (Python/aiortc)
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

### 4.2 RTSP 기반 구현

#### Android 서버 (libstreaming)
```java
// RTSP 서버 시작
Session session = SessionBuilder.getInstance()
    .setContext(context)
    .setAudioEncoder(SessionBuilder.AUDIO_NONE)
    .setVideoEncoder(SessionBuilder.VIDEO_H264)
    .setCamera(Camera.CameraInfo.CAMERA_FACING_BACK)
    .build();
```

#### PC 수신 (OpenCV)
```python
import cv2

cap = cv2.VideoCapture("rtsp://phone_ip:8554/video")
while True:
    ret, frame = cap.read()
    if ret:
        # Process frame for SLAM
```

### 4.3 커스텀 TCP/UDP 구현

#### IMU 데이터 전송 (UDP - 저지연)
```python
# Android (Kotlin)
val socket = DatagramSocket()
val buffer = "${timestamp},${ax},${ay},${az},${gx},${gy},${gz}".toByteArray()
val packet = DatagramPacket(buffer, buffer.size, serverAddress, port)
socket.send(packet)

# PC 수신 (Python)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))
while True:
    data, addr = sock.recvfrom(1024)
    timestamp, ax, ay, az, gx, gy, gz = parse_imu(data)
```

## 5. 하이브리드 아키텍처 (권장)

VI-SLAM 용도로는 비디오와 IMU를 분리하여 전송하는 것이 효과적:

```
[스마트폰]
    │
    ├─── 카메라 프레임 ──→ WebRTC/RTSP (30Hz)
    │                         │
    └─── IMU 데이터 ──────→ UDP (200Hz, 저지연)
                              │
                              ▼
                         [PC 수신]
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
                      VI-SLAM 처리
```

### 동기화 전략

1. **공통 시간 기준**: NTP 또는 PTP로 시간 동기화
2. **타임스탬프 포함**: 모든 데이터에 센서 타임스탬프 포함
3. **버퍼링**: PC에서 타임스탬프 기반 데이터 정렬
4. **보간**: IMU 데이터를 카메라 프레임 시간에 맞게 보간

## 6. 네트워크 요구사항

### 대역폭 계산

```
비디오 (H.264, 720p, 30fps):
- 저품질: 1-2 Mbps
- 중품질: 2-4 Mbps
- 고품질: 4-8 Mbps

IMU (200Hz, 7 float values):
- ~11 KB/s (무시할 수준)

총 요구 대역폭: 2-10 Mbps
```

### 권장 네트워크 환경

| 환경 | 특성 | 적합성 |
|------|------|--------|
| 5GHz WiFi | 낮은 지연, 높은 대역폭 | 최적 |
| 2.4GHz WiFi | 간섭 가능, 중간 대역폭 | 적합 |
| USB 테더링 | 매우 낮은 지연 | 우수 |
| 5G 모바일 | 가변 지연 | 조건부 적합 |

## 7. 오픈소스 솔루션

### Ant Media Server
- WebRTC 기반 스트리밍 서버
- ~0.5초 지연
- Android/iOS SDK 제공
- [antmedia.io](https://antmedia.io/)

### GStreamer
- 멀티미디어 프레임워크
- RTSP/RTP/WebRTC 지원
- 크로스 플랫폼

### FFmpeg
- 인코딩/디코딩/스트리밍
- RTSP/RTMP/UDP 지원
- 명령줄 기반

## References

- [WebRTC vs RTSP Comparison (Nabto)](https://www.nabto.com/webrtc-vs-rtsp/)
- [RTSP Explained (Ant Media)](https://antmedia.io/rtsp-explained-what-is-rtsp-how-it-works/)
- [Video Streaming Protocols (GetStream)](https://getstream.io/blog/streaming-protocols/)
- [Low Latency Streaming (kaanlabs)](https://kaanlabs.com/low-latency-webcam-to-browser-streaming-with-rtsp-webrtc/)
- [7 Ways WebRTC Solves Ultra-Low Latency (Red5)](https://www.red5.net/blog/7-ways-webrtc-solves-ultra-low-latency-streaming/)
