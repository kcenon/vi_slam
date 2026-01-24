# Data Format and Streaming Implementation

## 1. Data Formats

### 1.1 IMU Data Format

#### CSV Format (Compatibility-Focused)

```csv
# imu_data.csv
# timestamp_ns: nanoseconds, based on sensor_timestamp
# accel_x/y/z: m/s², gyro_x/y/z: rad/s
timestamp_ns,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
1642345678000000000,0.123,-9.81,0.045,0.001,-0.002,0.003
1642345678005000000,0.125,-9.79,0.048,0.002,-0.001,0.002
...
```

#### Binary Format (Performance-Focused)

```
Binary IMU Format v1.0
======================

Header (32 bytes):
  - Magic: 4 bytes (0x494D5531 = "IMU1")
  - Version: 4 bytes (uint32, little-endian)
  - Start timestamp: 8 bytes (int64, nanoseconds)
  - IMU rate: 4 bytes (float32, Hz)
  - Reserved: 12 bytes

Each Record (40 bytes):
  - Timestamp: 8 bytes (int64, nanoseconds)
  - Accelerometer: 12 bytes (3 x float32)
  - Gyroscope: 12 bytes (3 x float32)
  - Flags: 4 bytes (uint32, bit flags)
  - Reserved: 4 bytes

Flags:
  - Bit 0: Accelerometer valid
  - Bit 1: Gyroscope valid
  - Bit 2: Uncalibrated data
  - Bits 3-31: Reserved
```

#### Protocol Buffers Format (Extensibility-Focused)

```protobuf
// imu_data.proto
syntax = "proto3";

package vislam;

message ImuSample {
    int64 timestamp_ns = 1;

    message Vector3 {
        double x = 1;
        double y = 2;
        double z = 3;
    }

    Vector3 accelerometer = 2;  // m/s²
    Vector3 gyroscope = 3;      // rad/s
    Vector3 magnetometer = 4;   // μT (optional)

    // Uncalibrated data (optional)
    Vector3 accel_bias = 5;
    Vector3 gyro_drift = 6;

    int32 accuracy = 7;  // 0-3
}

message ImuBatch {
    repeated ImuSample samples = 1;
    int64 batch_timestamp_ns = 2;
    string device_id = 3;
}

message ImuStreamHeader {
    int32 version = 1;
    int64 start_timestamp_ns = 2;
    double nominal_rate_hz = 3;
    string device_model = 4;

    message ImuIntrinsics {
        double accel_noise_density = 1;     // m/s²/√Hz
        double accel_random_walk = 2;       // m/s³/√Hz
        double gyro_noise_density = 3;      // rad/s/√Hz
        double gyro_random_walk = 4;        // rad/s²/√Hz
    }
    ImuIntrinsics intrinsics = 5;
}
```

### 1.2 Camera Metadata Format

#### CSV Format

```csv
# frame_metadata.csv
frame_number,timestamp_ns,exposure_time_ns,iso,focal_length_mm,focal_length_px,rolling_shutter_skew_ns,width,height
0,1642345678000000000,8333333,100,4.25,1000.5,15000000,1280,720
1,1642345678033333333,8333333,100,4.25,1000.5,15000000,1280,720
...
```

#### Protocol Buffers Format

```protobuf
// camera_data.proto
syntax = "proto3";

package vislam;

message FrameMetadata {
    int64 frame_number = 1;
    int64 timestamp_ns = 2;

    // Exposure info
    int64 exposure_time_ns = 3;
    int32 iso = 4;

    // Lens info
    float focal_length_mm = 5;
    float focal_length_px = 6;
    float aperture = 7;

    // Rolling shutter
    int64 rolling_shutter_skew_ns = 8;

    // Image dimensions
    int32 width = 9;
    int32 height = 10;

    // OIS data (if available)
    message OISData {
        repeated float x_shifts = 1;
        repeated float y_shifts = 2;
        repeated int64 timestamps = 3;
    }
    OISData ois_data = 11;
}

message CameraIntrinsics {
    float fx = 1;  // focal length x (pixels)
    float fy = 2;  // focal length y (pixels)
    float cx = 3;  // principal point x
    float cy = 4;  // principal point y

    // Distortion coefficients (radtan model)
    repeated double distortion_coeffs = 5;  // [k1, k2, p1, p2, k3]

    int32 width = 6;
    int32 height = 7;
}
```

### 1.3 Synchronized Data Package

```protobuf
// synchronized_data.proto
syntax = "proto3";

package vislam;

import "imu_data.proto";
import "camera_data.proto";

message SynchronizedFrame {
    FrameMetadata frame_metadata = 1;

    // IMU data before and after frame
    repeated ImuSample imu_before = 2;  // Previous frame ~ Current frame
    repeated ImuSample imu_after = 3;   // Current frame ~ Next frame (optional)

    // Image data (optional: can reference external file)
    bytes image_data = 4;
    string image_path = 5;  // Or external file path
}

message RecordingSession {
    string session_id = 1;
    int64 start_timestamp_ns = 2;
    int64 end_timestamp_ns = 3;

    CameraIntrinsics camera_intrinsics = 4;
    ImuStreamHeader imu_header = 5;

    // Extrinsics (Camera-IMU transformation)
    message Transform {
        repeated double rotation = 1;     // Quaternion [w, x, y, z]
        repeated double translation = 2;  // [x, y, z] meters
    }
    Transform T_imu_camera = 6;

    double time_offset_imu_camera_s = 7;  // IMU - Camera time offset (seconds)
}
```

## 2. Real-Time Streaming Implementation

### 2.1 UDP-Based IMU Streaming

#### Android Sender (Kotlin)

```kotlin
class UdpImuStreamer(
    private val serverAddress: InetAddress,
    private val port: Int = 5005
) {
    private val socket = DatagramSocket()
    private val buffer = ByteBuffer.allocate(48).order(ByteOrder.LITTLE_ENDIAN)

    // Reusable buffer for serialization performance
    fun sendImuSample(
        timestamp: Long,
        accel: FloatArray,
        gyro: FloatArray
    ) {
        buffer.clear()

        // Header (4 bytes)
        buffer.putInt(0x494D5530)  // "IMU0"

        // Timestamp (8 bytes)
        buffer.putLong(timestamp)

        // Accelerometer (12 bytes)
        accel.forEach { buffer.putFloat(it) }

        // Gyroscope (12 bytes)
        gyro.forEach { buffer.putFloat(it) }

        // Sequence number (4 bytes) - For packet loss detection
        buffer.putInt(sequenceNumber++)

        // Checksum (4 bytes)
        buffer.putInt(calculateChecksum(buffer.array(), 0, 44))

        val packet = DatagramPacket(
            buffer.array(),
            buffer.position(),
            serverAddress,
            port
        )

        socket.send(packet)
    }

    private var sequenceNumber = 0

    private fun calculateChecksum(data: ByteArray, offset: Int, length: Int): Int {
        var sum = 0
        for (i in offset until offset + length) {
            sum = sum xor data[i].toInt()
        }
        return sum
    }

    fun close() {
        socket.close()
    }
}
```

#### PC Receiver (Python)

```python
import socket
import struct
import threading
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

@dataclass
class ImuSample:
    timestamp_ns: int
    accel: tuple  # (ax, ay, az)
    gyro: tuple   # (gx, gy, gz)
    sequence: int

class UdpImuReceiver:
    MAGIC = 0x494D5530  # "IMU0"
    PACKET_SIZE = 48

    def __init__(self, port: int = 5005, buffer_size: int = 10000):
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', port))
        self.socket.settimeout(0.1)

        self.buffer = deque(maxlen=buffer_size)
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.callback: Optional[Callable[[ImuSample], None]] = None

        self.last_sequence = -1
        self.dropped_packets = 0

    def start(self, callback: Optional[Callable[[ImuSample], None]] = None):
        self.callback = callback
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.socket.close()

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(self.PACKET_SIZE)
                sample = self._parse_packet(data)
                if sample:
                    self._check_sequence(sample.sequence)
                    self.buffer.append(sample)
                    if self.callback:
                        self.callback(sample)
            except socket.timeout:
                continue

    def _parse_packet(self, data: bytes) -> Optional[ImuSample]:
        if len(data) != self.PACKET_SIZE:
            return None

        magic, timestamp, ax, ay, az, gx, gy, gz, seq, checksum = struct.unpack(
            '<IqffffffiI', data
        )

        if magic != self.MAGIC:
            return None

        # Verify checksum
        expected_checksum = 0
        for b in data[:44]:
            expected_checksum ^= b
        if expected_checksum != checksum:
            return None

        return ImuSample(
            timestamp_ns=timestamp,
            accel=(ax, ay, az),
            gyro=(gx, gy, gz),
            sequence=seq
        )

    def _check_sequence(self, sequence: int):
        if self.last_sequence >= 0:
            expected = (self.last_sequence + 1) & 0xFFFFFFFF
            if sequence != expected:
                self.dropped_packets += (sequence - expected) & 0xFFFFFFFF
        self.last_sequence = sequence

    def get_samples(self, count: int = None) -> list:
        if count is None:
            samples = list(self.buffer)
            self.buffer.clear()
            return samples
        else:
            samples = []
            for _ in range(min(count, len(self.buffer))):
                samples.append(self.buffer.popleft())
            return samples

    def get_stats(self) -> dict:
        return {
            'buffer_size': len(self.buffer),
            'dropped_packets': self.dropped_packets
        }
```

### 2.2 WebRTC-Based Video Streaming

#### Android WebRTC Setup (Kotlin)

```kotlin
// build.gradle
// implementation "io.getstream:stream-webrtc-android:1.0.0"

class WebRtcVideoStreamer(
    private val context: Context,
    private val signalingUrl: String
) {
    private var peerConnectionFactory: PeerConnectionFactory? = null
    private var peerConnection: PeerConnection? = null
    private var localVideoTrack: VideoTrack? = null
    private var videoCapturer: CameraVideoCapturer? = null

    fun initialize() {
        // Initialize WebRTC
        val options = PeerConnectionFactory.InitializationOptions.builder(context)
            .setEnableInternalTracer(true)
            .createInitializationOptions()
        PeerConnectionFactory.initialize(options)

        // Create factory
        val encoderFactory = DefaultVideoEncoderFactory(
            EglBase.create().eglBaseContext,
            true,
            true
        )
        val decoderFactory = DefaultVideoDecoderFactory(
            EglBase.create().eglBaseContext
        )

        peerConnectionFactory = PeerConnectionFactory.builder()
            .setVideoEncoderFactory(encoderFactory)
            .setVideoDecoderFactory(decoderFactory)
            .createPeerConnectionFactory()
    }

    fun startCapture(width: Int = 1280, height: Int = 720, fps: Int = 30) {
        // Create camera capturer
        val enumerator = Camera2Enumerator(context)
        val deviceName = enumerator.deviceNames.find {
            enumerator.isBackFacing(it)
        } ?: return

        videoCapturer = enumerator.createCapturer(deviceName, null)

        // Create video source and track
        val surfaceTextureHelper = SurfaceTextureHelper.create(
            "CaptureThread",
            EglBase.create().eglBaseContext
        )

        val videoSource = peerConnectionFactory?.createVideoSource(false)
        videoCapturer?.initialize(surfaceTextureHelper, context, videoSource?.capturerObserver)
        videoCapturer?.startCapture(width, height, fps)

        localVideoTrack = peerConnectionFactory?.createVideoTrack("video0", videoSource)
    }

    fun createOffer(callback: (SessionDescription) -> Unit) {
        val constraints = MediaConstraints().apply {
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveVideo", "false"))
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveAudio", "false"))
        }

        peerConnection?.createOffer(object : SdpObserver {
            override fun onCreateSuccess(sdp: SessionDescription) {
                peerConnection?.setLocalDescription(object : SdpObserver {
                    override fun onSetSuccess() {
                        callback(sdp)
                    }
                    // ... other callbacks
                }, sdp)
            }
            // ... other callbacks
        }, constraints)
    }

    fun stop() {
        videoCapturer?.stopCapture()
        peerConnection?.close()
    }
}
```

#### PC WebRTC Receiver (Python - aiortc)

```python
import asyncio
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRecorder
from aiohttp import web
import aiohttp_cors

class WebRtcVideoReceiver:
    def __init__(self):
        self.pc = None
        self.frame_callback = None
        self.latest_frame = None
        self.frame_timestamp = None

    async def handle_offer(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

        self.pc = RTCPeerConnection()

        @self.pc.on('track')
        async def on_track(track):
            if track.kind == 'video':
                asyncio.create_task(self._process_video(track))

        await self.pc.setRemoteDescription(offer)
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)

        return web.json_response({
            'sdp': self.pc.localDescription.sdp,
            'type': self.pc.localDescription.type
        })

    async def _process_video(self, track):
        while True:
            try:
                frame = await track.recv()

                # VideoFrame → numpy array
                img = frame.to_ndarray(format='bgr24')

                self.latest_frame = img
                self.frame_timestamp = frame.time

                if self.frame_callback:
                    self.frame_callback(img, frame.time)

            except Exception as e:
                print(f"Error processing frame: {e}")
                break

    def get_latest_frame(self):
        return self.latest_frame, self.frame_timestamp

    async def start_server(self, port: int = 8080):
        app = web.Application()

        # CORS configuration
        cors = aiohttp_cors.setup(app)
        resource = cors.add(app.router.add_resource('/offer'))
        cors.add(resource.add_route('POST', self.handle_offer), {
            '*': aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                allow_headers='*'
            )
        })

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', port)
        await site.start()

        print(f"WebRTC signaling server running on port {port}")

# Usage example
async def main():
    receiver = WebRtcVideoReceiver()

    def on_frame(frame, timestamp):
        # Pass frame to SLAM processing
        cv2.imshow('WebRTC Stream', frame)
        cv2.waitKey(1)

    receiver.frame_callback = on_frame
    await receiver.start_server(8080)

    # Keep server running
    while True:
        await asyncio.sleep(1)

if __name__ == '__main__':
    asyncio.run(main())
```

### 2.3 Integrated Streaming Client (PC)

```python
import asyncio
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Callable
from collections import deque

@dataclass
class SynchronizedData:
    frame_timestamp_ns: int
    frame: np.ndarray
    imu_samples: List[ImuSample]

class IntegratedStreamReceiver:
    """
    Client for integrated reception of video (WebRTC) and IMU (UDP)
    """

    def __init__(self, video_port: int = 8080, imu_port: int = 5005):
        self.video_receiver = WebRtcVideoReceiver()
        self.imu_receiver = UdpImuReceiver(port=imu_port)

        self.imu_buffer = deque(maxlen=50000)  # ~250 seconds @ 200Hz
        self.frame_buffer = deque(maxlen=100)   # ~3 seconds @ 30fps

        self.synchronized_callback: Optional[Callable[[SynchronizedData], None]] = None

        self.last_frame_time = 0

    async def start(self):
        # Start IMU reception (separate thread)
        self.imu_receiver.start(callback=self._on_imu_sample)

        # Start video reception
        self.video_receiver.frame_callback = self._on_video_frame
        await self.video_receiver.start_server()

    def _on_imu_sample(self, sample: ImuSample):
        self.imu_buffer.append(sample)

    def _on_video_frame(self, frame: np.ndarray, timestamp: float):
        timestamp_ns = int(timestamp * 1e9)

        # Collect IMU data since previous frame
        imu_samples = self._get_imu_between(self.last_frame_time, timestamp_ns)

        self.last_frame_time = timestamp_ns

        if self.synchronized_callback:
            data = SynchronizedData(
                frame_timestamp_ns=timestamp_ns,
                frame=frame,
                imu_samples=imu_samples
            )
            self.synchronized_callback(data)

    def _get_imu_between(self, start_ns: int, end_ns: int) -> List[ImuSample]:
        """Return IMU samples within time range"""
        samples = []
        for sample in self.imu_buffer:
            if start_ns < sample.timestamp_ns <= end_ns:
                samples.append(sample)
        return sorted(samples, key=lambda x: x.timestamp_ns)

    def stop(self):
        self.imu_receiver.stop()

# Usage example
async def main():
    receiver = IntegratedStreamReceiver()

    def on_synchronized_data(data: SynchronizedData):
        print(f"Frame at {data.frame_timestamp_ns}, "
              f"with {len(data.imu_samples)} IMU samples")
        # Pass to SLAM algorithm

    receiver.synchronized_callback = on_synchronized_data
    await receiver.start()

    # Main loop
    while True:
        await asyncio.sleep(0.1)
```

## 3. File Storage Implementation

### 3.1 Session Recorder

```python
import os
import json
import csv
from datetime import datetime
from pathlib import Path

class SessionRecorder:
    def __init__(self, base_dir: str = './recordings'):
        self.base_dir = Path(base_dir)
        self.session_dir = None
        self.imu_writer = None
        self.frame_metadata_writer = None
        self.video_writer = None

    def start_session(self, session_name: str = None):
        if session_name is None:
            session_name = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.session_dir = self.base_dir / session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # IMU CSV file
        imu_file = open(self.session_dir / 'imu_data.csv', 'w', newline='')
        self.imu_writer = csv.writer(imu_file)
        self.imu_writer.writerow([
            'timestamp_ns', 'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z'
        ])

        # Frame metadata CSV file
        meta_file = open(self.session_dir / 'frame_metadata.csv', 'w', newline='')
        self.frame_metadata_writer = csv.writer(meta_file)
        self.frame_metadata_writer.writerow([
            'frame_number', 'timestamp_ns', 'image_filename'
        ])

        # Video frames folder
        (self.session_dir / 'frames').mkdir(exist_ok=True)

        print(f"Recording session started: {self.session_dir}")

    def record_imu(self, sample: ImuSample):
        if self.imu_writer:
            self.imu_writer.writerow([
                sample.timestamp_ns,
                sample.accel[0], sample.accel[1], sample.accel[2],
                sample.gyro[0], sample.gyro[1], sample.gyro[2]
            ])

    def record_frame(self, frame_number: int, timestamp_ns: int, frame: np.ndarray):
        if self.session_dir is None:
            return

        # Save image
        filename = f'frame_{frame_number:06d}.png'
        filepath = self.session_dir / 'frames' / filename
        cv2.imwrite(str(filepath), frame)

        # Record metadata
        if self.frame_metadata_writer:
            self.frame_metadata_writer.writerow([
                frame_number, timestamp_ns, filename
            ])

    def save_calibration(self, camera_intrinsics: dict, imu_params: dict,
                         extrinsics: dict):
        calibration = {
            'camera': camera_intrinsics,
            'imu': imu_params,
            'T_imu_camera': extrinsics,
            'timestamp': datetime.now().isoformat()
        }

        with open(self.session_dir / 'calibration.json', 'w') as f:
            json.dump(calibration, f, indent=2)

    def stop_session(self):
        # Close files
        if hasattr(self, '_imu_file'):
            self._imu_file.close()
        if hasattr(self, '_meta_file'):
            self._meta_file.close()

        print(f"Recording session stopped: {self.session_dir}")
```

## 4. Data Format Comparison

| Format | Size | Parsing Speed | Compatibility | Schema | Recommended Use |
|--------|------|---------------|---------------|--------|-----------------|
| CSV | High | Slow | Best | None | Debugging, small-scale |
| Binary | Lowest | Fastest | Low | Fixed | Real-time streaming |
| Protobuf | Low | Fast | Medium | Yes | Production, large-scale |
| JSON | Very High | Slow | High | Flexible | Configuration, metadata |

## References

- [Protocol Buffers Documentation](https://developers.google.com/protocol-buffers)
- [GetStream WebRTC Android](https://github.com/GetStream/webrtc-android)
- [aiortc - Python WebRTC](https://github.com/aiortc/aiortc)
- [EuRoC Dataset Format](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
