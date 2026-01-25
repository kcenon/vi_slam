# Streaming Module

This module implements UDP IMU streaming and WebRTC DataChannel metadata transmission for the VI-SLAM Android application.

## Components

### ImuData
Binary IMU packet format implementation for sensor data transmission.

**Packet Format** (56 bytes, little-endian):
| Offset | Size | Type      | Field           |
|--------|------|-----------|-----------------|
| 0      | 8    | int64     | timestamp_ns    |
| 8      | 24   | double[3] | acceleration (m/s²) |
| 32     | 24   | double[3] | gyro (rad/s)    |

**Usage**:
```kotlin
val imuData = ImuData(
    timestampNs = System.nanoTime(),
    accelerationX = 9.81,
    accelerationY = 0.0,
    accelerationZ = 0.0,
    gyroX = 0.0,
    gyroY = 0.0,
    gyroZ = 0.0
)

// Convert to binary packet
val packet = imuData.toPacket() // 56 bytes

// Decode from binary packet
val decoded = ImuData.fromPacket(packet)
```

### UdpImuStreamer
UDP-based IMU data transmission at 200Hz target rate.

**Features**:
- Non-blocking I/O using Kotlin coroutines
- Packet loss monitoring (<1% target)
- Thread-safe start/stop operations
- Real-time statistics tracking

**Usage**:
```kotlin
val streamer = UdpImuStreamer("192.168.1.100", 15000)

// Start streaming
streamer.start()

// Send IMU measurements
streamer.sendImuData(imuData)

// Get statistics
val stats = streamer.getStatistics()
Log.d(TAG, "Sent ${stats.packetsSent} packets at ${stats.averagePacketsPerSecond} pps")

// Stop streaming
streamer.stop()
```

### FrameMetadata
Frame synchronization metadata for WebRTC video stream.

**JSON Format** (compact):
```json
{
  "seq": 12345,
  "ts": 1234567890123456,
  "w": 1920,
  "h": 1080
}
```

**Usage**:
```kotlin
val metadata = FrameMetadata(
    sequenceNumber = frameCount,
    timestampNs = frameTimestamp,
    width = 1920,
    height = 1080
)

// Convert to JSON
val json = metadata.toJson()

// Decode from JSON
val decoded = FrameMetadata.fromJson(json)
```

### DataChannelMetadataSender
WebRTC DataChannel-based metadata transmission with reliable delivery.

**Features**:
- Reliable, ordered delivery via DataChannel
- Automatic retransmission
- NAT/firewall traversal support
- State monitoring

**Usage**:
```kotlin
val sender = DataChannelMetadataSender(dataChannel)

// Register state callback
sender.registerStateCallback { state ->
    Log.d(TAG, "DataChannel state: $state")
}

// Send frame metadata
if (sender.isChannelReady()) {
    sender.sendMetadata(frameMetadata)
}

// Get statistics
val stats = sender.getStatistics()
```

## Architecture

```
┌─────────────────┐
│  IMU Sensor     │
│  200Hz          │
└────────┬────────┘
         │
         v
┌─────────────────┐       UDP Socket
│  UdpImuStreamer │──────────────────> PC Receiver
└─────────────────┘       (15000)


┌─────────────────┐
│  Camera Frame   │
│  30Hz           │
└────────┬────────┘
         │
         v
┌──────────────────────┐  DataChannel   ┌──────────────┐
│ DataChannelMetadata  │ ─────────────> │ PC Receiver  │
│ Sender               │  (WebRTC)      └──────────────┘
└──────────────────────┘
```

## Performance Targets

| Metric | Target | Implementation |
|--------|--------|----------------|
| IMU Rate | 200Hz | UDP streaming |
| Packet Loss | <1% | Monitored via statistics |
| Latency | <10ms | UDP (no retransmission) |
| Metadata Delivery | 100% | DataChannel (reliable) |

## Dependencies

- **Kotlin Coroutines**: Async I/O operations
- **WebRTC**: DataChannel implementation
- **org.json**: JSON serialization (Android framework)

## Testing

Unit tests are provided for core functionality:
- `ImuDataTest`: Packet encoding/decoding
- `FrameMetadataTest`: JSON serialization
- `UdpImuStreamerTest`: UDP transmission

**Note**: Tests require Android instrumentation test environment for full functionality.

## Integration Example

```kotlin
class StreamingService {
    private val imuStreamer = UdpImuStreamer("192.168.1.100", 15000)
    private lateinit var metadataSender: DataChannelMetadataSender

    suspend fun startStreaming(dataChannel: DataChannel) {
        // Start IMU streaming
        imuStreamer.start()

        // Setup metadata sender
        metadataSender = DataChannelMetadataSender(dataChannel)

        // Register IMU callback
        imuSensor.registerListener { acc, gyro, timestamp ->
            val imuData = ImuData(
                timestampNs = timestamp,
                accelerationX = acc[0],
                accelerationY = acc[1],
                accelerationZ = acc[2],
                gyroX = gyro[0],
                gyroY = gyro[1],
                gyroZ = gyro[2]
            )
            imuStreamer.sendImuData(imuData)
        }

        // Register frame callback
        camera.registerFrameCallback { image, timestamp ->
            val metadata = FrameMetadata(
                sequenceNumber = frameCount++,
                timestampNs = timestamp,
                width = image.width,
                height = image.height
            )
            metadataSender.sendMetadata(metadata)
        }
    }

    suspend fun stopStreaming() {
        imuStreamer.stop()
    }
}
```

## Troubleshooting

### High Packet Loss (>1%)
- Check network bandwidth and congestion
- Verify firewall/NAT configuration
- Reduce IMU sampling rate if needed

### DataChannel Not Ready
- Verify WebRTC peer connection state
- Check ICE candidate exchange
- Ensure DataChannel is configured as reliable

### Timestamp Synchronization Issues
- Use hardware timestamps (SENSOR_TIMESTAMP) from Android
- Verify both IMU and camera use same clock source
- Check for clock drift over long sessions

## Future Improvements

1. **H.264 Encoding**: Integrate video encoding for complete streaming pipeline
2. **Adaptive Bitrate**: Adjust quality based on network conditions
3. **Error Recovery**: Implement reconnection logic for network failures
4. **Compression**: Add optional compression for metadata (e.g., Protobuf)
5. **Multi-Destination**: Support streaming to multiple receivers
