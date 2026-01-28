# Recorder Module

Recorder module for VI-SLAM Android application. Records synchronized camera frames and IMU data to local storage.

## Features

- H.264 video encoding to MP4
- IMU data recording to CSV format
- Metadata generation (JSON)
- **Session recovery** for interrupted recordings
- Thread-safe state management

## Components

### IRecorder

Interface defining the recorder contract. All recorders must implement this interface.

### LocalRecorder

Main implementation that records data to local storage.

**Usage**:
```kotlin
val context = applicationContext
val recorder = LocalRecorder(context)

val config = RecorderConfig(
    outputDirectory = File(getExternalFilesDir(null), "recordings"),
    videoWidth = 640,
    videoHeight = 480,
    videoFps = 30,
    videoBitrate = 2_000_000,
    maxDurationMs = 60 * 60 * 1000  // 60 minutes
)

recorder.initialize(config).onSuccess {
    recorder.startRecording().onSuccess { info ->
        Log.i(TAG, "Recording started: ${info.recordingId}")
    }
}

// Later...
recorder.stopRecording().onSuccess { summary ->
    Log.i(TAG, "Recorded ${summary.frameCount} frames")
    Log.i(TAG, "Output: ${summary.videoFile}")
}
```

### Session Recovery

LocalRecorder supports recovery of interrupted recording sessions. Sessions can be interrupted due to:
- App crashes
- System kills (low memory)
- Unexpected device shutdown
- Force stop by user

#### How Recovery Works

1. **Session State Tracking**: Recording state is persisted to SharedPreferences:
   - On recording start (initial state)
   - Every 100 frames (periodic checkpoint)
   - On recording stop (state cleared)

2. **Recovery Detection**: On app initialization, incomplete sessions are detected:
   ```kotlin
   val recoverableSessions = recorder.listRecoverableSessions()
   ```

3. **Data Recovery**: Recover interrupted sessions:
   ```kotlin
   recorder.recoverSession(sessionId).onSuccess { result ->
       Log.i(TAG, "Recovered ${result.recoveredFrameCount} frames")
       Log.i(TAG, "Recovered ${result.recoveredImuCount} IMU samples")

       if (result.videoRecovered) {
           Log.i(TAG, "Video file: ${result.videoFile}")
       }

       if (result.imuRecovered) {
           Log.i(TAG, "IMU file: ${result.imuFile}")
       }
   }
   ```

#### Recovery Operations

**IMU CSV Recovery**:
- Validates CSV header
- Removes truncated/incomplete lines
- Removes empty lines
- Validates data format (timestamp, sensor type, numeric values)
- Repairs file in-place

**MP4 Video Recovery**:
- Checks if file is playable using MediaExtractor
- Extracts available metadata (duration, frame count)
- Future: Implement full MP4 repair using MediaMuxer re-muxing

**Metadata Generation**:
- Creates recovery metadata with:
  - Recovery timestamp
  - Recovery status (video/IMU)
  - Estimated vs recovered counts
  - Warnings for failed recoveries

#### Recovery Example

```kotlin
class MainActivity : AppCompatActivity() {
    private lateinit var recorder: LocalRecorder

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        recorder = LocalRecorder(applicationContext)

        // Check for recoverable sessions on app start
        val recoverableSessions = recorder.listRecoverableSessions()

        if (recoverableSessions.isNotEmpty()) {
            AlertDialog.Builder(this)
                .setTitle("Recover Recording?")
                .setMessage("Found ${recoverableSessions.size} incomplete recording(s)")
                .setPositiveButton("Recover") { _, _ ->
                    recoverAllSessions(recoverableSessions)
                }
                .setNegativeButton("Discard", null)
                .show()
        }
    }

    private fun recoverAllSessions(sessions: List<RecoverableSession>) {
        lifecycleScope.launch {
            sessions.forEach { session ->
                recorder.recoverSession(session.recordingId).onSuccess { result ->
                    if (result.success) {
                        Log.i(TAG, "Recovered session: ${session.recordingId}")
                    } else {
                        Log.w(TAG, "Failed to recover: ${result.errorMessage}")
                    }
                }
            }
        }
    }
}
```

## Data Models

### RecorderConfig

Configuration for recording sessions.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| outputDirectory | File | - | Base directory for recordings (must exist) |
| videoFormat | VideoFormat | H264_MP4 | Video encoding format |
| imuFormat | ImuFormat | CSV | IMU data format |
| maxDurationMs | Long | 3,600,000 | Max recording duration (ms) |
| enableMetadata | Boolean | true | Generate metadata.json |
| videoWidth | Int | 640 | Video width (pixels) |
| videoHeight | Int | 480 | Video height (pixels) |
| videoFps | Int | 30 | Target frame rate |
| videoBitrate | Int | 2,000,000 | Bitrate (bits/sec) |

### RecordingInfo

Information about an active recording session.

| Field | Type | Description |
|-------|------|-------------|
| recordingId | String | Unique session identifier |
| startTime | Long | Start timestamp (ms) |
| outputPath | String | Session output directory |
| config | RecorderConfig | Configuration used |

### RecordingSummary

Summary of a completed recording session.

| Field | Type | Description |
|-------|------|-------------|
| recordingId | String | Session identifier |
| success | Boolean | True if completed successfully |
| frameCount | Long | Total frames recorded |
| imuSampleCount | Long | Total IMU samples recorded |
| durationMs | Long | Actual duration (ms) |
| outputFiles | List<String> | Generated file paths |
| videoFile | String? | MP4 video file path |
| imuFile | String? | CSV IMU file path |
| metadataFile | String? | JSON metadata file path |
| errorMessage | String? | Error description (if failed) |

**Methods**:
- `averageFps(): Double` - Calculate average frame rate
- `averageImuRate(): Double` - Calculate average IMU sampling rate

### RecoverableSession

Information about a recoverable recording session.

| Field | Type | Description |
|-------|------|-------------|
| recordingId | String | Session identifier |
| startTime | Long | Start timestamp (ms) |
| outputPath | String | Session directory |
| estimatedFrameCount | Long | Last known frame count |
| estimatedImuCount | Long | Last known IMU sample count |
| lastUpdateTime | Long | Last state update timestamp |

### RecoveryResult

Result of a session recovery operation.

| Field | Type | Description |
|-------|------|-------------|
| recordingId | String | Session identifier |
| success | Boolean | True if any data recovered |
| recoveredFrameCount | Long | Frames recovered from video |
| recoveredImuCount | Long | IMU samples recovered |
| videoRecovered | Boolean | True if video playable |
| imuRecovered | Boolean | True if IMU data valid |
| metadataGenerated | Boolean | True if metadata created |
| outputFiles | List<String> | Recovered file paths |
| videoFile | String? | Recovered video path |
| imuFile | String? | Recovered IMU path |
| metadataFile | String? | Generated metadata path |
| errorMessage | String? | Error description (if failed) |

## Output Format

### Directory Structure

```
<outputDirectory>/
├── recording_<timestamp>_<uuid>/
│   ├── video.mp4          # H.264 encoded video
│   ├── imu_data.csv       # IMU samples with nanosecond timestamps
│   └── metadata.json      # Session metadata
```

### IMU CSV Format

```csv
timestamp_ns,sensor_type,x,y,z
1234567890000000,accel,0.1,0.2,9.8
1234567890500000,gyro,0.01,-0.02,0.00
```

- `timestamp_ns`: Nanosecond timestamp
- `sensor_type`: "accel" (accelerometer) or "gyro" (gyroscope)
- `x, y, z`: Measurement values in device coordinates

### Metadata JSON Format

Normal recording:
```json
{
  "recording_id": "20250126_143052_a1b2c3d4",
  "start_time": 1706284252000,
  "duration_ms": 30000,
  "frame_count": 900,
  "imu_sample_count": 6000,
  "device": {
    "manufacturer": "Samsung",
    "model": "SM-G998B",
    "android_version": "13",
    "sdk_int": 33
  },
  "camera": {
    "width": 640,
    "height": 480,
    "fps": 30,
    "bitrate": 2000000,
    "format": "H264_MP4"
  },
  "imu": {
    "format": "CSV"
  },
  "output_files": {
    "video": "video.mp4",
    "imu": "imu_data.csv",
    "metadata": "metadata.json"
  }
}
```

Recovered recording:
```json
{
  "recording_id": "20250126_143052_a1b2c3d4",
  "start_time": 1706284252000,
  "estimated_duration_ms": 30000,
  "recovered_frame_count": 850,
  "recovered_imu_count": 5800,
  "recovered": true,
  "recovery_timestamp": 1706284300000,
  "recovery_status": {
    "video_recovered": true,
    "imu_recovered": true
  },
  "device": { ... },
  "output_files": {
    "video": "video.mp4",
    "imu": "imu_data.csv",
    "metadata": "metadata.json"
  },
  "warnings": []
}
```

## Recovery Limitations

### Current Implementation

1. **MP4 Recovery**: Currently checks if file is playable. Full MP4 repair (re-muxing) is not yet implemented.
2. **Data Loss**: Some data may be lost between last state update and crash (up to 100 frames).
3. **Partial Frames**: Last frame being encoded may be incomplete.

### Future Enhancements

1. Implement full MP4 repair using MediaMuxer
2. Extract raw H.264 frames if MP4 structure is corrupt
3. Reduce state update interval for less data loss
4. Add fsync() calls for critical data

## Thread Safety

- All state transitions are synchronized
- Atomic counters used for statistics
- Session state updates are synchronized via SessionStateManager
- Safe to call from multiple threads

## Error Handling

- All methods return `Result<T>` for explicit error handling
- Individual frame/sample failures don't stop recording
- State machine prevents invalid transitions
- Detailed error messages in logs

## Testing

### Unit Tests

- CSV validation and repair tests
- MP4 recovery tests
- Data model tests
- State management tests

**Run tests**:
```bash
./gradlew :app:testDebugUnitTest --tests "RecoveryTest"
```

### Integration Tests

Integration tests require Android framework (MediaCodec, MediaMuxer) and should be run as instrumented tests:

```bash
./gradlew :app:connectedDebugAndroidTest
```

## Dependencies

- Kotlin Standard Library
- Android MediaCodec (H.264 encoding)
- Android MediaMuxer (MP4 muxing)
- Android MediaExtractor (MP4 recovery)
- Gson (JSON generation)
- SharedPreferences (session state persistence)

## License

See project LICENSE file.
