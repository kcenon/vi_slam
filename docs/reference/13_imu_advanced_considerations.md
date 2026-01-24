# Smartphone IMU: Advanced Considerations

This document is supplementary material to [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md), covering additional technical and operational considerations.

---

## 1. Sampling Rate Jitter and Timestamp Issues

### 1.1 Android Sampling Instability

Android OS **does not guarantee** the requested sampling rate.

```
+-------------------------------------------------------------+
|                    Observed Phenomena                        |
+-------------------------------------------------------------+
| - Requested: 200Hz -> Actual: 180-220Hz variation           |
| - Timestamp jitter: several ms error                        |
| - Intermittent sample drops or duplicates                   |
| - Sudden delays during background transitions               |
+-------------------------------------------------------------+
```

**Jitter Cause Classification**:
| Cause | Description | Impact |
|------|------|------|
| OS Scheduling | SensorManager callback delay | Irregular arrival times |
| Hardware Clock | Sensor-CPU clock drift | Accumulated time error |
| Buffering | Sensor batch processing | Burst arrivals |
| Power Management | CPU power-saving mode transitions | Long delay spikes |

### 1.2 Jitter Mitigation Strategies

```kotlin
class TimestampCorrector {
    private val expectedIntervalNs: Long = 5_000_000  // 200Hz = 5ms
    private var lastCorrectedTimestamp: Long = 0
    private val maxJitterNs: Long = 2_000_000  // 2ms allowed jitter

    fun correctTimestamp(rawTimestamp: Long): Long {
        if (lastCorrectedTimestamp == 0L) {
            lastCorrectedTimestamp = rawTimestamp
            return rawTimestamp
        }

        val expectedTimestamp = lastCorrectedTimestamp + expectedIntervalNs
        val diff = rawTimestamp - expectedTimestamp

        val correctedTimestamp = when {
            // Within normal range
            abs(diff) <= maxJitterNs -> rawTimestamp

            // Too early (probably previous sample dropped)
            diff < -maxJitterNs -> expectedTimestamp

            // Too late (delay occurred)
            else -> rawTimestamp  // Use actual time
        }

        lastCorrectedTimestamp = correctedTimestamp
        return correctedTimestamp
    }
}

// Sampling frequency monitoring
class SamplingMonitor {
    private val intervals = CircularBuffer<Long>(100)

    fun recordSample(timestamp: Long) {
        // Collect statistics and detect anomalies
        val stats = intervals.getStatistics()
        if (stats.stdDev > threshold) {
            Log.w("IMU", "High sampling jitter detected: ${stats.stdDev}ns")
        }
    }
}
```

### 1.3 Handling Android 12+ Restrictions

```kotlin
// Android 12+: 200Hz limit and permission required
// HIGH_SAMPLING_RATE_SENSORS permission allows up to ~400-500Hz

class HighRateSensorManager(private val context: Context) {

    fun requestHighSamplingRate(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // Check permission
            val hasPermission = ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.HIGH_SAMPLING_RATE_SENSORS
            ) == PackageManager.PERMISSION_GRANTED

            if (!hasPermission) {
                // Permission request needed
                Log.w("IMU", "HIGH_SAMPLING_RATE_SENSORS permission required")
            }
            hasPermission
        } else {
            true  // Android 11 and below: no restriction
        }
    }
}
```

**Reference**: [Assessing Jitter in Sensor Time Series from Android Mobile Devices](https://ieeexplore.ieee.org/document/7501679/)

---

## 2. Magnetometer Fusion and Indoor Interference

### 2.1 Role and Limitations of Magnetometer

```
                  +-----------------------------------------+
                  |         9-DOF IMU Configuration          |
                  +-----------------------------------------+
                  |  Accelerometer (3-axis) -> Roll, Pitch estimation |
                  |  Gyroscope (3-axis) -> Angular velocity, attitude change |
                  |  Magnetometer (3-axis) -> Yaw (Heading) absolute reference |
                  +-----------------------------------------+

Advantages:
- Eliminates Yaw drift (accumulates when using gyro only)
- Provides absolute heading reference

Issues:
- Indoor magnetic field distortion (rebar, electronics)
- Soft/hard iron errors
- Dynamic interference (elevators, motors)
```

### 2.2 Indoor Magnetic Interference Characteristics

| Environment | Interference Level | Yaw Error |
|------|----------|----------|
| Outdoor (rural) | Low | < 5 deg |
| Outdoor (urban) | Medium | 5-15 deg |
| Indoor (wood/low-rise) | Medium | 10-30 deg |
| Indoor (reinforced concrete) | High | 30-90 deg |
| Elevator/vehicle interior | Very High | > 90 deg (unusable) |

### 2.3 Adaptive Magnetometer Fusion

```python
import numpy as np

class AdaptiveMagnetometerFusion:
    """
    Magnetic field interference detection and adaptive fusion
    """
    def __init__(self):
        self.expected_field_strength = 50.0  # uT (Earth's magnetic field average)
        self.field_strength_tolerance = 15.0  # uT

        # Fusion weights
        self.mag_weight = 1.0
        self.gyro_weight = 1.0

    def detect_interference(self, mag_reading: np.ndarray) -> float:
        """
        Detect magnetic field anomaly (0: normal, 1: full interference)
        """
        field_strength = np.linalg.norm(mag_reading)

        deviation = abs(field_strength - self.expected_field_strength)
        interference_level = min(1.0, deviation / self.field_strength_tolerance)

        return interference_level

    def adaptive_fusion(self,
                        gyro_yaw: float,
                        mag_yaw: float,
                        mag_reading: np.ndarray) -> float:
        """
        Adaptive Yaw estimation based on interference level
        """
        interference = self.detect_interference(mag_reading)

        # Rely more on gyro when interference is high
        effective_mag_weight = self.mag_weight * (1 - interference)
        effective_gyro_weight = self.gyro_weight + interference

        total_weight = effective_mag_weight + effective_gyro_weight

        fused_yaw = (
            effective_mag_weight * mag_yaw +
            effective_gyro_weight * gyro_yaw
        ) / total_weight

        return fused_yaw

    def update_field_reference(self, mag_reading: np.ndarray):
        """
        Update reference magnetic field in stable environment
        """
        if self.detect_interference(mag_reading) < 0.2:
            # Update only when interference is low
            self.expected_field_strength = 0.9 * self.expected_field_strength + \
                                           0.1 * np.linalg.norm(mag_reading)
```

### 2.4 Magnetometer Usage Recommendations for VI-SLAM

```
+-------------------------------------------------------------+
|               Magnetometer Usage Decision Flowchart          |
+-------------------------------------------------------------+
|                                                             |
|  Outdoor + Long trajectory?  --YES--> Use magnetometer (prevent Yaw drift) |
|       |                                                     |
|       NO                                                    |
|       v                                                     |
|  Indoor environment?  --YES--> Disable magnetometer or use adaptive fusion |
|       |                                                     |
|       NO                                                    |
|       v                                                     |
|  Loop Closure possible?  --YES--> Visual-based correction without magnetometer |
|       |                                                     |
|       NO                                                    |
|       v                                                     |
|  GPS available?  --YES--> Correct Heading with GPS          |
|                                                             |
+-------------------------------------------------------------+
```

**Reference**: [A Fusion Method for Combining Low-Cost IMU/Magnetometer Outputs](https://www.mdpi.com/1424-8220/18/8/2616)

---

## 3. Zero Velocity Update (ZUPT)

### 3.1 ZUPT Principle

```
When stationary state detected:
- Actual velocity = 0
- Measured velocity != 0 (due to drift)
- Difference = Velocity error -> Used for bias estimation

       +---------------------------------------------------+
       |                   ZUPT Effect                      |
       |                                                   |
       |  Error |      Normal INS                          |
       |       |            /                              |
       |       |           /                               |
       |       |          /                                |
       |       |   ZUPT  /                                 |
       |       |    v   /   v ZUPT                         |
       |       |    o--o----o--o----o                      |
       |       +------------------------- Time             |
       +---------------------------------------------------+
```

### 3.2 Stationary State Detection Algorithm

```kotlin
class StationaryDetector(
    // Threshold settings
    private val accelVarianceThreshold: Float = 0.1f,     // m/s^2
    private val gyroVarianceThreshold: Float = 0.01f,     // rad/s
    private val accelMagnitudeThreshold: Float = 0.5f,    // |a| - g
    private val windowSizeMs: Long = 200
) {
    private val accelBuffer = mutableListOf<FloatArray>()
    private val gyroBuffer = mutableListOf<FloatArray>()

    fun addReading(accel: FloatArray, gyro: FloatArray, timestamp: Long) {
        accelBuffer.add(accel)
        gyroBuffer.add(gyro)

        // Maintain window size
        pruneOldReadings(timestamp)
    }

    fun isStationary(): Boolean {
        if (accelBuffer.size < 10) return false

        // 1. Check acceleration magnitude (only gravity should be present)
        val accelMagnitudes = accelBuffer.map { sqrt(it[0]*it[0] + it[1]*it[1] + it[2]*it[2]) }
        val avgMagnitude = accelMagnitudes.average()
        if (abs(avgMagnitude - 9.81) > accelMagnitudeThreshold) {
            return false
        }

        // 2. Check acceleration variance
        val accelVariance = calculateVariance(accelBuffer)
        if (accelVariance > accelVarianceThreshold) {
            return false
        }

        // 3. Check gyro variance
        val gyroVariance = calculateVariance(gyroBuffer)
        if (gyroVariance > gyroVarianceThreshold) {
            return false
        }

        return true
    }

    private fun calculateVariance(buffer: List<FloatArray>): Float {
        // Sum of variance for each axis
        val variances = (0..2).map { axis ->
            val values = buffer.map { it[axis] }
            val mean = values.average()
            values.map { (it - mean) * (it - mean) }.average()
        }
        return variances.sum().toFloat()
    }
}
```

### 3.3 Limitations of ZUPT on Smartphones

| Scenario | ZUPT Applicability | Reason |
|----------|-----------------|------|
| IMU mounted on foot | High | Stationary period exists with each step |
| Smartphone held in hand | Low | Rare stationary periods due to hand shake |
| Stationary on table | High | Clear stationary state |
| Inside vehicle | Medium | Can be used during signal wait |

**Alternatives for Smartphones**:
```
1. Quasi-static detection: Use "almost stationary" state instead of complete stop
2. Motion Constraint: Limit velocity on specific axis (e.g., vertical velocity ~ 0)
3. Visual stationary detection: Combine with camera-based motion detection
```

**Reference**: [Robust Pedestrian Dead Reckoning Based on MEMS-IMU](https://pmc.ncbi.nlm.nih.gov/articles/PMC5982656/)

---

## 4. Power Consumption and Battery Optimization

### 4.1 Power Consumption by Sensor

| Component | Current Draw | Notes |
|-----------|----------|------|
| IMU (high performance mode) | 0.5-1.0 mA | 200Hz continuous |
| IMU (low power mode) | 0.03-0.1 mA | Batch/event-based |
| Camera (30fps) | 200-400 mA | Largest consumer |
| GPS (continuous) | 30-50 mA | Outdoor use |
| CPU (SLAM processing) | 100-500 mA | Algorithm dependent |

### 4.2 Battery Optimization Strategies

```kotlin
class PowerAwareIMUManager(private val context: Context) {

    enum class PowerMode {
        HIGH_PERFORMANCE,    // 200Hz, maximum accuracy
        BALANCED,            // 100Hz, battery saving
        LOW_POWER,           // 50Hz + batching
        ULTRA_LOW_POWER      // Event-based only
    }

    private var currentMode = PowerMode.HIGH_PERFORMANCE

    fun setMode(mode: PowerMode) {
        currentMode = mode

        when (mode) {
            PowerMode.HIGH_PERFORMANCE -> {
                setSamplingRate(200)
                setBatching(false)
                setWakeLock(true)
            }
            PowerMode.BALANCED -> {
                setSamplingRate(100)
                setBatching(false)
                setWakeLock(true)
            }
            PowerMode.LOW_POWER -> {
                setSamplingRate(50)
                setBatching(true, batchLatencyMs = 1000)
                setWakeLock(false)
            }
            PowerMode.ULTRA_LOW_POWER -> {
                // Activate only on motion detection
                enableSignificantMotionTrigger()
                setWakeLock(false)
            }
        }
    }

    // Utilize Android sensor batching
    private fun setBatching(enabled: Boolean, batchLatencyMs: Int = 0) {
        val maxReportLatencyUs = if (enabled) batchLatencyMs * 1000 else 0

        sensorManager.registerListener(
            listener,
            accelerometer,
            samplingPeriodUs,
            maxReportLatencyUs  // Batching delay
        )
    }
}
```

### 4.3 Adaptive Power Management

```
+--------------------------------------------------------------+
|               Context-Aware Power Management                   |
+--------------------------------------------------------------+
|                                                              |
|  [Stationary state detected] --> Low power mode (50Hz, batching) |
|        |                                                     |
|        v                                                     |
|  [Motion detected] --> High performance mode (200Hz)         |
|        |                                                     |
|        v                                                     |
|  [Visual tracking failure] --> IMU only, maximum performance |
|        |                                                     |
|        v                                                     |
|  [Battery < 20%] --> Forced low power mode                   |
|                                                              |
+--------------------------------------------------------------+
```

**Expected Battery Life** (based on 3000mAh battery):
| Mode | Expected Usage Time |
|------|---------------|
| Camera + IMU (max performance) | 2-3 hours |
| Camera + IMU (power saving) | 4-6 hours |
| IMU only (high performance) | 10-15 hours |
| IMU only (low power) | 24+ hours |

**Reference**: [Android Battery Consumption Guidelines](https://developer.android.com/docs/quality-guidelines/build-for-billions/battery-consumption)

---

## 5. Failure Recovery and Auxiliary Sensor Fusion

### 5.1 VI-SLAM Failure Types

| Failure Type | Cause | Symptoms |
|-----------|------|------|
| Visual tracking failure | Fast rotation, motion blur, low light | Sudden feature count drop |
| Scale drift | Long-term operation, IMU error accumulation | Trajectory size mismatch |
| Yaw drift | Gyro bias | Gradual heading error |
| Initialization failure | Insufficient parallax, stationary state | Scale estimation impossible |

### 5.2 Relocalization Strategy

```python
class RelocalizationManager:
    def __init__(self, keyframe_database):
        self.keyframe_db = keyframe_database
        self.lost_frame_count = 0
        self.max_lost_frames = 30  # 1 second @ 30fps

    def check_tracking_status(self, current_frame):
        """
        Check tracking status and attempt recovery
        """
        tracked_features = current_frame.get_tracked_count()

        if tracked_features < 10:
            self.lost_frame_count += 1

            if self.lost_frame_count > self.max_lost_frames:
                return self.attempt_relocalization(current_frame)
        else:
            self.lost_frame_count = 0
            return TrackingStatus.OK

        return TrackingStatus.TRACKING_LOST

    def attempt_relocalization(self, current_frame):
        """
        Search for similar frame in keyframe DB
        """
        # 1. BoW (Bag of Words) based search
        candidates = self.keyframe_db.query_similar(
            current_frame.get_bow_vector(),
            top_k=5
        )

        # 2. Match features for each candidate
        for candidate in candidates:
            matches = feature_matcher.match(
                current_frame.descriptors,
                candidate.descriptors
            )

            if len(matches) > 50:
                # 3. Estimate pose with PnP
                success, pose = estimate_pose_pnp(
                    current_frame.keypoints,
                    candidate.map_points,
                    matches
                )

                if success:
                    return TrackingStatus.RELOCALIZED, pose

        return TrackingStatus.RELOCALIZATION_FAILED
```

### 5.3 GPS Fusion (VINS-Fusion Method)

```yaml
# VINS-Fusion GPS Configuration
# config/gps_fusion.yaml

gnss_enable: 1
gnss_meas_freq: 10            # GPS update frequency (Hz)

# GPS-VIO Loosely Coupled
gnss_local_online_sync: 1     # Local coordinate synchronization
gnss_mode: 2                  # 0: none, 1: position only, 2: position + heading

# GPS Dropout handling
gnss_max_dropout_time: 5.0    # Maximum time using VIO only without GPS signal
gnss_position_std_threshold: 5.0  # GPS reliability threshold (m)
```

**GPS Fusion Architecture**:
```
+----------------------------------------------------------------+
|                    GPS-VIO Tightly Coupled                      |
+----------------------------------------------------------------+
|                                                                |
|   Camera ---> Visual Features ---+                              |
|                                  +---> Graph Optimization ---> Final Pose |
|   IMU -------> Preintegration ---+                              |
|                              |                                  |
|   GPS -------> Position Factor --+                              |
|                                                                |
|   Advantages:                                                   |
|   - Continuous tracking with VIO during GPS dropout            |
|   - Global coordinate alignment                                 |
|   - Long-range drift elimination                               |
|                                                                |
+----------------------------------------------------------------+
```

### 5.4 Loop Closure Strategy

```python
class LoopClosureDetector:
    """
    Loop detection for long-range drift correction
    """
    def __init__(self):
        self.min_loop_distance = 10  # Minimum 10 frame interval
        self.similarity_threshold = 0.8

    def detect_loop(self, current_keyframe, all_keyframes):
        """
        Search for past keyframes similar to current keyframe
        """
        current_idx = len(all_keyframes) - 1

        for i, past_kf in enumerate(all_keyframes[:-self.min_loop_distance]):
            # Check time interval
            if current_idx - i < self.min_loop_distance:
                continue

            # Appearance similarity
            similarity = self.compute_similarity(current_keyframe, past_kf)

            if similarity > self.similarity_threshold:
                # Geometric verification
                if self.geometric_verification(current_keyframe, past_kf):
                    return LoopCandidate(
                        current_idx=current_idx,
                        loop_idx=i,
                        similarity=similarity
                    )

        return None

    def apply_loop_correction(self, loop_candidate, pose_graph):
        """
        Correct drift through pose graph optimization
        """
        # Add loop edge
        pose_graph.add_loop_constraint(
            loop_candidate.current_idx,
            loop_candidate.loop_idx,
            relative_pose=loop_candidate.relative_pose
        )

        # Optimize entire graph
        optimized_poses = pose_graph.optimize()

        return optimized_poses
```

**Reference**: [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

---

## 6. Vehicle/Vibration Environment Handling

### 6.1 Vehicle Vibration Characteristics

```
Main vibration sources in vehicle environments:
+-------------------------------------------------------------+
|  Frequency Range   |  Cause                    |  Impact          |
+-------------------------------------------------------------+
|  10-30 Hz          |  Engine idle              |  Low-freq noise  |
|  30-100 Hz         |  Road surface bumps       |  Accel distortion |
|  100-500 Hz        |  High RPM engine, tires   |  High-freq noise |
|  > 500 Hz          |  Electronic equipment     |  Needs filtering |
+-------------------------------------------------------------+
```

### 6.2 Vibration Filtering

```python
import numpy as np
from scipy import signal

class VibrationFilter:
    """
    Vehicle vibration compensation filter
    """
    def __init__(self, sample_rate=200):
        self.fs = sample_rate

        # Low-pass filter (remove engine vibration)
        self.lpf_cutoff = 20  # Hz
        self.b_lpf, self.a_lpf = signal.butter(
            4, self.lpf_cutoff / (self.fs / 2), btype='low'
        )

        # Adaptive notch filter (remove specific frequencies)
        self.notch_freqs = []  # Detected dynamically

        # State variables
        self.zi_lpf = signal.lfilter_zi(self.b_lpf, self.a_lpf)

    def detect_vibration_frequencies(self, data, window_size=256):
        """
        Detect main vibration frequencies using FFT
        """
        freqs, psd = signal.welch(data, self.fs, nperseg=window_size)

        # Peak detection
        peaks, _ = signal.find_peaks(psd, height=np.mean(psd) * 3)

        # Only peaks above 10Hz (exclude actual movement)
        vibration_freqs = [freqs[p] for p in peaks if freqs[p] > 10]

        return vibration_freqs

    def apply_adaptive_filter(self, accel_data):
        """
        Adaptive filtering
        """
        # 1. Detect vibration frequencies
        vibration_freqs = self.detect_vibration_frequencies(accel_data)

        # 2. Apply notch filter (for each detected frequency)
        filtered = accel_data.copy()
        for freq in vibration_freqs:
            b_notch, a_notch = signal.iirnotch(freq, Q=30, fs=self.fs)
            filtered = signal.filtfilt(b_notch, a_notch, filtered)

        # 3. Low-pass filter
        filtered, self.zi_lpf = signal.lfilter(
            self.b_lpf, self.a_lpf, filtered, zi=self.zi_lpf * filtered[0]
        )

        return filtered
```

### 6.3 Special Considerations for Vehicle VI-SLAM

| Consideration | Description | Countermeasure |
|----------|------|--------|
| High-speed motion | Motion blur, difficult feature tracking | Short exposure, Global Shutter |
| Vibration | Increased IMU noise | Adaptive filter, low-pass |
| Rapid lighting changes | Tunnel entry/exit | Auto Exposure, HDR |
| GPS shadows | Tunnels, under overpasses | Bridge with VIO |
| Repetitive environments | Highways, parking lots | GPS fusion, map-based localization |

**Reference**: [Vehicle Vibration Error Compensation on IMU](https://www.researchgate.net/publication/330373155_Vehicle_Vibration_Error_Compensation_on_IMU-accelerometer_Sensor_Using_Adaptive_Filter_and_Low-pass_Filter_Approaches)

---

## 7. Hardware-Level Considerations

### 7.1 IMU Chipset Characteristics by Smartphone

| Manufacturer | Representative Chipset | Characteristics |
|--------|----------|------|
| Bosch | BMI260/270 | Low power, built-in smart features |
| STMicroelectronics | LSM6DSO | High precision, low noise |
| InvenSense (TDK) | ICM-42688 | Low bias drift |
| Qualcomm | Integrated sensor hub | SoC integrated |

### 7.2 Recommended Test Procedure

```
+--------------------------------------------------------------+
|               Smartphone IMU Quality Assessment Procedure      |
+--------------------------------------------------------------+
|                                                              |
|  1. Static Test (15-24 hours)                                |
|     - Allan Variance analysis                                |
|     - Noise parameter extraction                             |
|     - Temperature stability check                            |
|                                                              |
|  2. Dynamic Test (using calibration board)                   |
|     - Scale factor verification                              |
|     - Axis misalignment measurement                          |
|     - Camera-IMU time offset                                 |
|                                                              |
|  3. Sampling Quality Test                                    |
|     - Frequency stability measurement                        |
|     - Timestamp jitter analysis                              |
|     - Dropout occurrence frequency                           |
|                                                              |
|  4. Integration Test                                         |
|     - EuRoC dataset comparison                               |
|     - Real environment trajectory accuracy                   |
|                                                              |
+--------------------------------------------------------------+
```

---

## 8. Summary: Additional Considerations Checklist

| Category | Check Item | Importance |
|----------|----------|--------|
| **Sampling Quality** | Timestamp jitter monitoring | High |
| | Sample dropout detection | High |
| | Android 12+ permission handling | Medium |
| **Sensor Fusion** | Magnetometer interference detection | Environment dependent |
| | Adaptive fusion weights | Medium |
| **Stationary Detection** | ZUPT implementation (if applicable) | Low-Medium |
| | Quasi-static detection | Medium |
| **Power Management** | Context-aware power mode | Medium |
| | Sensor batching utilization | Low |
| **Failure Recovery** | Relocalization implementation | High |
| | Loop Closure | High |
| | GPS fusion (if applicable) | Environment dependent |
| **Environment Adaptation** | Vibration filtering (vehicle) | Environment dependent |
| | Lighting change handling | Medium |

---

## References

### Sampling and Timestamps
- [Assessing Jitter in Sensor Time Series from Android](https://ieeexplore.ieee.org/document/7501679/)
- [Mobile AR Sensor Logger](https://arxiv.org/pdf/2001.00470)

### Magnetometer and Sensor Fusion
- [A Fusion Method for Combining Low-Cost IMU/Magnetometer](https://www.mdpi.com/1424-8220/18/8/2616)
- [Indoor Localization Methods for Smartphones](https://www.mdpi.com/1424-8220/25/6/1806)

### ZUPT and Pedestrian Navigation
- [Robust Pedestrian Dead Reckoning Based on MEMS-IMU](https://pmc.ncbi.nlm.nih.gov/articles/PMC5982656/)
- [Pseudo-Zero Velocity Re-Detection ZUPT](https://ieeexplore.ieee.org/document/9391710/)

### Failure Recovery and GPS Fusion
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [GPS-Visual-Inertial Odometry](https://arxiv.org/pdf/2203.02677)

### Vibration Environment
- [Vehicle Vibration Error Compensation on IMU](https://www.researchgate.net/publication/330373155_Vehicle_Vibration_Error_Compensation_on_IMU-accelerometer_Sensor_Using_Adaptive_Filter_and_Low-pass_Filter_Approaches)
- [INS and Vibration](https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/APN-112-Inertial-Navigation-Systems-and-Vibration/APN-112-Inertial-Navigation-Systems-and-Vibration.pdf)

### Related Documents
- [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [09_timestamp_synchronization.md](09_timestamp_synchronization.md)
