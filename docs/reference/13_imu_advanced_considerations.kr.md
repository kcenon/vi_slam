# Smartphone IMU: Advanced Considerations

이 문서는 [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)의 보완 자료로, 추가적인 기술적/운영적 고려사항을 다룬다.

---

## 1. 샘플링 레이트 지터 및 타임스탬프 문제

### 1.1 Android 샘플링 불안정성

Android OS는 요청된 샘플링 레이트를 **보장하지 않는다**.

```
┌─────────────────────────────────────────────────────────────┐
│                    실제 관측 현상                             │
├─────────────────────────────────────────────────────────────┤
│ • 요청: 200Hz → 실제: 180-220Hz 변동                         │
│ • 타임스탬프 지터: 수 ms 오차                                 │
│ • 간헐적 샘플 누락 또는 중복                                  │
│ • 백그라운드 전환 시 급격한 지연                              │
└─────────────────────────────────────────────────────────────┘
```

**지터 원인 분류**:
| 원인 | 설명 | 영향 |
|------|------|------|
| OS 스케줄링 | SensorManager 콜백 지연 | 불규칙한 도착 시간 |
| 하드웨어 클럭 | 센서-CPU 클럭 드리프트 | 누적 시간 오차 |
| 버퍼링 | 센서 배치 처리 | 버스트 도착 |
| 전력 관리 | CPU 절전 모드 전환 | 긴 지연 스파이크 |

### 1.2 지터 완화 전략

```kotlin
class TimestampCorrector {
    private val expectedIntervalNs: Long = 5_000_000  // 200Hz = 5ms
    private var lastCorrectedTimestamp: Long = 0
    private val maxJitterNs: Long = 2_000_000  // 2ms 허용 지터

    fun correctTimestamp(rawTimestamp: Long): Long {
        if (lastCorrectedTimestamp == 0L) {
            lastCorrectedTimestamp = rawTimestamp
            return rawTimestamp
        }

        val expectedTimestamp = lastCorrectedTimestamp + expectedIntervalNs
        val diff = rawTimestamp - expectedTimestamp

        val correctedTimestamp = when {
            // 정상 범위 내
            abs(diff) <= maxJitterNs -> rawTimestamp

            // 너무 빠름 (아마 이전 샘플 누락)
            diff < -maxJitterNs -> expectedTimestamp

            // 너무 느림 (지연 발생)
            else -> rawTimestamp  // 실제 시간 사용
        }

        lastCorrectedTimestamp = correctedTimestamp
        return correctedTimestamp
    }
}

// 샘플링 주파수 모니터링
class SamplingMonitor {
    private val intervals = CircularBuffer<Long>(100)

    fun recordSample(timestamp: Long) {
        // 통계 수집 및 이상 감지
        val stats = intervals.getStatistics()
        if (stats.stdDev > threshold) {
            Log.w("IMU", "High sampling jitter detected: ${stats.stdDev}ns")
        }
    }
}
```

### 1.3 Android 12+ 제한사항 대응

```kotlin
// Android 12 이상: 200Hz 제한 및 권한 필요
// HIGH_SAMPLING_RATE_SENSORS 권한으로 최대 ~400-500Hz 가능

class HighRateSensorManager(private val context: Context) {

    fun requestHighSamplingRate(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // 권한 확인
            val hasPermission = ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.HIGH_SAMPLING_RATE_SENSORS
            ) == PackageManager.PERMISSION_GRANTED

            if (!hasPermission) {
                // 권한 요청 필요
                Log.w("IMU", "HIGH_SAMPLING_RATE_SENSORS permission required")
            }
            hasPermission
        } else {
            true  // Android 11 이하: 제한 없음
        }
    }
}
```

**참고**: [Assessing Jitter in Sensor Time Series from Android Mobile Devices](https://ieeexplore.ieee.org/document/7501679/)

---

## 2. 자력계 융합 및 실내 간섭

### 2.1 자력계의 역할과 한계

```
                  ┌─────────────────────────────────────────┐
                  │         9-DOF IMU 구성                   │
                  ├─────────────────────────────────────────┤
                  │  가속도계 (3축) → Roll, Pitch 추정       │
                  │  자이로스코프 (3축) → 각속도, 자세 변화   │
                  │  자력계 (3축) → Yaw (Heading) 절대 기준  │
                  └─────────────────────────────────────────┘

장점:
- Yaw 드리프트 제거 (자이로만 사용 시 누적)
- 절대 방위 기준 제공

문제점:
- 실내 자기장 왜곡 (철근, 전자기기)
- 소프트/하드 아이언 오차
- 동적 간섭 (엘리베이터, 모터)
```

### 2.2 실내 자기 간섭 특성

| 환경 | 간섭 수준 | Yaw 오차 |
|------|----------|----------|
| 야외 (도시 외곽) | 낮음 | < 5° |
| 야외 (도시) | 중간 | 5-15° |
| 실내 (목조/저층) | 중간 | 10-30° |
| 실내 (철근콘크리트) | 높음 | 30-90° |
| 엘리베이터/차량 내부 | 매우 높음 | > 90° (사용 불가) |

### 2.3 적응적 자력계 융합

```python
import numpy as np

class AdaptiveMagnetometerFusion:
    """
    자기장 간섭 감지 및 적응적 융합
    """
    def __init__(self):
        self.expected_field_strength = 50.0  # μT (지구 자기장 평균)
        self.field_strength_tolerance = 15.0  # μT

        # 융합 가중치
        self.mag_weight = 1.0
        self.gyro_weight = 1.0

    def detect_interference(self, mag_reading: np.ndarray) -> float:
        """
        자기장 이상 감지 (0: 정상, 1: 완전 간섭)
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
        간섭 수준에 따른 적응적 Yaw 추정
        """
        interference = self.detect_interference(mag_reading)

        # 간섭이 심할수록 자이로에 의존
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
        안정적 환경에서 기준 자기장 업데이트
        """
        if self.detect_interference(mag_reading) < 0.2:
            # 간섭 낮을 때만 업데이트
            self.expected_field_strength = 0.9 * self.expected_field_strength + \
                                           0.1 * np.linalg.norm(mag_reading)
```

### 2.4 VI-SLAM에서의 자력계 사용 권장사항

```
┌─────────────────────────────────────────────────────────────┐
│               자력계 사용 결정 플로우차트                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  야외 + 긴 궤적?  ──YES──▶ 자력계 사용 (Yaw 드리프트 방지)  │
│       │                                                     │
│       NO                                                    │
│       ▼                                                     │
│  실내 환경?  ──YES──▶ 자력계 비활성화 또는 적응적 융합      │
│       │                                                     │
│       NO                                                    │
│       ▼                                                     │
│  Loop Closure 가능?  ──YES──▶ 자력계 없이 시각 기반 보정   │
│       │                                                     │
│       NO                                                    │
│       ▼                                                     │
│  GPS 사용 가능?  ──YES──▶ GPS로 Heading 보정               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**참고**: [A Fusion Method for Combining Low-Cost IMU/Magnetometer Outputs](https://www.mdpi.com/1424-8220/18/8/2616)

---

## 3. Zero Velocity Update (ZUPT)

### 3.1 ZUPT 원리

```
정지 상태 감지 시:
- 실제 속도 = 0
- 측정된 속도 ≠ 0 (드리프트로 인해)
- 차이 = 속도 오차 → 바이어스 추정에 활용

       ┌───────────────────────────────────────────────────┐
       │                   ZUPT 효과                        │
       │                                                   │
       │  오차 │      일반 INS                              │
       │       │            ╱                              │
       │       │           ╱                               │
       │       │          ╱                                │
       │       │   ZUPT  ╱                                 │
       │       │    ↓   ╱   ↓ ZUPT                         │
       │       │    ●──●────●──●────●                      │
       │       └───────────────────────── 시간             │
       └───────────────────────────────────────────────────┘
```

### 3.2 정지 상태 감지 알고리즘

```kotlin
class StationaryDetector(
    // 임계값 설정
    private val accelVarianceThreshold: Float = 0.1f,     // m/s²
    private val gyroVarianceThreshold: Float = 0.01f,     // rad/s
    private val accelMagnitudeThreshold: Float = 0.5f,    // |a| - g
    private val windowSizeMs: Long = 200
) {
    private val accelBuffer = mutableListOf<FloatArray>()
    private val gyroBuffer = mutableListOf<FloatArray>()

    fun addReading(accel: FloatArray, gyro: FloatArray, timestamp: Long) {
        accelBuffer.add(accel)
        gyroBuffer.add(gyro)

        // 윈도우 크기 유지
        pruneOldReadings(timestamp)
    }

    fun isStationary(): Boolean {
        if (accelBuffer.size < 10) return false

        // 1. 가속도 크기 검사 (중력만 존재해야 함)
        val accelMagnitudes = accelBuffer.map { sqrt(it[0]*it[0] + it[1]*it[1] + it[2]*it[2]) }
        val avgMagnitude = accelMagnitudes.average()
        if (abs(avgMagnitude - 9.81) > accelMagnitudeThreshold) {
            return false
        }

        // 2. 가속도 분산 검사
        val accelVariance = calculateVariance(accelBuffer)
        if (accelVariance > accelVarianceThreshold) {
            return false
        }

        // 3. 자이로 분산 검사
        val gyroVariance = calculateVariance(gyroBuffer)
        if (gyroVariance > gyroVarianceThreshold) {
            return false
        }

        return true
    }

    private fun calculateVariance(buffer: List<FloatArray>): Float {
        // 각 축별 분산의 합
        val variances = (0..2).map { axis ->
            val values = buffer.map { it[axis] }
            val mean = values.average()
            values.map { (it - mean) * (it - mean) }.average()
        }
        return variances.sum().toFloat()
    }
}
```

### 3.3 스마트폰 ZUPT의 한계

| 시나리오 | ZUPT 적용 가능성 | 이유 |
|----------|-----------------|------|
| 발에 장착된 IMU | 높음 | 걸음마다 정지 구간 존재 |
| 손에 든 스마트폰 | 낮음 | 손 흔들림으로 정지 구간 희소 |
| 테이블 위 정지 | 높음 | 명확한 정지 상태 |
| 차량 내부 | 중간 | 신호 대기 시 활용 가능 |

**스마트폰에서의 대안**:
```
1. Quasi-static 감지: 완전 정지가 아닌 "거의 정지" 상태 활용
2. Motion Constraint: 특정 축 속도 제한 (예: 수직 속도 ≈ 0)
3. 시각적 정지 감지: 카메라 기반 움직임 감지와 결합
```

**참고**: [Robust Pedestrian Dead Reckoning Based on MEMS-IMU](https://pmc.ncbi.nlm.nih.gov/articles/PMC5982656/)

---

## 4. 전력 소모 및 배터리 최적화

### 4.1 센서별 전력 소모

| 구성 요소 | 전류 소모 | 비고 |
|-----------|----------|------|
| IMU (고성능 모드) | 0.5-1.0 mA | 200Hz 연속 |
| IMU (저전력 모드) | 0.03-0.1 mA | 배치/이벤트 기반 |
| 카메라 (30fps) | 200-400 mA | 가장 큰 소비원 |
| GPS (연속) | 30-50 mA | 실외용 |
| CPU (SLAM 처리) | 100-500 mA | 알고리즘 의존 |

### 4.2 배터리 최적화 전략

```kotlin
class PowerAwareIMUManager(private val context: Context) {

    enum class PowerMode {
        HIGH_PERFORMANCE,    // 200Hz, 최대 정확도
        BALANCED,            // 100Hz, 배터리 절약
        LOW_POWER,           // 50Hz + 배칭
        ULTRA_LOW_POWER      // 이벤트 기반만
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
                // 모션 감지 시에만 활성화
                enableSignificantMotionTrigger()
                setWakeLock(false)
            }
        }
    }

    // Android 센서 배칭 활용
    private fun setBatching(enabled: Boolean, batchLatencyMs: Int = 0) {
        val maxReportLatencyUs = if (enabled) batchLatencyMs * 1000 else 0

        sensorManager.registerListener(
            listener,
            accelerometer,
            samplingPeriodUs,
            maxReportLatencyUs  // 배칭 지연
        )
    }
}
```

### 4.3 적응적 전력 관리

```
┌──────────────────────────────────────────────────────────────┐
│               상황 인식 전력 관리                              │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  [정지 상태 감지] ──▶ 저전력 모드 (50Hz, 배칭)               │
│        │                                                     │
│        ▼                                                     │
│  [움직임 감지] ──▶ 고성능 모드 (200Hz)                       │
│        │                                                     │
│        ▼                                                     │
│  [시각 추적 실패] ──▶ IMU만 사용, 최대 성능                  │
│        │                                                     │
│        ▼                                                     │
│  [배터리 < 20%] ──▶ 강제 저전력 모드                         │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

**예상 배터리 수명** (3000mAh 배터리 기준):
| 모드 | 예상 사용 시간 |
|------|---------------|
| 카메라 + IMU (최대 성능) | 2-3 시간 |
| 카메라 + IMU (절전) | 4-6 시간 |
| IMU only (고성능) | 10-15 시간 |
| IMU only (저전력) | 24+ 시간 |

**참고**: [Android Battery Consumption Guidelines](https://developer.android.com/docs/quality-guidelines/build-for-billions/battery-consumption)

---

## 5. 실패 복구 및 보조 센서 융합

### 5.1 VI-SLAM 실패 유형

| 실패 유형 | 원인 | 징후 |
|-----------|------|------|
| 시각 추적 실패 | 빠른 회전, 모션 블러, 저조도 | 특징점 수 급감 |
| 스케일 드리프트 | 장시간 운용, IMU 오차 누적 | 궤적 크기 불일치 |
| Yaw 드리프트 | 자이로 바이어스 | 점진적 방향 오차 |
| 초기화 실패 | 부족한 시차, 정지 상태 | 스케일 추정 불가 |

### 5.2 Relocalization 전략

```python
class RelocalizationManager:
    def __init__(self, keyframe_database):
        self.keyframe_db = keyframe_database
        self.lost_frame_count = 0
        self.max_lost_frames = 30  # 1초 @ 30fps

    def check_tracking_status(self, current_frame):
        """
        추적 상태 확인 및 복구 시도
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
        키프레임 DB에서 유사 프레임 검색
        """
        # 1. BoW (Bag of Words) 기반 검색
        candidates = self.keyframe_db.query_similar(
            current_frame.get_bow_vector(),
            top_k=5
        )

        # 2. 각 후보에 대해 특징점 매칭
        for candidate in candidates:
            matches = feature_matcher.match(
                current_frame.descriptors,
                candidate.descriptors
            )

            if len(matches) > 50:
                # 3. PnP로 포즈 추정
                success, pose = estimate_pose_pnp(
                    current_frame.keypoints,
                    candidate.map_points,
                    matches
                )

                if success:
                    return TrackingStatus.RELOCALIZED, pose

        return TrackingStatus.RELOCALIZATION_FAILED
```

### 5.3 GPS 융합 (VINS-Fusion 방식)

```yaml
# VINS-Fusion GPS 설정
# config/gps_fusion.yaml

gnss_enable: 1
gnss_meas_freq: 10            # GPS 업데이트 빈도 (Hz)

# GPS-VIO Loosely Coupled
gnss_local_online_sync: 1     # 로컬 좌표계 동기화
gnss_mode: 2                  # 0: none, 1: position only, 2: position + heading

# GPS Dropout 대응
gnss_max_dropout_time: 5.0    # GPS 신호 없이 VIO만 사용하는 최대 시간
gnss_position_std_threshold: 5.0  # GPS 신뢰도 임계값 (m)
```

**GPS 융합 아키텍처**:
```
┌────────────────────────────────────────────────────────────────┐
│                    GPS-VIO Tightly Coupled                     │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│   Camera ───▶ 시각 특징 ───┐                                   │
│                            ├──▶ 그래프 최적화 ──▶ 최종 포즈    │
│   IMU ──────▶ Preintegration ─┤                                │
│                            │                                   │
│   GPS ──────▶ 위치 Factor ──┘                                  │
│                                                                │
│   장점:                                                        │
│   • GPS Dropout 시 VIO로 연속 추적                             │
│   • 전역 좌표계 정합                                           │
│   • 장거리 드리프트 제거                                       │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 5.4 Loop Closure 전략

```python
class LoopClosureDetector:
    """
    장거리 드리프트 보정을 위한 루프 감지
    """
    def __init__(self):
        self.min_loop_distance = 10  # 최소 10 프레임 간격
        self.similarity_threshold = 0.8

    def detect_loop(self, current_keyframe, all_keyframes):
        """
        현재 키프레임과 유사한 과거 키프레임 검색
        """
        current_idx = len(all_keyframes) - 1

        for i, past_kf in enumerate(all_keyframes[:-self.min_loop_distance]):
            # 시간 간격 확인
            if current_idx - i < self.min_loop_distance:
                continue

            # 외관 유사도
            similarity = self.compute_similarity(current_keyframe, past_kf)

            if similarity > self.similarity_threshold:
                # 기하학적 검증
                if self.geometric_verification(current_keyframe, past_kf):
                    return LoopCandidate(
                        current_idx=current_idx,
                        loop_idx=i,
                        similarity=similarity
                    )

        return None

    def apply_loop_correction(self, loop_candidate, pose_graph):
        """
        포즈 그래프 최적화로 드리프트 보정
        """
        # 루프 에지 추가
        pose_graph.add_loop_constraint(
            loop_candidate.current_idx,
            loop_candidate.loop_idx,
            relative_pose=loop_candidate.relative_pose
        )

        # 전체 그래프 최적화
        optimized_poses = pose_graph.optimize()

        return optimized_poses
```

**참고**: [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

---

## 6. 차량/진동 환경 대응

### 6.1 차량 진동 특성

```
차량 환경의 주요 진동원:
┌─────────────────────────────────────────────────────────────┐
│  주파수 범위    │  원인                    │  영향          │
├─────────────────────────────────────────────────────────────┤
│  10-30 Hz       │  엔진 아이들링           │  저주파 노이즈 │
│  30-100 Hz      │  노면 요철               │  가속도 왜곡   │
│  100-500 Hz     │  엔진 고회전, 타이어     │  고주파 노이즈 │
│  > 500 Hz       │  전자 장비               │  필터링 필요   │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 진동 필터링

```python
import numpy as np
from scipy import signal

class VibrationFilter:
    """
    차량 진동 보상 필터
    """
    def __init__(self, sample_rate=200):
        self.fs = sample_rate

        # 저역통과 필터 (엔진 진동 제거)
        self.lpf_cutoff = 20  # Hz
        self.b_lpf, self.a_lpf = signal.butter(
            4, self.lpf_cutoff / (self.fs / 2), btype='low'
        )

        # 적응형 노치 필터 (특정 주파수 제거)
        self.notch_freqs = []  # 동적으로 감지

        # 상태 변수
        self.zi_lpf = signal.lfilter_zi(self.b_lpf, self.a_lpf)

    def detect_vibration_frequencies(self, data, window_size=256):
        """
        FFT로 주요 진동 주파수 감지
        """
        freqs, psd = signal.welch(data, self.fs, nperseg=window_size)

        # 피크 검출
        peaks, _ = signal.find_peaks(psd, height=np.mean(psd) * 3)

        # 10Hz 이상 피크만 (실제 움직임 제외)
        vibration_freqs = [freqs[p] for p in peaks if freqs[p] > 10]

        return vibration_freqs

    def apply_adaptive_filter(self, accel_data):
        """
        적응형 필터링
        """
        # 1. 진동 주파수 감지
        vibration_freqs = self.detect_vibration_frequencies(accel_data)

        # 2. 노치 필터 적용 (감지된 각 주파수에 대해)
        filtered = accel_data.copy()
        for freq in vibration_freqs:
            b_notch, a_notch = signal.iirnotch(freq, Q=30, fs=self.fs)
            filtered = signal.filtfilt(b_notch, a_notch, filtered)

        # 3. 저역통과 필터
        filtered, self.zi_lpf = signal.lfilter(
            self.b_lpf, self.a_lpf, filtered, zi=self.zi_lpf * filtered[0]
        )

        return filtered
```

### 6.3 차량용 VI-SLAM 특수 고려사항

| 고려사항 | 설명 | 대응책 |
|----------|------|--------|
| 고속 이동 | 모션 블러, 특징 추적 어려움 | 짧은 노출, Global Shutter |
| 진동 | IMU 노이즈 증가 | 적응형 필터, 저역통과 |
| 급격한 조명 변화 | 터널 진입/출구 | Auto Exposure, HDR |
| GPS 음영 | 터널, 고가 하부 | VIO로 브리징 |
| 반복적 환경 | 고속도로, 주차장 | GPS 융합, 맵 기반 로컬라이제이션 |

**참고**: [Vehicle Vibration Error Compensation on IMU](https://www.researchgate.net/publication/330373155_Vehicle_Vibration_Error_Compensation_on_IMU-accelerometer_Sensor_Using_Adaptive_Filter_and_Low-pass_Filter_Approaches)

---

## 7. 하드웨어 수준 고려사항

### 7.1 스마트폰별 IMU 칩셋 특성

| 제조사 | 대표 칩셋 | 특성 |
|--------|----------|------|
| Bosch | BMI260/270 | 저전력, 스마트 기능 내장 |
| STMicroelectronics | LSM6DSO | 고정밀, 낮은 노이즈 |
| InvenSense (TDK) | ICM-42688 | 낮은 바이어스 드리프트 |
| Qualcomm | 내장 센서 허브 | SoC 통합 |

### 7.2 권장 테스트 절차

```
┌──────────────────────────────────────────────────────────────┐
│               스마트폰 IMU 품질 평가 절차                      │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  1. 정적 테스트 (15-24시간)                                  │
│     • Allan Variance 분석                                    │
│     • 노이즈 파라미터 추출                                   │
│     • 온도 안정성 확인                                       │
│                                                              │
│  2. 동적 테스트 (캘리브레이션 보드 사용)                      │
│     • 스케일 팩터 검증                                       │
│     • 축간 오정렬 측정                                       │
│     • 카메라-IMU 시간 오프셋                                 │
│                                                              │
│  3. 샘플링 품질 테스트                                       │
│     • 주파수 안정성 측정                                     │
│     • 타임스탬프 지터 분석                                   │
│     • 드롭아웃 발생 빈도                                     │
│                                                              │
│  4. 통합 테스트                                              │
│     • EuRoC 데이터셋 비교                                    │
│     • 실제 환경 궤적 정확도                                  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

---

## 8. 요약: 추가 고려사항 체크리스트

| 카테고리 | 체크 항목 | 중요도 |
|----------|----------|--------|
| **샘플링 품질** | 타임스탬프 지터 모니터링 | 높음 |
| | 샘플 드롭아웃 감지 | 높음 |
| | Android 12+ 권한 처리 | 중간 |
| **센서 융합** | 자력계 간섭 감지 | 환경 의존 |
| | 적응적 융합 가중치 | 중간 |
| **정지 감지** | ZUPT 구현 (해당 시) | 낮음-중간 |
| | Quasi-static 감지 | 중간 |
| **전력 관리** | 상황 인식 전력 모드 | 중간 |
| | 센서 배칭 활용 | 낮음 |
| **실패 복구** | Relocalization 구현 | 높음 |
| | Loop Closure | 높음 |
| | GPS 융합 (해당 시) | 환경 의존 |
| **환경 적응** | 진동 필터링 (차량) | 환경 의존 |
| | 조명 변화 대응 | 중간 |

---

## References

### 샘플링 및 타임스탬프
- [Assessing Jitter in Sensor Time Series from Android](https://ieeexplore.ieee.org/document/7501679/)
- [Mobile AR Sensor Logger](https://arxiv.org/pdf/2001.00470)

### 자력계 및 센서 융합
- [A Fusion Method for Combining Low-Cost IMU/Magnetometer](https://www.mdpi.com/1424-8220/18/8/2616)
- [Indoor Localization Methods for Smartphones](https://www.mdpi.com/1424-8220/25/6/1806)

### ZUPT 및 보행자 항법
- [Robust Pedestrian Dead Reckoning Based on MEMS-IMU](https://pmc.ncbi.nlm.nih.gov/articles/PMC5982656/)
- [Pseudo-Zero Velocity Re-Detection ZUPT](https://ieeexplore.ieee.org/document/9391710/)

### 실패 복구 및 GPS 융합
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [GPS-Visual-Inertial Odometry](https://arxiv.org/pdf/2203.02677)

### 진동 환경
- [Vehicle Vibration Error Compensation on IMU](https://www.researchgate.net/publication/330373155_Vehicle_Vibration_Error_Compensation_on_IMU-accelerometer_Sensor_Using_Adaptive_Filter_and_Low-pass_Filter_Approaches)
- [INS and Vibration](https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/APN-112-Inertial-Navigation-Systems-and-Vibration/APN-112-Inertial-Navigation-Systems-and-Vibration.pdf)

### 관련 문서
- [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [09_timestamp_synchronization.md](09_timestamp_synchronization.md)
