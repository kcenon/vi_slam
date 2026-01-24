# Camera-IMU Timestamp Synchronization Implementation

## 1. 개요

VI-SLAM에서 카메라와 IMU 데이터의 정확한 시간 동기화는 시스템 성능에 결정적인 영향을 미친다. 이 문서에서는 소프트웨어 기반 동기화 구현 방법을 다룬다.

### 동기화의 중요성

```
시간 오프셋 1ms 영향 (일반적인 모션):
- 회전 1 rad/s → 1 mrad (0.057°) 오차
- 이동 1 m/s → 1 mm 오차

빠른 동작 (10 rad/s 회전):
- 1ms 오프셋 → 10 mrad (0.57°) 오차
- VI-SLAM 성능 급격히 저하
```

## 2. 플랫폼별 타임스탬프 시스템

### 2.1 Android

```kotlin
/*
 * Android 타임스탬프 소스:
 *
 * 1. SystemClock.elapsedRealtimeNanos()
 *    - 기기 부팅 이후 경과 시간
 *    - 딥슬립 시간 포함
 *    - 단조 증가 보장
 *
 * 2. System.nanoTime()
 *    - JVM 시작 이후 경과 시간
 *    - 딥슬립 시간 미포함
 *    - 카메라/센서 타임스탬프와 불일치 가능
 *
 * 3. System.currentTimeMillis()
 *    - 벽시계 시간 (Wall clock)
 *    - 사용자/NTP에 의해 변경 가능
 *    - 동기화 용도 부적합
 *
 * Camera2 & SensorManager:
 * - SENSOR_TIMESTAMP_SOURCE_REALTIME인 경우
 *   elapsedRealtimeNanos()와 동일 기준 사용
 */

object TimestampUtils {

    // 현재 시스템 시간 (나노초)
    fun currentNanos(): Long = SystemClock.elapsedRealtimeNanos()

    // 나노초 → 초
    fun nanosToSeconds(nanos: Long): Double = nanos / 1_000_000_000.0

    // 초 → 나노초
    fun secondsToNanos(seconds: Double): Long = (seconds * 1_000_000_000).toLong()

    // 타임스탬프 유효성 검증
    fun isValidTimestamp(timestamp: Long): Boolean {
        val current = currentNanos()
        // 미래 시간이거나 1시간 이상 과거면 무효
        return timestamp <= current && (current - timestamp) < 3_600_000_000_000L
    }
}
```

### 2.2 iOS

```swift
/*
 * iOS 타임스탬프 소스:
 *
 * 1. mach_absolute_time()
 *    - 기기 부팅 이후 틱 수
 *    - mach_timebase_info로 나노초 변환 필요
 *
 * 2. CACurrentMediaTime()
 *    - mach_absolute_time 기반, 초 단위 반환
 *    - CoreAnimation 타이밍 함수
 *
 * 3. Date() / CFAbsoluteTimeGetCurrent()
 *    - 벽시계 시간
 *    - 동기화 용도 부적합
 *
 * AVFoundation (Camera):
 * - CMSampleBufferGetPresentationTimeStamp
 * - hostTime 기준 (mach_absolute_time 변환)
 *
 * CoreMotion (IMU):
 * - timestamp 속성
 * - 시스템 부팅 이후 초 단위
 * - mach_absolute_time 기반
 */

struct TimestampUtils {

    // mach_absolute_time → 나노초
    static func machToNanos(_ machTime: UInt64) -> Int64 {
        var timebaseInfo = mach_timebase_info_data_t()
        mach_timebase_info(&timebaseInfo)

        let nanos = machTime * UInt64(timebaseInfo.numer) / UInt64(timebaseInfo.denom)
        return Int64(nanos)
    }

    // 현재 시간 (나노초)
    static func currentNanos() -> Int64 {
        return machToNanos(mach_absolute_time())
    }

    // CoreMotion TimeInterval → 나노초
    static func timeIntervalToNanos(_ interval: TimeInterval) -> Int64 {
        return Int64(interval * 1_000_000_000)
    }
}
```

## 3. 소프트웨어 동기화 전략

### 3.1 공통 시간 기준 사용

```kotlin
// Android: Camera2와 SensorManager가 같은 시간 기준 사용 확인

class SynchronizationChecker(private val context: Context) {

    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    data class SyncCapability(
        val cameraTimestampSource: String,
        val canHardwareSync: Boolean,
        val estimatedOffset: Long?  // 나노초
    )

    fun checkSyncCapability(cameraId: String): SyncCapability {
        val characteristics = cameraManager.getCameraCharacteristics(cameraId)

        val timestampSource = characteristics.get(
            CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
        )

        val canHardwareSync = timestampSource ==
            CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME

        return SyncCapability(
            cameraTimestampSource = when (timestampSource) {
                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME -> "REALTIME"
                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN -> "UNKNOWN"
                else -> "OTHER"
            },
            canHardwareSync = canHardwareSync,
            estimatedOffset = if (canHardwareSync) 0L else null
        )
    }
}
```

### 3.2 시간 오프셋 추정 (Cross-Correlation)

```kotlin
/**
 * 각속도 기반 시간 오프셋 추정
 *
 * 원리:
 * 1. 카메라 프레임에서 광류(optical flow)로 각속도 추정
 * 2. IMU 자이로스코프 각속도와 비교
 * 3. 상호상관(cross-correlation)으로 최적 오프셋 찾기
 */

class TimeOffsetEstimator {

    data class AngularVelocitySample(
        val timestamp: Long,  // 나노초
        val omega: Double     // rad/s (magnitude)
    )

    /**
     * 상호상관을 통한 시간 오프셋 추정
     *
     * @param cameraSamples 카메라에서 추정한 각속도
     * @param imuSamples IMU 자이로스코프 각속도
     * @param searchRangeMs 검색 범위 (밀리초)
     * @return 추정된 오프셋 (나노초, 양수면 IMU가 카메라보다 앞섬)
     */
    fun estimateOffset(
        cameraSamples: List<AngularVelocitySample>,
        imuSamples: List<AngularVelocitySample>,
        searchRangeMs: Long = 100
    ): Long {
        val searchRangeNs = searchRangeMs * 1_000_000L
        val stepNs = 1_000_000L  // 1ms 단위

        var bestOffset = 0L
        var bestCorrelation = Double.NEGATIVE_INFINITY

        var offset = -searchRangeNs
        while (offset <= searchRangeNs) {
            val correlation = computeCorrelation(cameraSamples, imuSamples, offset)

            if (correlation > bestCorrelation) {
                bestCorrelation = correlation
                bestOffset = offset
            }

            offset += stepNs
        }

        // Sub-millisecond refinement using quadratic fitting
        return refineOffset(cameraSamples, imuSamples, bestOffset, stepNs)
    }

    private fun computeCorrelation(
        cameraSamples: List<AngularVelocitySample>,
        imuSamples: List<AngularVelocitySample>,
        offset: Long
    ): Double {
        // 카메라 샘플에 오프셋 적용 후 IMU와 매칭
        var sumProduct = 0.0
        var count = 0

        for (camSample in cameraSamples) {
            val adjustedTime = camSample.timestamp + offset

            // 가장 가까운 IMU 샘플 찾기
            val imuSample = findClosestSample(imuSamples, adjustedTime) ?: continue

            sumProduct += camSample.omega * imuSample.omega
            count++
        }

        return if (count > 0) sumProduct / count else 0.0
    }

    private fun findClosestSample(
        samples: List<AngularVelocitySample>,
        targetTime: Long
    ): AngularVelocitySample? {
        return samples.minByOrNull { abs(it.timestamp - targetTime) }
    }

    private fun refineOffset(
        cameraSamples: List<AngularVelocitySample>,
        imuSamples: List<AngularVelocitySample>,
        coarseOffset: Long,
        step: Long
    ): Long {
        // Quadratic fitting for sub-step accuracy
        val c0 = computeCorrelation(cameraSamples, imuSamples, coarseOffset - step)
        val c1 = computeCorrelation(cameraSamples, imuSamples, coarseOffset)
        val c2 = computeCorrelation(cameraSamples, imuSamples, coarseOffset + step)

        // Parabola vertex: offset_refined = coarse + step * (c0 - c2) / (2 * (c0 - 2*c1 + c2))
        val denominator = 2 * (c0 - 2 * c1 + c2)
        if (abs(denominator) < 1e-10) return coarseOffset

        val refinement = step * (c0 - c2) / denominator
        return coarseOffset + refinement.toLong()
    }
}
```

### 3.3 실시간 오프셋 보정

```kotlin
/**
 * Kalman Filter 기반 실시간 시간 오프셋 추정
 */
class OnlineTimeOffsetEstimator {

    // 상태: [offset, offset_drift]
    private var state = doubleArrayOf(0.0, 0.0)

    // 공분산 행렬
    private var P = arrayOf(
        doubleArrayOf(1e-3, 0.0),
        doubleArrayOf(0.0, 1e-6)
    )

    // 프로세스 노이즈
    private val Q = arrayOf(
        doubleArrayOf(1e-8, 0.0),
        doubleArrayOf(0.0, 1e-10)
    )

    // 측정 노이즈
    private var R = 1e-4

    /**
     * 새 측정값으로 오프셋 추정 업데이트
     *
     * @param measuredOffset 단일 측정에서 추정된 오프셋 (초)
     * @param dt 이전 업데이트 이후 시간 (초)
     */
    fun update(measuredOffset: Double, dt: Double) {
        // Prediction step
        val F = arrayOf(
            doubleArrayOf(1.0, dt),
            doubleArrayOf(0.0, 1.0)
        )

        val predictedState = matVecMul(F, state)
        val predictedP = matAdd(matMul(matMul(F, P), transpose(F)), Q)

        // Update step
        val H = doubleArrayOf(1.0, 0.0)
        val y = measuredOffset - (H[0] * predictedState[0] + H[1] * predictedState[1])

        val S = H[0] * predictedP[0][0] + H[1] * predictedP[1][0] +
                H[0] * predictedP[0][1] + H[1] * predictedP[1][1] + R

        val K = doubleArrayOf(
            (predictedP[0][0] * H[0] + predictedP[0][1] * H[1]) / S,
            (predictedP[1][0] * H[0] + predictedP[1][1] * H[1]) / S
        )

        state[0] = predictedState[0] + K[0] * y
        state[1] = predictedState[1] + K[1] * y

        // Update covariance
        P[0][0] = (1 - K[0] * H[0]) * predictedP[0][0]
        P[0][1] = (1 - K[0] * H[0]) * predictedP[0][1]
        P[1][0] = -K[1] * H[0] * predictedP[0][0] + predictedP[1][0]
        P[1][1] = -K[1] * H[0] * predictedP[0][1] + predictedP[1][1]
    }

    fun getEstimatedOffset(): Double = state[0]  // 초 단위

    fun getEstimatedDrift(): Double = state[1]   // 초/초

    fun getOffsetUncertainty(): Double = sqrt(P[0][0])

    // 행렬 연산 헬퍼 함수들 (생략)
    private fun matVecMul(m: Array<DoubleArray>, v: DoubleArray): DoubleArray = TODO()
    private fun matMul(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> = TODO()
    private fun matAdd(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> = TODO()
    private fun transpose(m: Array<DoubleArray>): Array<DoubleArray> = TODO()
}
```

## 4. IMU 데이터 보간

### 4.1 선형 보간

```kotlin
/**
 * 카메라 프레임 시간에 맞춘 IMU 데이터 보간
 */
class ImuInterpolator {

    data class ImuSample(
        val timestamp: Long,    // 나노초
        val accel: DoubleArray, // [ax, ay, az] m/s²
        val gyro: DoubleArray   // [gx, gy, gz] rad/s
    )

    /**
     * 선형 보간으로 특정 시간의 IMU 값 추정
     */
    fun interpolate(
        imuSamples: List<ImuSample>,
        targetTime: Long
    ): ImuSample? {
        // targetTime을 포함하는 두 샘플 찾기
        val sortedSamples = imuSamples.sortedBy { it.timestamp }

        val beforeIdx = sortedSamples.indexOfLast { it.timestamp <= targetTime }
        if (beforeIdx < 0 || beforeIdx >= sortedSamples.size - 1) {
            return null  // 범위 밖
        }

        val before = sortedSamples[beforeIdx]
        val after = sortedSamples[beforeIdx + 1]

        // 보간 비율 계산
        val t = (targetTime - before.timestamp).toDouble() /
                (after.timestamp - before.timestamp).toDouble()

        // 선형 보간
        return ImuSample(
            timestamp = targetTime,
            accel = interpolateArray(before.accel, after.accel, t),
            gyro = interpolateArray(before.gyro, after.gyro, t)
        )
    }

    /**
     * 카메라 프레임 사이의 모든 IMU 샘플 + 시작/끝 시간 보간
     */
    fun getIntegratedImu(
        imuSamples: List<ImuSample>,
        startTime: Long,
        endTime: Long
    ): List<ImuSample> {
        val result = mutableListOf<ImuSample>()

        // 시작 시간 보간 샘플
        interpolate(imuSamples, startTime)?.let { result.add(it) }

        // 시작~끝 사이의 원본 샘플들
        imuSamples.filter { it.timestamp in (startTime + 1) until endTime }
            .sortedBy { it.timestamp }
            .forEach { result.add(it) }

        // 끝 시간 보간 샘플
        interpolate(imuSamples, endTime)?.let { result.add(it) }

        return result
    }

    private fun interpolateArray(
        a: DoubleArray,
        b: DoubleArray,
        t: Double
    ): DoubleArray {
        return DoubleArray(a.size) { i ->
            a[i] * (1 - t) + b[i] * t
        }
    }
}
```

### 4.2 SLERP (구면 선형 보간)

```kotlin
/**
 * 자이로스코프 데이터의 경우 회전 보간에 SLERP 사용 권장
 */
class QuaternionInterpolator {

    data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {

        fun normalize(): Quaternion {
            val norm = sqrt(w * w + x * x + y * y + z * z)
            return Quaternion(w / norm, x / norm, y / norm, z / norm)
        }

        fun dot(other: Quaternion): Double {
            return w * other.w + x * other.x + y * other.y + z * other.z
        }

        operator fun times(scalar: Double): Quaternion {
            return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar)
        }

        operator fun plus(other: Quaternion): Quaternion {
            return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z)
        }
    }

    /**
     * SLERP (Spherical Linear Interpolation)
     */
    fun slerp(q0: Quaternion, q1: Quaternion, t: Double): Quaternion {
        var dot = q0.dot(q1)

        // 최단 경로 보장
        val q1Adjusted = if (dot < 0) {
            dot = -dot
            Quaternion(-q1.w, -q1.x, -q1.y, -q1.z)
        } else {
            q1
        }

        // 매우 가까운 경우 선형 보간
        if (dot > 0.9995) {
            return (q0 * (1 - t) + q1Adjusted * t).normalize()
        }

        val theta0 = acos(dot)
        val theta = theta0 * t
        val sinTheta = sin(theta)
        val sinTheta0 = sin(theta0)

        val s0 = cos(theta) - dot * sinTheta / sinTheta0
        val s1 = sinTheta / sinTheta0

        return (q0 * s0 + q1Adjusted * s1).normalize()
    }
}
```

## 5. 동기화 품질 검증

### 5.1 동기화 메트릭

```kotlin
class SynchronizationValidator {

    data class SyncMetrics(
        val meanOffset: Double,      // 평균 오프셋 (초)
        val stdOffset: Double,       // 오프셋 표준편차
        val maxJitter: Double,       // 최대 지터
        val missingFrames: Int,      // 누락 프레임 수
        val outOfOrderSamples: Int   // 순서 오류 샘플 수
    )

    fun validateSynchronization(
        frameTimestamps: List<Long>,
        imuTimestamps: List<Long>,
        expectedFrameRate: Double,
        expectedImuRate: Double
    ): SyncMetrics {
        // 1. 프레임 간격 분석
        val frameIntervals = frameTimestamps.zipWithNext { a, b -> b - a }
        val expectedFrameInterval = (1_000_000_000.0 / expectedFrameRate).toLong()

        val frameMissing = frameIntervals.count {
            it > expectedFrameInterval * 1.5
        }

        // 2. IMU 간격 분석
        val imuIntervals = imuTimestamps.zipWithNext { a, b -> b - a }
        val outOfOrder = imuTimestamps.zipWithNext().count { (a, b) -> b <= a }

        // 3. 지터 계산
        val expectedImuInterval = 1_000_000_000.0 / expectedImuRate
        val jitters = imuIntervals.map { abs(it - expectedImuInterval) }

        return SyncMetrics(
            meanOffset = 0.0,  // 별도 계산 필요
            stdOffset = 0.0,
            maxJitter = jitters.maxOrNull()?.let { it / 1_000_000.0 } ?: 0.0,  // ms
            missingFrames = frameMissing,
            outOfOrderSamples = outOfOrder
        )
    }

    fun checkContinuity(timestamps: List<Long>, maxGapNs: Long): List<Long> {
        // 연속성이 깨진 위치의 타임스탬프 반환
        return timestamps.zipWithNext()
            .filter { (a, b) -> b - a > maxGapNs }
            .map { it.first }
    }
}
```

### 5.2 시각적 검증

```kotlin
/**
 * 디버깅용: 카메라-IMU 동기화 시각화 데이터 생성
 */
class SyncVisualizer {

    data class VisualizationData(
        val timeSeconds: List<Double>,
        val cameraAngularVel: List<Double>,
        val imuAngularVel: List<Double>,
        val correlationByOffset: Map<Double, Double>
    )

    fun generateVisualizationData(
        cameraData: List<Pair<Long, Double>>,  // (timestamp, omega)
        imuData: List<Pair<Long, Double>>,
        testOffsets: List<Long>
    ): VisualizationData {
        val baseTime = minOf(
            cameraData.minOfOrNull { it.first } ?: 0,
            imuData.minOfOrNull { it.first } ?: 0
        )

        val timeSeconds = cameraData.map { (it.first - baseTime) / 1e9 }
        val cameraVel = cameraData.map { it.second }
        val imuVel = imuData.map { it.second }

        val correlations = testOffsets.associate { offset ->
            val correlation = computeCorrelation(cameraData, imuData, offset)
            offset / 1e6 to correlation  // ms 단위로 변환
        }

        return VisualizationData(
            timeSeconds = timeSeconds,
            cameraAngularVel = cameraVel,
            imuAngularVel = imuVel,
            correlationByOffset = correlations
        )
    }

    private fun computeCorrelation(
        cameraData: List<Pair<Long, Double>>,
        imuData: List<Pair<Long, Double>>,
        offset: Long
    ): Double {
        // 상호상관 계산 (간소화)
        return 0.0  // 실제 구현 필요
    }
}
```

## 6. 구현 체크리스트

### 데이터 수집 단계

- [ ] 카메라 타임스탬프 소스 확인 (REALTIME 여부)
- [ ] IMU 샘플링 주파수 확인 (200Hz 이상 권장)
- [ ] 타임스탬프 단위 통일 (나노초 권장)
- [ ] 타임스탬프 단조 증가 확인

### 동기화 처리 단계

- [ ] 초기 시간 오프셋 추정 (cross-correlation)
- [ ] 온라인 오프셋 추적 (Kalman Filter)
- [ ] IMU 데이터 보간 (프레임 시간에 맞춤)

### 검증 단계

- [ ] 프레임 드롭 확인
- [ ] IMU 데이터 연속성 확인
- [ ] 시간 오프셋 안정성 확인
- [ ] SLAM 궤적 품질 검증

## References

- [Online Temporal Calibration for Camera-IMU Systems](https://arxiv.org/pdf/1808.00692)
- [Kalibr Camera-IMU Calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [Sensor Synchronization for Android Phone VI-SLAM](https://www.researchgate.net/publication/324929837_Sensor_Synchronization_for_Android_Phone_Tightly-Coupled_Visual-Inertial_SLAM)
- [MARS Logger Paper](https://arxiv.org/pdf/2001.00470)
