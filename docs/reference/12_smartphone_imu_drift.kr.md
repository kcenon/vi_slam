# Smartphone IMU Drift: Causes, Characteristics, and Mitigation

## 1. 개요

스마트폰에 탑재된 MEMS(Micro-Electro-Mechanical Systems) IMU는 저비용으로 대량 생산되어 성능 한계가 있다. 특히 드리프트(drift) 현상은 VI-SLAM 시스템의 정확도에 직접적인 영향을 미친다.

### 왜 스마트폰 IMU가 문제인가?

| 특성 | 고급 IMU (Tactical Grade) | 스마트폰 MEMS |
|------|--------------------------|---------------|
| 가격 | $1,000 - $10,000+ | $1 - $5 |
| 바이어스 안정성 | 0.01 - 1 °/h | 10 - 1000 °/h |
| 노이즈 밀도 (Gyro) | 0.001 - 0.01 °/s/√Hz | 0.005 - 0.05 °/s/√Hz |
| 60초 후 위치 오차 | < 1m | > 150m |

**핵심 문제**: 간단한 MEMS IMU 기반 INS에서 60초 작동 후 평균 위치 오차가 150m 이상 발생할 수 있다.

---

## 2. IMU 오차 유형 및 원인

### 2.1 결정론적 오차 (Deterministic Errors)

캘리브레이션으로 제거 가능한 체계적 오차.

#### 바이어스 (Bias)
```
측정값 = 실제값 + 바이어스

가속도계: a_measured = a_true + b_a
자이로스코프: ω_measured = ω_true + b_g
```

**영향**:
- 가속도계 바이어스 1mg → 1초 후 0.5mm 위치 오차, 100초 후 **5m 오차**
- 자이로 바이어스 1°/h → 1시간 후 1° 자세 오차

#### 스케일 팩터 오차 (Scale Factor Error)
```
측정값 = (1 + δS) × 실제값

δS: 스케일 팩터 오차 (일반적으로 0.1% ~ 1%)
```

**스마트폰 MEMS 특성**:
- 미캘리브레이션 시 스케일 팩터 오차: 최대 10% (0.1)
- 캘리브레이션 후: 0.1% ~ 1%

#### 축간 오정렬 (Misalignment / Cross-coupling)
```
    ┌ 1      δ_xy   δ_xz ┐
M = │ δ_yx   1      δ_yz │
    └ δ_zx   δ_zy   1    ┘

δ: 오정렬 각도 (일반적으로 0.01 ~ 0.02 rad)
```

### 2.2 확률적 오차 (Stochastic Errors)

캘리브레이션으로 제거 불가능한 무작위 오차.

#### White Noise (백색 잡음)
```
고주파 열전기적 반응에 의한 노이즈

가속도계: Velocity Random Walk (VRW) - m/s/√Hz
자이로스코프: Angle Random Walk (ARW) - rad/s/√Hz
```

**스마트폰 MEMS 일반값**:
```yaml
# 저가형 스마트폰 IMU
accelerometer_noise_density: 0.01     # m/s²/√Hz
gyroscope_noise_density: 0.005        # rad/s/√Hz (약 0.3 °/s/√Hz)
```

#### Bias Instability (바이어스 불안정성)
```
센서 바이어스가 시간에 따라 천천히 변동하는 현상
= "In-run bias stability"

일정 온도에서도 발생하는 물리적 한계
```

**바이어스 불안정성 영향**:
- 가장 중요한 IMU 스펙
- 장시간 운용 시 누적 오차의 주 원인

#### Bias Random Walk (바이어스 랜덤 워크)
```
바이어스가 시간에 따라 랜덤하게 워킹

가속도계: σ_ba (m/s³/√Hz)
자이로스코프: σ_bg (rad/s²/√Hz)
```

**스마트폰 MEMS 일반값**:
```yaml
accelerometer_random_walk: 0.0002    # m/s³/√Hz
gyroscope_random_walk: 4.0e-06       # rad/s²/√Hz
```

### 2.3 환경 의존적 오차

#### 온도 영향
```
바이어스 = b_0 + k_T × ΔT

k_T: 온도 계수 (°/h/°C 또는 mg/°C)
ΔT: 온도 변화
```

**스마트폰 특성**:
- 사용 중 온도 상승 (프로세서 발열)
- 온도 변화에 따른 바이어스 드리프트 발생

#### 가속도 민감도 (g-sensitivity)
```
MEMS 자이로의 가속도 영향

오차 = 가속도 민감도 × 가속도 × 시간

예: 20g, 10초, 0.05 (°/s)/g → 약 10° 각도 오차
```

---

## 3. Allan Variance 분석

IMU 노이즈 특성을 정량화하는 표준 방법.

### 3.1 Allan Variance 개요

```
              ┌────────────────────────────────────────────┐
              │         Allan Deviation Plot               │
log(σ(τ))     │                                            │
    │         │     ARW                                    │
    │         │    slope = -1/2                           │
    │         │        \                                   │
    │         │         \     Bias Instability            │
    │         │          \____  (minimum, slope ≈ 0)      │
    │         │               \_____                       │
    │         │                     \___  RRW              │
    │         │                         \ slope = +1/2    │
    └─────────┴────────────────────────────────────────────┘
                              log(τ)
```

### 3.2 주요 파라미터 추출

| 파라미터 | Allan Plot 특성 | 의미 |
|----------|----------------|------|
| ARW (N) | slope = -1/2, τ=1에서 값 | 백색 잡음 (고주파) |
| Bias Instability (B) | minimum 지점 (slope ≈ 0) | 장기 안정성 |
| RRW (K) | slope = +1/2 | 바이어스 랜덤 워크 |

### 3.3 데이터 수집 및 분석

```python
# Allan Variance 분석을 위한 데이터 수집
# 권장: 15-24시간 정지 상태 기록

import numpy as np
from allantools import adev

def compute_allan_variance(gyro_data, sample_rate):
    """
    Args:
        gyro_data: 자이로스코프 데이터 (rad/s)
        sample_rate: 샘플링 주파수 (Hz)

    Returns:
        tau: 클러스터 시간 배열
        adev: Allan deviation 배열
    """
    # Allan deviation 계산
    tau, adev_values, _, _ = adev(
        gyro_data,
        rate=sample_rate,
        data_type="freq"
    )

    return tau, adev_values

def extract_noise_params(tau, adev):
    """
    Allan deviation plot에서 노이즈 파라미터 추출
    """
    # ARW: τ=1에서 slope=-1/2 라인의 값
    arw_idx = np.argmin(np.abs(tau - 1.0))
    arw = adev[arw_idx]

    # Bias Instability: 최소값
    bias_instability = np.min(adev)

    return {
        'arw': arw,          # rad/s/√Hz
        'bias_instability': bias_instability  # rad/s
    }
```

### 3.4 스마트폰별 측정 결과 (연구 데이터)

| 스마트폰 | Gyro 노이즈 SD (rad/s) | Accel 노이즈 SD (m/s²) |
|----------|----------------------|------------------------|
| Google Pixel 7 Pro | 0.0018 | 0.0089 |
| iPhone XR | **0.0027** (최대) | 0.0095 |
| Samsung A53 | 0.0021 | **0.0106** (최대) |
| OnePlus 7 Pro | 0.0023 | 0.0098 |

**출처**: [Smartphone MEMS Accelerometer and Gyroscope Measurement Errors](https://www.mdpi.com/1424-8220/23/17/7609)

---

## 4. 드리프트 보정 기법

### 4.1 필터 기반 방법

#### Extended Kalman Filter (EKF)

```
상태 벡터에 바이어스 포함:
x = [position, velocity, orientation, gyro_bias, accel_bias]^T

예측 단계:
x_pred = f(x, u)  // 상태 전이
P_pred = F·P·F^T + Q  // 공분산 예측

업데이트 단계 (시각 측정이 있을 때):
K = P_pred·H^T·(H·P_pred·H^T + R)^(-1)
x = x_pred + K·(z - h(x_pred))
P = (I - K·H)·P_pred
```

**Python 구현 예시**:
```python
import numpy as np

class IMUBiasEKF:
    def __init__(self):
        # 상태: [roll, pitch, yaw, bg_x, bg_y, bg_z]
        self.x = np.zeros(6)

        # 공분산 행렬
        self.P = np.eye(6) * 0.1

        # 프로세스 노이즈
        self.Q = np.diag([
            1e-4, 1e-4, 1e-4,    # 자세 노이즈
            1e-6, 1e-6, 1e-6     # 바이어스 랜덤 워크
        ])

        # 측정 노이즈 (시각 측정)
        self.R = np.eye(3) * 0.01

    def predict(self, gyro, dt):
        """
        IMU 예측 단계
        """
        # 바이어스 보정된 각속도
        omega = gyro - self.x[3:6]

        # 자세 업데이트
        self.x[0:3] += omega * dt

        # 상태 전이 행렬
        F = np.eye(6)
        F[0:3, 3:6] = -np.eye(3) * dt

        # 공분산 예측
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, visual_orientation):
        """
        시각 측정으로 업데이트
        """
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)

        # 혁신 (innovation)
        y = visual_orientation - self.x[0:3]

        # 칼만 이득
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # 상태 업데이트
        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

        return self.x[3:6]  # 추정된 바이어스 반환
```

#### Error State Kalman Filter (ESKF)

VI-SLAM에서 더 일반적으로 사용되는 방법.

```
진짜 상태 = 명목 상태 ⊕ 오차 상태

x_true = x_nominal ⊕ δx

장점:
- 작은 오차에 대해 선형화 정확도 향상
- 특이점 회피 용이
```

### 4.2 VI-SLAM 프레임워크의 바이어스 처리

#### VINS-Mono / VINS-Fusion

```cpp
// IMU Preintegration with Bias
class IMUPreintegration {
    // 바이어스 변화에 따른 선형화
    // Jacobian: ∂preintegration/∂bias

    Matrix3d J_bg;  // gyro bias jacobian
    Matrix3d J_ba;  // accel bias jacobian

    void repropagate(const Vector3d& new_bg,
                     const Vector3d& new_ba) {
        // 바이어스 변화 시 first-order 근사로 빠른 재계산
        delta_p += J_ba * (new_ba - linearized_ba);
        delta_v += J_ba * (new_ba - linearized_ba);
        delta_q = delta_q * Quaternion(J_bg * (new_bg - linearized_bg));
    }
};
```

**VINS-Mono 바이어스 추정 특성**:
- 온라인 자동 바이어스 추정
- 시간 오프셋 동시 추정 (`estimate_td: 1`)
- 비동기 센서 처리 지원

#### OpenVINS

```
MSCKF (Multi-State Constraint Kalman Filter) 기반

상태 벡터:
x = [q, p, v, bg, ba, ..., camera_states]

특징:
- on-manifold 슬라이딩 윈도우 칼만 필터
- 온라인 카메라 내부/외부 캘리브레이션
- 카메라-IMU 시간 오프셋 캘리브레이션
```

**참고**: [OpenVINS Documentation](https://docs.openvins.com/)

### 4.3 딥러닝 기반 방법

#### IMU 디노이징 네트워크

```
입력: Raw IMU 시퀀스 [N × 6] (가속도 + 자이로)
출력: 디노이즈된 IMU 또는 보정된 바이어스

아키텍처:
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│ Raw IMU     │───▶│ LSTM / TCN   │───▶│ Denoised    │
│ Sequence    │    │ Network      │    │ IMU         │
└─────────────┘    └──────────────┘    └─────────────┘
```

**주요 연구**:
- **CNN 기반**: 시간분할 방법으로 IMU 출력을 CNN 입력 형식에 맞춤
- **LSTM/GRU 하이브리드**: 최대 72% 자세 오차 감소
- **NGC-Net**: Temporal Convolutional Network로 자이로 캘리브레이션

```python
import torch
import torch.nn as nn

class IMUDenoiser(nn.Module):
    """
    LSTM 기반 IMU 디노이징 네트워크
    """
    def __init__(self, input_size=6, hidden_size=128, num_layers=2):
        super().__init__()

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            bidirectional=True
        )

        self.fc = nn.Sequential(
            nn.Linear(hidden_size * 2, 64),
            nn.ReLU(),
            nn.Linear(64, 6)  # 디노이즈된 IMU
        )

    def forward(self, x):
        # x: [batch, seq_len, 6]
        lstm_out, _ = self.lstm(x)
        # lstm_out: [batch, seq_len, hidden*2]

        output = self.fc(lstm_out)
        return output

# 학습 데이터: Ground Truth IMU 또는 시각-관성 융합 결과
```

#### 바이어스 예측 네트워크 (IPNet)

```
VINS-Mono/OpenVINS에 플러그인 가능

Raw IMU → IPNet → 평균 바이어스 추정
                      ↓
              전통적 VIO의 초기값으로 사용

장점:
- 전통적 재귀 예측 의존성 제거
- 초기 바이어스 오차 영향 감소
```

**참고**: [A Plug-and-Play Learning-based IMU Bias Factor](https://arxiv.org/html/2503.12527v1)

---

## 5. 실용적 대응 전략

### 5.1 캘리브레이션 파이프라인

```
┌──────────────────────────────────────────────────────────────┐
│               IMU 캘리브레이션 워크플로우                      │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  1. 정지 상태 바이어스 측정 (15-24시간)                        │
│     └─▶ Allan Variance 분석                                  │
│     └─▶ 노이즈 파라미터 추출 (ARW, Bias Instability, RRW)     │
│                                                              │
│  2. 동적 캘리브레이션 (Kalibr)                                │
│     └─▶ 카메라-IMU 외부 파라미터                              │
│     └─▶ 시간 오프셋                                          │
│                                                              │
│  3. 온라인 바이어스 추정                                      │
│     └─▶ VINS/OpenVINS의 실시간 바이어스 업데이트             │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### 5.2 Android에서 비보정 센서 사용

```kotlin
// VI-SLAM에는 비보정 센서 권장
// Android가 자동 적용하는 보정이 VI-SLAM에 부적합할 수 있음

val gyroscopeUncalibrated = sensorManager.getDefaultSensor(
    Sensor.TYPE_GYROSCOPE_UNCALIBRATED
)

val accelerometerUncalibrated = sensorManager.getDefaultSensor(
    Sensor.TYPE_ACCELEROMETER_UNCALIBRATED
)

// 비보정 센서 데이터 구조
// values[0-2]: 측정값
// values[3-5]: Android 추정 바이어스 (참고용)
```

**왜 비보정 센서를 사용하는가?**
- Android의 자동 캘리브레이션이 VI-SLAM의 바이어스 추정과 충돌
- 일관된 바이어스 모델 적용 가능
- VI-SLAM 프레임워크의 온라인 캘리브레이션 활용

### 5.3 권장 VI-SLAM 프레임워크 설정

#### VINS-Mono 설정

```yaml
# config/smartphone.yaml

# IMU 파라미터 (Allan Variance 결과 반영)
acc_n: 0.1          # 가속도계 노이즈 (m/s²)
gyr_n: 0.01         # 자이로스코프 노이즈 (rad/s)
acc_w: 0.002        # 가속도계 랜덤 워크
gyr_w: 0.0002       # 자이로스코프 랜덤 워크

# 온라인 캘리브레이션 활성화
estimate_extrinsic: 2   # 외부 파라미터 온라인 추정
estimate_td: 1          # 시간 오프셋 온라인 추정
td: 0.02                # 초기 시간 오프셋 추정값 (초)
```

#### OpenVINS 설정

```yaml
# config/smartphone/estimator_config.yaml

# IMU 노이즈 파라미터
gyroscope_noise_density: 0.005      # rad/s/√Hz
accelerometer_noise_density: 0.01   # m/s²/√Hz
gyroscope_random_walk: 4.0e-06      # rad/s²/√Hz
accelerometer_random_walk: 0.0002   # m/s³/√Hz

# 캘리브레이션 옵션
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true

# 바이어스 초기화
init_window_time: 1.0       # 초기화 윈도우 (초)
init_imu_thresh: 1.0        # IMU 움직임 임계값
```

### 5.4 드리프트 감지 및 복구

```kotlin
class DriftDetector(
    private val maxBiasGyro: Float = 0.1f,      // rad/s
    private val maxBiasAccel: Float = 1.0f,     // m/s²
    private val maxVelocityStatic: Float = 0.1f // m/s
) {
    private var lastValidState: SLAMState? = null

    fun checkDrift(currentState: SLAMState): DriftStatus {
        // 1. 바이어스 이상 감지
        if (currentState.gyroBias.norm() > maxBiasGyro ||
            currentState.accelBias.norm() > maxBiasAccel) {
            return DriftStatus.BIAS_ANOMALY
        }

        // 2. 정지 상태에서 속도 드리프트 감지
        if (currentState.isStationary &&
            currentState.velocity.norm() > maxVelocityStatic) {
            return DriftStatus.VELOCITY_DRIFT
        }

        // 3. 스케일 드리프트 감지 (시각적 특징점 기반)
        if (detectScaleDrift(currentState)) {
            return DriftStatus.SCALE_DRIFT
        }

        lastValidState = currentState
        return DriftStatus.NORMAL
    }

    fun recoverFromDrift(status: DriftStatus): RecoveryAction {
        return when (status) {
            DriftStatus.BIAS_ANOMALY -> RecoveryAction.REINITIALIZE_BIAS
            DriftStatus.VELOCITY_DRIFT -> RecoveryAction.ZERO_VELOCITY_UPDATE
            DriftStatus.SCALE_DRIFT -> RecoveryAction.VISUAL_RELOCALIZATION
            DriftStatus.NORMAL -> RecoveryAction.NONE
        }
    }
}

enum class DriftStatus {
    NORMAL, BIAS_ANOMALY, VELOCITY_DRIFT, SCALE_DRIFT
}

enum class RecoveryAction {
    NONE, REINITIALIZE_BIAS, ZERO_VELOCITY_UPDATE, VISUAL_RELOCALIZATION
}
```

---

## 6. 성능 기대치

### 6.1 순수 INS vs VI-SLAM

| 시간 | 순수 MEMS INS 오차 | VI-SLAM 오차 |
|------|------------------|--------------|
| 10초 | ~5m | < 0.1m |
| 60초 | **> 150m** | < 0.5m |
| 10분 | 수 km | < 2m |

### 6.2 스마트폰별 예상 성능

| 스마트폰 등급 | VI-SLAM 정확도 | 주요 제한 |
|-------------|---------------|----------|
| 플래그십 (최신) | 0.5 - 2% 이동거리 | Rolling Shutter, OIS |
| 중급 | 1 - 5% 이동거리 | 높은 IMU 노이즈 |
| 저가 | 3 - 10% 이동거리 | 불안정한 샘플링, 높은 드리프트 |

**참고**: 정확도는 환경, 움직임 패턴, 조명 조건에 따라 크게 달라질 수 있음.

---

## 7. 핵심 과제 및 해결책 요약

| 과제 | 영향 | 해결책 |
|------|------|--------|
| 자이로 바이어스 드리프트 | 자세 오차 누적 | 시각 측정으로 바이어스 관측/보정 |
| 가속도계 바이어스 | 위치 오차 급증 | EKF/최적화로 온라인 추정 |
| 온도 변화 | 바이어스 변동 | 온도 보상 모델, 자주 재캘리브레이션 |
| 고주파 노이즈 | 단기 정확도 저하 | 저역통과 필터, 딥러닝 디노이징 |
| 가속도 민감도 | 빠른 움직임 시 오차 | 동작 패턴 제한, 보정 모델 |
| Rolling Shutter | 빠른 회전 시 왜곡 | RS 지원 프레임워크 (VINS, OpenVINS) |

---

## References

### 논문 및 기술 보고서
- [Smartphone MEMS Accelerometer and Gyroscope Measurement Errors](https://www.mdpi.com/1424-8220/23/17/7609)
- [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator](https://arxiv.org/abs/1708.03852)
- [OpenVINS: A Research Platform for Visual-Inertial Estimation](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)
- [Deep Learning for Inertial Positioning: A Survey](https://arxiv.org/html/2303.03757v3)
- [A Plug-and-Play Learning-based IMU Bias Factor](https://arxiv.org/html/2503.12527v1)
- [Inertial Navigation Meets Deep Learning: A Survey](https://arxiv.org/html/2307.00014v2)

### 도구 및 라이브러리
- [Kalibr IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [imu_utils - Allan Variance Analysis](https://github.com/gaowenliang/imu_utils)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [MATLAB Allan Variance Analysis](https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html)

### 관련 문서
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [07_android_sensor_api.md](07_android_sensor_api.md)
- [05_vislam_frameworks.md](05_vislam_frameworks.md)
