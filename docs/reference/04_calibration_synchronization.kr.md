# Camera-IMU Calibration and Synchronization

## 1. 개요

VI-SLAM 시스템에서 카메라와 IMU 간의 정확한 캘리브레이션과 시간 동기화는 시스템 성능에 결정적인 영향을 미친다.

### 캘리브레이션 유형

| 유형 | 설명 | 중요도 |
|------|------|--------|
| Spatial (공간) | 카메라-IMU 간 상대 위치/회전 | 매우 높음 |
| Temporal (시간) | 카메라-IMU 간 시간 오프셋 | 매우 높음 |
| Intrinsic (내부) | 각 센서 고유 파라미터 | 높음 |

## 2. 카메라 캘리브레이션

### 2.1 카메라 내부 파라미터 (Intrinsics)

```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]

fx, fy: 초점 거리 (픽셀 단위)
cx, cy: 주점 (principal point)
```

### 2.2 렌즈 왜곡 모델

#### Radial Distortion
```
x_distorted = x(1 + k1*r² + k2*r⁴ + k3*r⁶)
y_distorted = y(1 + k1*r² + k2*r⁴ + k3*r⁶)
```

#### Tangential Distortion
```
x_distorted = x + [2*p1*xy + p2*(r² + 2x²)]
y_distorted = y + [p1*(r² + 2y²) + 2*p2*xy]
```

### 2.3 캘리브레이션 도구

#### OpenCV
```python
import cv2
import numpy as np

# 체커보드 코너 검출
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

# 캘리브레이션
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
```

#### Kalibr
```bash
# 카메라 캘리브레이션
rosrun kalibr kalibr_calibrate_cameras \
  --bag /path/to/bag \
  --topics /cam0/image_raw \
  --models pinhole-radtan \
  --target /path/to/target.yaml
```

## 3. IMU 캘리브레이션

### 3.1 IMU 내부 파라미터

#### 바이어스 (Bias)
```
가속도계 바이어스: b_a = [bax, bay, baz]
자이로스코프 바이어스: b_g = [bgx, bgy, bgz]
```

#### 노이즈 특성
```
가속도계 노이즈 밀도: σ_a (m/s²/√Hz)
자이로스코프 노이즈 밀도: σ_g (rad/s/√Hz)
가속도계 랜덤 워크: σ_ba (m/s³/√Hz)
자이로스코프 랜덤 워크: σ_bg (rad/s²/√Hz)
```

### 3.2 IMU 노이즈 파라미터 추정

#### Allan Variance 분석
```python
# imu_utils 사용
# https://github.com/gaowenliang/imu_utils

roslaunch imu_utils imu.launch
# 2시간 이상 정지 상태에서 IMU 데이터 수집 필요
```

#### 일반적인 MEMS IMU 노이즈 파라미터
```yaml
# 저가형 스마트폰 IMU 예시
accelerometer_noise_density: 0.01     # m/s²/√Hz
accelerometer_random_walk: 0.0002    # m/s³/√Hz
gyroscope_noise_density: 0.005       # rad/s/√Hz
gyroscope_random_walk: 4.0e-06       # rad/s²/√Hz
```

## 4. Camera-IMU Extrinsic 캘리브레이션

### 4.1 변환 행렬

```
T_cam_imu = [R_cam_imu | t_cam_imu]
            [    0     |     1    ]

R_cam_imu: IMU에서 카메라로의 회전 (3x3)
t_cam_imu: IMU에서 카메라로의 평행이동 (3x1)
```

### 4.2 Kalibr를 이용한 캘리브레이션

#### 준비물
- Aprilgrid 또는 체커보드 타겟
- 캘리브레이션된 카메라
- IMU 데이터

#### 캘리브레이션 과정
```bash
# Camera-IMU 캘리브레이션
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /path/to/bag \
  --cam /path/to/cam_chain.yaml \
  --imu /path/to/imu.yaml \
  --target /path/to/target.yaml
```

#### 권장 설정
- 카메라: 20Hz
- IMU: 200Hz
- 동작: 3축 모두 여기 (excitation)
- 시간: 60-120초

### 4.3 캘리브레이션 품질 확인

```
좋은 캘리브레이션 지표:
- Reprojection error: < 0.5 픽셀
- Gyro error: < 0.01 rad/s
- Accel error: < 0.1 m/s²
```

## 5. 시간 동기화 (Temporal Calibration)

### 5.1 시간 오프셋의 영향

```
오프셋 1ms 당 영향 (일반적인 동작 속도):
- 회전 1 rad/s → 1 mrad 오차
- 이동 1 m/s → 1 mm 오차

빠른 동작에서는 더 큰 오차 발생
```

### 5.2 동기화 방법

#### Hardware Synchronization (최선)
```
방법: 외부 트리거를 사용하여 카메라 셔터와 IMU 샘플링 동기화
정확도: < 1ms
요구사항: 하드웨어 수정 필요

[Trigger Signal] ──┬──→ [Camera]
                   └──→ [IMU]
```

#### Software Synchronization
```
방법: 시스템 클럭 타임스탬프 사용
정확도: 1-10ms (OS/플랫폼 의존)
장점: 하드웨어 수정 불필요
```

#### Online Temporal Calibration
```
방법: 알고리즘이 실행 중 시간 오프셋 추정
지원 알고리즘:
- VINS-Mono: 지원 (자동 추정)
- ORB-SLAM3: 미지원 (사전 동기화 필수)
- OpenVINS: 지원
```

### 5.3 시간 오프셋 추정 방법

#### Angular Velocity Correlation
```python
# 카메라에서 추정한 각속도와 IMU 각속도 상관분석
def estimate_time_offset(camera_angular_vel, imu_angular_vel, timestamps):
    # Cross-correlation으로 초기 오프셋 추정
    correlation = np.correlate(camera_angular_vel, imu_angular_vel, mode='full')
    offset_samples = np.argmax(correlation) - len(camera_angular_vel) + 1

    # Quadratic fitting으로 sub-sample 정확도 향상
    # ...
    return time_offset
```

#### Kalibr Temporal Calibration
```bash
# 시간 오프셋도 함께 추정
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /path/to/bag \
  --cam cam.yaml \
  --imu imu.yaml \
  --target target.yaml \
  --time-calibration
```

### 5.4 일반적인 시간 오프셋 값

| 플랫폼 | 일반적 오프셋 | 변동성 |
|--------|-------------|--------|
| 고급 센서 (RealSense) | < 1ms | 낮음 |
| 스마트폰 (고급) | 5-20ms | 중간 |
| 스마트폰 (저가) | 10-50ms | 높음 |
| 자체 조립 | 가변 | 매우 높음 |

## 6. 스마트폰 특수 고려사항

### 6.1 Rolling Shutter 보정

대부분의 스마트폰 카메라는 Rolling Shutter 사용:

```
Global Shutter: 모든 픽셀 동시 노출
Rolling Shutter: 행 단위 순차 노출

Rolling Shutter Skew: 첫 행과 마지막 행 사이 시간차
일반적인 값: 10-30ms (센서 종속)
```

#### 보정 방법
- VINS-Mono: Rolling Shutter 모델 지원
- ORB-SLAM3: Rolling Shutter 미지원
- OpenVINS: Rolling Shutter 지원

### 6.2 OIS (Optical Image Stabilization)

```
문제점:
- 프레임마다 광학 파라미터 변화
- 대부분의 스마트폰에서 비활성화 불가

해결책:
- OIS 데이터 제공 기기 사용 (일부 Android)
- OIS 미지원 카메라 사용 (별도 외장 카메라)
- OIS 변화량을 캘리브레이션에 반영
```

## 7. 캘리브레이션 도구 요약

| 도구 | 플랫폼 | 기능 | 링크 |
|------|--------|------|------|
| Kalibr | ROS | Camera + IMU + Time | [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) |
| OpenCV | Cross | Camera intrinsics | [opencv.org](https://opencv.org/) |
| imu_utils | ROS | IMU noise | [gaowenliang/imu_utils](https://github.com/gaowenliang/imu_utils) |
| OpenVINS Calibration | ROS | Full pipeline | [docs.openvins.com](https://docs.openvins.com/gs-calibration.html) |

## 8. 캘리브레이션 워크플로우

```
1. 카메라 Intrinsic 캘리브레이션
   └─→ OpenCV 또는 Kalibr

2. IMU Intrinsic 캘리브레이션 (선택)
   └─→ imu_utils (Allan Variance)

3. Camera-IMU Extrinsic 캘리브레이션
   └─→ Kalibr

4. Temporal Calibration
   └─→ Kalibr (오프라인) 또는 VINS-Mono (온라인)

5. 검증
   └─→ Reprojection error, 궤적 정확도 확인
```

## References

- [Kalibr Camera-IMU Calibration Wiki](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [Online Temporal Calibration Paper](https://arxiv.org/pdf/1808.00692)
- [Camera-IMU Time Offset Modeling (ECCV 2018)](https://openaccess.thecvf.com/content_ECCV_2018/papers/Yonggen_Ling_Modeling_Varying_Camera-IMU_ECCV_2018_paper.pdf)
- [Hardware Synchronization Project](https://github.com/TurtleZhong/Visual-Inertial-Synchronization-Hardware)
