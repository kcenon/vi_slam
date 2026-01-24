# VINS-Mono Smartphone Configuration Guide

스마트폰 카메라/IMU 데이터를 VINS-Mono에서 사용하기 위한 상세 설정 가이드.

---

## 1. 개요

### 1.1 왜 VINS-Mono인가?

| 특성 | VINS-Mono | ORB-SLAM3 | OpenVINS |
|------|-----------|-----------|----------|
| 시간 오프셋 자동 추정 | **지원** | 미지원 | 지원 |
| 비동기 센서 허용 | **예** | 아니오 | 예 |
| Rolling Shutter | **지원** | 미지원 | 지원 |
| 스마트폰 적합성 | **높음** | 낮음 | 중간 |

**스마트폰 데이터에 VINS-Mono가 적합한 이유**:
- 카메라-IMU 시간 오프셋 온라인 추정 (`estimate_td`)
- 외부 파라미터 온라인 캘리브레이션 (`estimate_extrinsic`)
- Rolling Shutter 모델 지원
- 강건한 초기화 및 실패 복구

### 1.2 센서 품질 등급

VINS-Mono 개발팀이 제시한 예상 성능 순위:

```
최상 → 최하:

1. Global Shutter + 동기화된 고급 IMU (VI-Sensor)
2. Global Shutter + 동기화된 저가 IMU
3. Global Shutter + 비동기 고주파 IMU
4. Global Shutter + 비동기 저주파 IMU
5. Rolling Shutter + 비동기 저주파 IMU ← 스마트폰
```

**주의**: 웹캠은 사용하지 마세요. ("Don't try web camera, the web camera is so awful.")

---

## 2. 설정 파일 구조

### 2.1 전체 구조

```yaml
%YAML:1.0

#------------------------------------------------------------------------------
# 토픽 및 출력 설정
#------------------------------------------------------------------------------
imu_topic: "/smartphone/imu"
image_topic: "/smartphone/camera/image_raw"
output_path: "/home/user/vins_output/"

#------------------------------------------------------------------------------
# 카메라 파라미터
#------------------------------------------------------------------------------
model_type: PINHOLE
camera_name: "smartphone_camera"
image_width: 1280
image_height: 720

distortion_parameters:
   k1: -0.28
   k2: 0.08
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 900.0
   fy: 900.0
   cx: 640.0
   cy: 360.0

#------------------------------------------------------------------------------
# 외부 파라미터 (Camera-IMU)
#------------------------------------------------------------------------------
estimate_extrinsic: 2  # 0: 고정, 1: 최적화, 2: 자동 캘리브레이션

# 초기 추정값 (estimate_extrinsic: 1 또는 2일 때)
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1,
          -1, 0, 0,
          0, -1, 0]
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

#------------------------------------------------------------------------------
# 시간 동기화
#------------------------------------------------------------------------------
estimate_td: 1         # 시간 오프셋 온라인 추정
td: 0.02               # 초기 추정값 (초)

#------------------------------------------------------------------------------
# Rolling Shutter
#------------------------------------------------------------------------------
rolling_shutter: 1     # 1: 활성화
rolling_shutter_tr: 0.033  # 롤링셔터 readout 시간 (초)

#------------------------------------------------------------------------------
# IMU 파라미터
#------------------------------------------------------------------------------
acc_n: 0.1             # 가속도계 노이즈
gyr_n: 0.01            # 자이로스코프 노이즈
acc_w: 0.002           # 가속도계 랜덤 워크
gyr_w: 0.0002          # 자이로스코프 랜덤 워크
g_norm: 9.81           # 중력 크기

#------------------------------------------------------------------------------
# 특징점 추적
#------------------------------------------------------------------------------
max_cnt: 200           # 최대 특징점 수
min_dist: 25           # 특징점 간 최소 거리 (픽셀)
freq: 10               # 출력 주파수 (Hz)
F_threshold: 1.0       # RANSAC 임계값 (픽셀)
show_track: 1          # 추적 시각화
equalize: 1            # 히스토그램 평활화
fisheye: 0             # 피쉬아이 렌즈

#------------------------------------------------------------------------------
# 최적화
#------------------------------------------------------------------------------
max_solver_time: 0.04  # 최대 솔버 시간 (초)
max_num_iterations: 8  # 최대 반복 횟수
keyframe_parallax: 10.0  # 키프레임 시차 (픽셀)

#------------------------------------------------------------------------------
# 루프 클로저
#------------------------------------------------------------------------------
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/user/vins_output/pose_graph/"

#------------------------------------------------------------------------------
# 시각화
#------------------------------------------------------------------------------
save_image: 1
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

---

## 3. 핵심 파라미터 상세 설명

### 3.1 카메라 캘리브레이션

#### 지원 모델
- `PINHOLE`: 핀홀 모델 (스마트폰 표준)
- `MEI`: Unified camera model (피쉬아이)

#### 캘리브레이션 방법

```bash
# 1. OpenCV로 캘리브레이션
rosrun camera_calibration cameracalibrator.py \
  --size 9x6 \
  --square 0.025 \
  image:=/smartphone/camera/image_raw

# 2. Kalibr로 캘리브레이션
rosrun kalibr kalibr_calibrate_cameras \
  --bag /path/to/calibration.bag \
  --topics /smartphone/camera/image_raw \
  --models pinhole-radtan \
  --target april_6x6.yaml
```

#### 스마트폰 일반 값

```yaml
# 1280x720 해상도 기준 대략적 초기값
image_width: 1280
image_height: 720

projection_parameters:
   fx: 900.0    # 수평 초점거리 (픽셀)
   fy: 900.0    # 수직 초점거리 (픽셀)
   cx: 640.0    # 주점 x (이미지 중심)
   cy: 360.0    # 주점 y (이미지 중심)

distortion_parameters:
   k1: -0.28    # 방사 왜곡 1차
   k2: 0.08     # 방사 왜곡 2차
   p1: 0.0      # 접선 왜곡
   p2: 0.0
```

**중요**: Rolling Shutter 카메라는 reprojection error가 **0.5 픽셀 이하**가 되도록 정밀 캘리브레이션 필요.

### 3.2 외부 파라미터 (Camera-IMU Transform)

#### estimate_extrinsic 옵션

| 값 | 의미 | 사용 시점 |
|----|------|----------|
| **0** | 고정값 사용 | 정확한 캘리브레이션 완료 |
| **1** | 초기값 주변 최적화 | 대략적 캘리브레이션 완료 |
| **2** | 완전 자동 추정 | **스마트폰 권장** |

#### 스마트폰 권장 설정

```yaml
# 스마트폰: 캘리브레이션 없이 시작
estimate_extrinsic: 2

# 초기화 시 몇 초간 회전 동작 필요!
```

**사용법**:
1. `estimate_extrinsic: 2` 설정
2. 시스템 시작 시 **3-5초간 다양한 축으로 회전**
3. 성공 시 결과가 자동 저장됨

#### 일반적인 스마트폰 Camera-IMU 배치

```yaml
# 스마트폰 뒷면 카메라 + IMU 일반적 배치
# (기기마다 다름, 정확한 값은 캘리브레이션 필요)

# 카메라 → IMU 회전 (대략적)
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1,      # 카메라 Z → IMU X
          -1, 0, 0,     # 카메라 X → IMU -Y
          0, -1, 0]     # 카메라 Y → IMU -Z

# 카메라 → IMU 평행이동 (cm 단위, 대략적)
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.02, 0.0]  # 약 2cm 오프셋
```

### 3.3 시간 동기화

#### 스마트폰에서의 시간 오프셋

```
스마트폰 특성:
- 카메라와 IMU가 비동기
- 일반적 시간 오프셋: 10-50ms
- 기기/OS 버전에 따라 다름
```

#### 설정

```yaml
# 필수: 시간 오프셋 추정 활성화
estimate_td: 1

# 초기값 (양수 = 카메라가 IMU보다 늦음)
td: 0.02  # 20ms 초기 추정

# 공식: 보정된_이미지_시간 = 원본_이미지_시간 + td
```

#### 시간 오프셋 추정 확인

```bash
# ROS 토픽에서 추정된 td 확인
rostopic echo /vins_estimator/td

# 정상 범위: -0.05 ~ 0.05 (±50ms)
# 이 범위를 벗어나면 데이터 문제 의심
```

### 3.4 Rolling Shutter 설정

#### Rolling Shutter란?

```
Global Shutter: 모든 픽셀 동시 노출
Rolling Shutter: 행 단위 순차 노출

    ┌─────────────────┐
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ ← t = 0
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ ← t = Δt
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ ← t = 2Δt
    │       ...       │
    │ ░░░░░░░░░░░░░░░ │ ← t = tr (readout time)
    └─────────────────┘
```

#### 스마트폰 설정

```yaml
# 대부분의 스마트폰 카메라는 Rolling Shutter
rolling_shutter: 1

# Readout time (센서 데이터시트 참조)
# 일반적 범위: 0.01 ~ 0.05초
rolling_shutter_tr: 0.033  # 약 33ms (30fps 기준)
```

**Rolling Shutter Readout Time 추정**:
```
30fps 카메라: tr ≈ 1/30 = 0.033초
60fps 카메라: tr ≈ 1/60 = 0.017초
```

### 3.5 IMU 노이즈 파라미터

#### 파라미터 정의

| 파라미터 | 의미 | 단위 | 스마트폰 일반값 |
|----------|------|------|----------------|
| `acc_n` | 가속도계 노이즈 밀도 | m/s²/√Hz | 0.05-0.2 |
| `gyr_n` | 자이로스코프 노이즈 밀도 | rad/s/√Hz | 0.005-0.02 |
| `acc_w` | 가속도계 바이어스 랜덤워크 | m/s³/√Hz | 0.001-0.005 |
| `gyr_w` | 자이로스코프 바이어스 랜덤워크 | rad/s²/√Hz | 1e-5 - 5e-4 |

#### 스마트폰 등급별 추천값

```yaml
# 플래그십 스마트폰 (Pixel, iPhone Pro, Galaxy S)
acc_n: 0.08
gyr_n: 0.008
acc_w: 0.001
gyr_w: 5.0e-5

# 중급 스마트폰
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.002
gyr_w: 0.0002

# 저가 스마트폰 (보수적 설정)
acc_n: 0.2
gyr_n: 0.02
acc_w: 0.005
gyr_w: 0.0005
```

#### Allan Variance 결과 변환

```python
# imu_utils 출력 → VINS-Mono 형식 변환
# 주의: VINS-Mono는 이산 시간 형식 사용

import math

def convert_to_vins(allan_result, imu_rate=200):
    """
    Allan variance 결과를 VINS-Mono 형식으로 변환

    Args:
        allan_result: imu_utils 출력
        imu_rate: IMU 샘플링 주파수 (Hz)
    """
    dt = 1.0 / imu_rate

    # 연속 → 이산 변환
    acc_n = allan_result['acc_noise_density'] / math.sqrt(dt)
    gyr_n = allan_result['gyro_noise_density'] / math.sqrt(dt)
    acc_w = allan_result['acc_random_walk'] * math.sqrt(dt)
    gyr_w = allan_result['gyro_random_walk'] * math.sqrt(dt)

    return {
        'acc_n': acc_n,
        'gyr_n': gyr_n,
        'acc_w': acc_w,
        'gyr_w': gyr_w
    }
```

**간단한 방법** (정밀 캘리브레이션 없이):
1. 기본값으로 시작
2. 드리프트가 심하면 노이즈 값 증가
3. 추적이 불안정하면 노이즈 값 감소

### 3.6 특징점 추적 파라미터

#### 핵심 파라미터

```yaml
# 최대 특징점 수 (성능 vs 정확도 트레이드오프)
max_cnt: 200       # 스마트폰 권장: 150-250

# 특징점 간 최소 거리 (픽셀)
min_dist: 25       # 25-35 권장

# RANSAC 임계값 (아웃라이어 제거)
F_threshold: 1.0   # 1.0-2.0

# 출력 주파수 (최소 10Hz 권장)
freq: 10

# 저조도 환경 대응
equalize: 1        # 히스토그램 평활화 활성화
```

#### 환경별 튜닝

| 환경 | max_cnt | min_dist | equalize |
|------|---------|----------|----------|
| 밝은 실내 | 150 | 30 | 0 |
| 어두운 실내 | 200 | 25 | 1 |
| 야외 | 150 | 30 | 1 |
| 텍스처 부족 | 250 | 20 | 1 |

### 3.7 최적화 파라미터

```yaml
# 솔버 시간 제한 (실시간 성능)
max_solver_time: 0.04      # 40ms (25fps 실시간 가능)

# 최대 반복 횟수
max_num_iterations: 8      # 8-12 범위

# 키프레임 선택 기준 (시차, 픽셀)
keyframe_parallax: 10.0    # 10-15 권장
```

**모바일/저사양 PC**:
```yaml
max_solver_time: 0.06      # 여유있게 설정
max_num_iterations: 6      # 반복 줄임
```

---

## 4. 스마트폰 전용 설정 템플릿

### 4.1 Android 스마트폰 (일반)

```yaml
%YAML:1.0

#------------------------------------------------------------------------------
# Android Smartphone Configuration for VINS-Mono
# 테스트 환경: Samsung Galaxy S21, Android 12
#------------------------------------------------------------------------------

imu_topic: "/android/imu"
image_topic: "/android/camera/image_raw"
output_path: "/home/user/vins_android_output/"

# 카메라 (1280x720 @ 30fps)
model_type: PINHOLE
camera_name: "android_rear_camera"
image_width: 1280
image_height: 720

distortion_parameters:
   k1: -0.25
   k2: 0.06
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 920.0
   fy: 920.0
   cx: 640.0
   cy: 360.0

# 외부 파라미터 - 자동 추정
estimate_extrinsic: 2

extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]

extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

# 시간 동기화 - 필수
estimate_td: 1
td: 0.025

# Rolling Shutter - 활성화
rolling_shutter: 1
rolling_shutter_tr: 0.033

# IMU 파라미터 (중급 스마트폰)
acc_n: 0.1
gyr_n: 0.01
acc_w: 0.002
gyr_w: 0.0002
g_norm: 9.81

# 특징점 추적
max_cnt: 200
min_dist: 25
freq: 10
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0

# 최적화
max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

# 루프 클로저
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/user/vins_android_output/pose_graph/"

# 시각화
save_image: 1
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

### 4.2 iPhone (iOS)

```yaml
%YAML:1.0

#------------------------------------------------------------------------------
# iPhone Configuration for VINS-Mono
# 참고: iOS는 VINS-Mobile 사용 권장
#------------------------------------------------------------------------------

imu_topic: "/iphone/imu"
image_topic: "/iphone/camera/image_raw"
output_path: "/home/user/vins_iphone_output/"

# 카메라 (1920x1080 @ 30fps)
model_type: PINHOLE
camera_name: "iphone_rear_camera"
image_width: 1920
image_height: 1080

distortion_parameters:
   k1: -0.20
   k2: 0.04
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 1400.0
   fy: 1400.0
   cx: 960.0
   cy: 540.0

# 외부 파라미터
estimate_extrinsic: 2

extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0, 0, 1, -1, 0, 0, 0, -1, 0]

extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

# 시간 동기화
estimate_td: 1
td: 0.015  # iPhone은 동기화가 더 좋음

# Rolling Shutter
rolling_shutter: 1
rolling_shutter_tr: 0.033

# IMU 파라미터 (iPhone은 품질 좋음)
acc_n: 0.08
gyr_n: 0.008
acc_w: 0.001
gyr_w: 5.0e-5
g_norm: 9.81

# 특징점 추적
max_cnt: 180
min_dist: 30
freq: 10
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0

# 최적화
max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

# 루프 클로저
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "/home/user/vins_iphone_output/pose_graph/"

# 시각화
save_image: 1
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

---

## 5. 트러블슈팅

### 5.1 초기화 실패

**증상**: 시스템이 시작되지 않거나 "Initializing..." 상태 지속

**원인 및 해결책**:

| 원인 | 해결책 |
|------|--------|
| 움직임 부족 | 시작 시 **다양한 축으로 3-5초 회전** |
| 특징점 부족 | `max_cnt` 증가, `equalize: 1` 확인 |
| 데이터 시작점 문제 | 녹화 시작 후 1-2초 후부터 사용 |
| IMU 노이즈 과소평가 | `acc_n`, `gyr_n` 값 증가 |

```bash
# 초기화 상태 확인
rostopic echo /vins_estimator/vins_state
# 0: initializing, 1: initialized
```

### 5.2 심한 드리프트

**증상**: 시간이 지남에 따라 궤적이 크게 벗어남

**체크리스트**:

```yaml
# 1. IMU 노이즈 파라미터 확인 (너무 작으면 드리프트)
acc_n: 0.1    # 증가 시도
gyr_n: 0.01   # 증가 시도

# 2. 시간 동기화 확인
estimate_td: 1  # 반드시 활성화

# 3. 외부 파라미터 추정 확인
estimate_extrinsic: 2  # 또는 1

# 4. 특징점 수 확인
max_cnt: 200   # 부족하면 증가
```

**로그 확인**:
```bash
# IMU 바이어스 추정값 확인
rostopic echo /vins_estimator/imu_bias

# 비정상: |acc_bias| > 1.0 m/s² 또는 |gyro_bias| > 0.1 rad/s
```

### 5.3 추적 실패 (Tracking Lost)

**증상**: 갑자기 추적이 멈추거나 재초기화

**원인 및 해결책**:

| 원인 | 해결책 |
|------|--------|
| 빠른 회전/이동 | 천천히 움직임 |
| 저조도 | `equalize: 1`, 조명 개선 |
| 모션 블러 | 노출 시간 단축 |
| 텍스처 부족 | `max_cnt` 증가, `min_dist` 감소 |
| Rolling Shutter 미설정 | `rolling_shutter: 1` 확인 |

```yaml
# 추적 안정성 향상 설정
max_cnt: 250          # 특징점 증가
min_dist: 20          # 밀집 허용
F_threshold: 1.5      # RANSAC 완화
equalize: 1           # 히스토그램 평활화
```

### 5.4 시간 오프셋 추정 이상

**증상**: `td` 값이 비정상적으로 크거나 불안정

**정상 범위**: -0.05초 ~ +0.05초 (±50ms)

**문제 해결**:
```yaml
# 1. 초기값을 합리적 범위로 설정
td: 0.02  # 20ms로 시작

# 2. 데이터 품질 확인
# - 카메라: 최소 20Hz
# - IMU: 최소 100Hz (200Hz 권장)

# 3. 타임스탬프 검증
rostopic echo /android/imu --noarr | head -20
rostopic echo /android/camera/image_raw --noarr | head -20
# 타임스탬프가 단조 증가하는지 확인
```

### 5.5 루프 클로저 실패

**증상**: 같은 장소를 재방문해도 루프가 닫히지 않음

```yaml
# 루프 클로저 설정 확인
loop_closure: 1
fast_relocalization: 0  # 안정성 우선

# 특징점 설정 (루프 감지에 영향)
max_cnt: 200  # 충분한 특징점 필요
```

---

## 6. 데이터 수집 권장사항

### 6.1 센서 요구사항

| 센서 | 최소 | 권장 | 비고 |
|------|------|------|------|
| 카메라 FPS | 20 Hz | 30 Hz | 일정한 프레임레이트 |
| IMU 샘플링 | 100 Hz | 200 Hz | 카메라보다 높아야 함 |
| 해상도 | 640x480 | 1280x720 | 너무 높으면 처리 느림 |

### 6.2 데이터 수집 팁

1. **시작 시**: 정지 상태에서 1-2초 대기 후 다양한 축으로 회전
2. **이동 중**: 급격한 움직임 피하기, 일정한 속도 유지
3. **환경**: 충분한 텍스처, 적절한 조명
4. **종료 시**: 정지 상태에서 1-2초 대기

### 6.3 Android 앱 설정 (OpenCamera Sensors 기준)

```
카메라 설정:
- 해상도: 1280x720
- FPS: 30
- 노출: 자동 또는 고정
- 화이트밸런스: 고정

IMU 설정:
- 샘플링 레이트: 200Hz
- 센서 타입: UNCALIBRATED 권장
- 타임스탬프: elapsedRealtimeNanos 사용
```

---

## 7. 실행 및 검증

### 7.1 실행 명령

```bash
# 1. 설정 파일 경로 지정
roslaunch vins_estimator vins_rviz.launch
roslaunch vins_estimator smartphone.launch \
  config_path:=/path/to/smartphone_config.yaml

# 2. 데이터 재생 (rosbag 사용 시)
rosbag play smartphone_data.bag
```

### 7.2 실시간 모니터링

```bash
# 추적 상태
rostopic echo /vins_estimator/vins_state

# 포즈 출력
rostopic echo /vins_estimator/odometry

# 시간 오프셋
rostopic echo /vins_estimator/td

# 특징점 수
rostopic echo /vins_estimator/feature_tracker/feature --noarr | grep -c "point"
```

### 7.3 결과 평가

```bash
# 궤적 저장
# output_path에 자동 저장됨

# EVO 도구로 평가 (Ground Truth 있을 경우)
evo_ape tum groundtruth.txt vins_result.txt -va --plot
```

---

## 8. 참고 자료

### 공식 문서
- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Mono README](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/README.md)
- [VINS-Mobile (iOS)](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile)

### 설정 예제
- [EuRoC Config](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/euroc/euroc_config.yaml)
- [TUM Config](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/tum/tum_config.yaml)

### 캘리브레이션 도구
- [Kalibr](https://github.com/ethz-asl/kalibr)
- [imu_utils](https://github.com/gaowenliang/imu_utils)

### 관련 문서
- [05_vislam_frameworks.md](05_vislam_frameworks.md)
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)
- [13_imu_advanced_considerations.md](13_imu_advanced_considerations.md)
