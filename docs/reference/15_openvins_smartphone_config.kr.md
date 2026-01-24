# OpenVINS Smartphone Configuration Guide

스마트폰 카메라/IMU 데이터를 OpenVINS에서 사용하기 위한 상세 설정 가이드.

---

## 1. 개요

### 1.1 OpenVINS 특징

| 특성 | OpenVINS | VINS-Mono | ORB-SLAM3 |
|------|----------|-----------|-----------|
| 필터 방식 | **MSCKF (EKF)** | 최적화 | 최적화 |
| CPU 사용량 | **낮음** | 중간 | 높음 |
| 온라인 캘리브레이션 | **포괄적** | 부분 | 미지원 |
| 문서화 | **최고** | 중간 | 낮음 |
| Loop Closure | 미지원 | 지원 | 지원 |

### 1.2 OpenVINS가 스마트폰에 적합한 이유

1. **낮은 계산 부하**: MSCKF 기반 필터로 CPU 효율적
2. **포괄적 온라인 캘리브레이션**:
   - 카메라-IMU 외부 파라미터 (`calib_cam_extrinsics`)
   - 카메라 내부 파라미터 (`calib_cam_intrinsics`)
   - 시간 오프셋 (`calib_cam_timeoffset`)
   - IMU 내부 파라미터 (`calib_imu_intrinsics`)
3. **우수한 문서화**: 상세한 캘리브레이션 가이드 제공
4. **Kalibr 호환**: 표준 캘리브레이션 포맷 사용

### 1.3 제한사항

- **Loop Closure 미지원**: VIO만 제공 (장거리 드리프트 누적)
- **최적화 기반 대비 정확도**: VINS-Mono보다 약간 낮음

---

## 2. 설정 파일 구조

### 2.1 필요한 설정 파일

```
config/
├── smartphone/
│   ├── estimator_config.yaml      # OpenVINS 핵심 설정
│   ├── kalibr_imu_chain.yaml      # IMU 노이즈 파라미터
│   └── kalibr_imucam_chain.yaml   # 카메라-IMU 변환 및 카메라 내부 파라미터
```

### 2.2 estimator_config.yaml (메인 설정)

```yaml
%YAML:1.0

#=============================================================================
# OpenVINS Smartphone Configuration
# 테스트 환경: Android/iOS 스마트폰
#=============================================================================

#-----------------------------------------------------------------------------
# 일반 설정
#-----------------------------------------------------------------------------
verbosity: "INFO"                    # ALL, DEBUG, INFO, WARNING, ERROR, SILENT

#-----------------------------------------------------------------------------
# 상태 옵션
#-----------------------------------------------------------------------------
use_fej: true                        # First Estimates Jacobian 사용
use_imuavg: true                     # IMU 측정값 평균화
use_rk4_integration: true            # RK4 적분 (false: Euler)
use_stereo: false                    # 스테레오 모드 (스마트폰: false)
max_cameras: 1                       # 카메라 수 (스마트폰: 1)
calib_cam_extrinsics: true           # 카메라-IMU 외부 파라미터 온라인 추정
calib_cam_intrinsics: true           # 카메라 내부 파라미터 온라인 추정
calib_cam_timeoffset: true           # 시간 오프셋 온라인 추정
calib_imu_intrinsics: false          # IMU 내부 파라미터 추정 (고급)
max_clones: 11                       # 슬라이딩 윈도우 크기
max_slam: 50                         # SLAM 특징점 최대 수
max_slam_in_update: 25               # 업데이트당 SLAM 특징점
max_msckf_in_update: 40              # 업데이트당 MSCKF 특징점
dt_slam_delay: 1                     # SLAM 지연 (프레임)

#-----------------------------------------------------------------------------
# 초기화 옵션
#-----------------------------------------------------------------------------
init_window_time: 1.0                # 초기화 윈도우 크기 (초)
init_imu_thresh: 1.0                 # IMU 움직임 임계값
init_max_disparity: 1.5              # 초기화 최대 시차
init_max_features: 50                # 초기화 특징점 수

#-----------------------------------------------------------------------------
# 노이즈/공분산 기록
#-----------------------------------------------------------------------------
record_timing_information: false
record_timing_filepath: "/tmp/timing_ov.txt"

#-----------------------------------------------------------------------------
# 특징점 추적
#-----------------------------------------------------------------------------
use_klt: true                        # KLT 추적 (true 권장)
num_pts: 200                         # 카메라당 특징점 수
fast_threshold: 15                   # FAST 검출 임계값
grid_x: 5                            # 수평 그리드 분할
grid_y: 5                            # 수직 그리드 분할
min_px_dist: 10                      # 특징점 간 최소 거리 (픽셀)
knn_ratio: 0.70                      # KNN 매칭 비율 (ORB 사용 시)
track_frequency: 20.0                # 추적 주파수 (Hz)
downsample_cameras: false            # 이미지 다운샘플링
multi_threading_subs: false          # 멀티스레드 구독

# 히스토그램 평활화
histogram_method: "CLAHE"            # NONE, HISTOGRAM, CLAHE

#-----------------------------------------------------------------------------
# 상태 초기화
#-----------------------------------------------------------------------------
init_state:
  p_IinG: [0, 0, 0]
  q_GtoI: [0, 0, 0, 1]
  v_IinG: [0, 0, 0]
  bg: [0, 0, 0]
  ba: [0, 0, 0]

#-----------------------------------------------------------------------------
# MSCKF/SLAM 업데이트 옵션
#-----------------------------------------------------------------------------
up_msckf_sigma_px: 1.0               # MSCKF 픽셀 노이즈
up_msckf_chi2_multipler: 1.0         # MSCKF Chi2 배율
up_slam_sigma_px: 1.0                # SLAM 픽셀 노이즈
up_slam_chi2_multipler: 1.0          # SLAM Chi2 배율

#-----------------------------------------------------------------------------
# 제로 속도 업데이트 (ZUPT)
#-----------------------------------------------------------------------------
try_zupt: true                       # ZUPT 시도
zupt_chi2_multipler: 1.0             # ZUPT Chi2 배율
zupt_max_velocity: 0.1               # 정지 판단 속도 임계값 (m/s)
zupt_noise_multiplier: 10.0          # ZUPT 노이즈 배율
zupt_max_disparity: 0.5              # ZUPT 최대 시차

#-----------------------------------------------------------------------------
# 파일 경로 (런타임 오버라이드 가능)
#-----------------------------------------------------------------------------
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

### 2.3 kalibr_imu_chain.yaml (IMU 노이즈)

```yaml
%YAML:1.0

#=============================================================================
# IMU 노이즈 파라미터 (Kalibr 포맷)
# 중요: 실제 값의 10-20배로 설정 권장 (모델링되지 않은 오차 고려)
#=============================================================================

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: 0.1      # m/s²/√Hz (σ_a)
  accelerometer_random_walk: 0.002      # m/s³/√Hz (σ_ba)
  gyroscope_noise_density: 0.01         # rad/s/√Hz (σ_g)
  gyroscope_random_walk: 0.0002         # rad/s²/√Hz (σ_bg)
  model: calibrated
  rostopic: /smartphone/imu
  time_offset: 0.0
  update_rate: 200.0                    # IMU 업데이트 레이트 (Hz)
```

### 2.4 kalibr_imucam_chain.yaml (카메라-IMU)

```yaml
%YAML:1.0

#=============================================================================
# 카메라-IMU 캘리브레이션 (Kalibr 포맷)
#=============================================================================

cam0:
  T_imu_cam:                            # IMU에서 카메라로의 변환
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.0]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole                 # pinhole 또는 omni
  distortion_coeffs: [-0.28, 0.08, 0.0, 0.0]
  distortion_model: radtan              # radtan, equidistant
  intrinsics: [900.0, 900.0, 640.0, 360.0]  # [fx, fy, cx, cy]
  resolution: [1280, 720]
  rostopic: /smartphone/camera/image_raw
  timeshift_cam_imu: 0.02               # 카메라-IMU 시간 오프셋 (초)
```

---

## 3. 핵심 파라미터 상세 설명

### 3.1 IMU 노이즈 파라미터

#### 파라미터 정의

| 파라미터 | 기호 | 단위 | 설명 |
|----------|------|------|------|
| `accelerometer_noise_density` | σ_a | m/s²/√Hz | 가속도계 백색 잡음 |
| `gyroscope_noise_density` | σ_g | rad/s/√Hz | 자이로스코프 백색 잡음 |
| `accelerometer_random_walk` | σ_ba | m/s³/√Hz | 가속도계 바이어스 랜덤워크 |
| `gyroscope_random_walk` | σ_bg | rad/s²/√Hz | 자이로스코프 바이어스 랜덤워크 |

#### Allan Variance에서 값 추출

```
Allan Deviation Plot에서:

1. White Noise (σ): τ=1에서 slope=-1/2 직선의 값
2. Random Walk: slope=+1/2 직선에서 추출

              │
  log(σ(τ))   │   slope = -1/2
              │        ↘
              │          ↘_____ slope ≈ 0
              │                ↘
              │                  ↘ slope = +1/2
              └───────────────────────→ log(τ)
                        τ=1
```

#### 스마트폰 등급별 권장값

```yaml
# 플래그십 스마트폰 (10-20배 inflate 적용)
accelerometer_noise_density: 0.08
accelerometer_random_walk: 0.001
gyroscope_noise_density: 0.008
gyroscope_random_walk: 5.0e-5

# 중급 스마트폰
accelerometer_noise_density: 0.1
accelerometer_random_walk: 0.002
gyroscope_noise_density: 0.01
gyroscope_random_walk: 0.0002

# 저가 스마트폰 (보수적)
accelerometer_noise_density: 0.2
accelerometer_random_walk: 0.005
gyroscope_noise_density: 0.02
gyroscope_random_walk: 0.0005
```

**중요**: OpenVINS 문서에서 "이 노이즈 값들을 실제 값의 **10-20배로 inflate**하여 모델링되지 않은 오차를 고려하라"고 권장.

### 3.2 온라인 캘리브레이션 옵션

```yaml
# 카메라-IMU 외부 파라미터 (R_ItoC, p_CinI)
calib_cam_extrinsics: true    # 스마트폰: 활성화 권장

# 카메라 내부 파라미터 (초점거리, 중심점, 왜곡)
calib_cam_intrinsics: true    # 초기 캘리브레이션이 부정확하면 활성화

# 카메라-IMU 시간 오프셋
calib_cam_timeoffset: true    # 스마트폰: 필수 활성화

# IMU 내부 파라미터 (회전, 스케일 오차)
calib_imu_intrinsics: false   # 고급 옵션, 일반적으로 비활성화
```

### 3.3 특징점 추적 파라미터

#### 핵심 파라미터

```yaml
# 추적 방법
use_klt: true                 # KLT (true) vs ORB+매칭 (false)

# 특징점 수 (성능 vs 정확도)
num_pts: 200                  # 스마트폰 권장: 150-250

# FAST 검출 임계값 (낮을수록 더 많은 특징점, 더 느림)
fast_threshold: 15            # 15-25 권장

# 균일 분포를 위한 그리드
grid_x: 5                     # 이미지를 5x5 그리드로 분할
grid_y: 5

# 특징점 간 최소 거리
min_px_dist: 10               # 10-15 픽셀

# 추적 주파수 (카메라 FPS와 별도)
track_frequency: 20.0         # 20Hz 이상 권장
```

#### 히스토그램 평활화

```yaml
# 저조도/고대비 환경 대응
histogram_method: "CLAHE"     # NONE, HISTOGRAM, CLAHE

# CLAHE: Contrast Limited Adaptive Histogram Equalization
# - 국소적 대비 향상
# - 노이즈 증폭 제한
# - 스마트폰에서 효과적
```

### 3.4 초기화 파라미터

```yaml
# 초기화 윈도우
init_window_time: 1.0         # 초기화에 사용할 데이터 시간 (초)

# IMU 움직임 임계값
init_imu_thresh: 1.0          # 초기화 트리거 가속도 변화

# 시차 기반 초기화 (정적 초기화)
init_max_disparity: 1.5       # 최대 허용 시차 (픽셀)

# 초기화 특징점
init_max_features: 50         # 초기화에 사용할 특징점 수
```

**초기화 동작**:
1. 동적 초기화 시도 (IMU 움직임 필요)
2. 실패 시 정적 초기화 시도 (시차 기반)
3. 정지 상태면 움직임 대기

### 3.5 ZUPT (Zero Velocity Update)

```yaml
# ZUPT 활성화
try_zupt: true

# 정지 판단 임계값
zupt_max_velocity: 0.1        # m/s 이하면 정지로 판단
zupt_max_disparity: 0.5       # 픽셀 시차 임계값

# ZUPT 노이즈 설정
zupt_noise_multiplier: 10.0   # ZUPT 측정 노이즈 배율
zupt_chi2_multipler: 1.0      # Chi2 테스트 배율
```

**스마트폰에서의 ZUPT**:
- 손에 든 상태에서는 정지 상태 드묾
- 테이블 위 정지 시 효과적
- 드리프트 보정에 도움

---

## 4. 스마트폰 전용 설정 템플릿

### 4.1 Android 스마트폰

**estimator_config.yaml**:
```yaml
%YAML:1.0

#=============================================================================
# Android Smartphone Configuration for OpenVINS
#=============================================================================

verbosity: "INFO"

# 상태 옵션
use_fej: true
use_imuavg: true
use_rk4_integration: true
use_stereo: false
max_cameras: 1
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
calib_imu_intrinsics: false
max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

# 초기화 (Android는 약간 관대하게)
init_window_time: 1.5
init_imu_thresh: 0.8
init_max_disparity: 2.0
init_max_features: 50

# 특징점 추적
use_klt: true
num_pts: 200
fast_threshold: 15
grid_x: 5
grid_y: 5
min_px_dist: 10
knn_ratio: 0.70
track_frequency: 20.0
downsample_cameras: false
histogram_method: "CLAHE"

# 업데이트 옵션
up_msckf_sigma_px: 1.0
up_msckf_chi2_multipler: 1.0
up_slam_sigma_px: 1.0
up_slam_chi2_multipler: 1.0

# ZUPT
try_zupt: true
zupt_chi2_multipler: 1.0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10.0
zupt_max_disparity: 0.5

# 설정 파일 경로
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

**kalibr_imu_chain.yaml**:
```yaml
%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  # Android 중급 스마트폰 (inflate 적용)
  accelerometer_noise_density: 0.1
  accelerometer_random_walk: 0.002
  gyroscope_noise_density: 0.01
  gyroscope_random_walk: 0.0002
  model: calibrated
  rostopic: /android/imu
  time_offset: 0.0
  update_rate: 200.0
```

**kalibr_imucam_chain.yaml**:
```yaml
%YAML:1.0

cam0:
  T_imu_cam:
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.02]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.25, 0.06, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [920.0, 920.0, 640.0, 360.0]
  resolution: [1280, 720]
  rostopic: /android/camera/image_raw
  timeshift_cam_imu: 0.025
```

### 4.2 iPhone

**estimator_config.yaml**:
```yaml
%YAML:1.0

#=============================================================================
# iPhone Configuration for OpenVINS
#=============================================================================

verbosity: "INFO"

# 상태 옵션
use_fej: true
use_imuavg: true
use_rk4_integration: true
use_stereo: false
max_cameras: 1
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
calib_imu_intrinsics: false
max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

# 초기화 (iPhone은 동기화가 더 좋음)
init_window_time: 1.0
init_imu_thresh: 1.0
init_max_disparity: 1.5
init_max_features: 50

# 특징점 추적
use_klt: true
num_pts: 180
fast_threshold: 18
grid_x: 5
grid_y: 5
min_px_dist: 12
knn_ratio: 0.70
track_frequency: 25.0
downsample_cameras: false
histogram_method: "CLAHE"

# 업데이트 옵션
up_msckf_sigma_px: 1.0
up_msckf_chi2_multipler: 1.0
up_slam_sigma_px: 1.0
up_slam_chi2_multipler: 1.0

# ZUPT
try_zupt: true
zupt_chi2_multipler: 1.0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10.0
zupt_max_disparity: 0.5

# 설정 파일 경로
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

**kalibr_imu_chain.yaml**:
```yaml
%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  # iPhone (품질 좋음, inflate 적용)
  accelerometer_noise_density: 0.08
  accelerometer_random_walk: 0.001
  gyroscope_noise_density: 0.008
  gyroscope_random_walk: 5.0e-5
  model: calibrated
  rostopic: /iphone/imu
  time_offset: 0.0
  update_rate: 200.0
```

**kalibr_imucam_chain.yaml**:
```yaml
%YAML:1.0

cam0:
  T_imu_cam:
    - [0.0, 0.0, 1.0, 0.0]
    - [-1.0, 0.0, 0.0, 0.0]
    - [0.0, -1.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.20, 0.04, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [1400.0, 1400.0, 960.0, 540.0]
  resolution: [1920, 1080]
  rostopic: /iphone/camera/image_raw
  timeshift_cam_imu: 0.015
```

---

## 5. 캘리브레이션 워크플로우

### 5.1 IMU 노이즈 캘리브레이션

```bash
# 1. 정지 상태에서 IMU 데이터 수집 (최소 2시간, 권장 20시간+)
rosbag record /smartphone/imu -o imu_static.bag

# 2. Allan Variance 분석
# allan_variance_ros 사용
rosrun allan_variance_ros allan_variance \
  /smartphone/imu \
  imu_static.bag \
  --output imu_allan.yaml

# 3. 결과 확인 및 YAML 생성
# σ_g, σ_a, σ_bg, σ_ba 값 추출
# 10-20배 inflate하여 kalibr_imu_chain.yaml에 입력
```

### 5.2 카메라 내부 캘리브레이션

```bash
# 1. Aprilgrid 준비 (6x6, 0.8m x 0.8m 권장)

# 2. 캘리브레이션 데이터 수집
rosbag record /smartphone/camera/image_raw -o camera_calib.bag
# 다양한 각도, 거리에서 30-60초 촬영

# 3. Kalibr 실행
rosrun kalibr kalibr_calibrate_cameras \
  --bag camera_calib.bag \
  --topics /smartphone/camera/image_raw \
  --models pinhole-radtan \
  --target aprilgrid_6x6.yaml \
  --bag-freq 10.0

# 4. 결과 확인
# reprojection error < 0.2-0.5 픽셀이어야 함
```

### 5.3 카메라-IMU 외부 캘리브레이션

```bash
# 1. 동적 캘리브레이션 데이터 수집
rosbag record /smartphone/camera/image_raw /smartphone/imu -o imu_cam_calib.bag
# 30-60초, 다양한 회전 및 이동

# 2. 데이터 수집 팁:
# - 모션 블러 최소화 (부드러운 움직임)
# - 최소 1개 평행이동 + 2개 이상 회전 필요
# - 가능한 많은 방향으로 회전

# 3. Kalibr 실행
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag imu_cam_calib.bag \
  --cam camera_calib-results.yaml \
  --imu kalibr_imu_chain.yaml \
  --target aprilgrid_6x6.yaml

# 4. 결과 검증
# - 가속도계/자이로스코프 오차가 3σ 범위 내인지 확인
# - reprojection error 확인
```

---

## 6. 실행 방법

### 6.1 ROS 1

```bash
# 1. 설정 파일 배치
mkdir -p ~/catkin_ws/src/open_vins/config/smartphone/
# estimator_config.yaml, kalibr_*.yaml 복사

# 2. 실행
roscore

# 3. OpenVINS 실행
roslaunch ov_msckf subscribe.launch \
  config_path:=$(rospack find ov_msckf)/config/smartphone/estimator_config.yaml \
  use_stereo:=false \
  max_cameras:=1

# 4. RViz
rviz -d $(rospack find ov_msckf)/launch/display.rviz

# 5. 데이터 재생 또는 실시간 스트리밍
rosbag play smartphone_data.bag
```

### 6.2 ROS 2

```bash
# 1. 실행
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/path/to/smartphone/estimator_config.yaml \
  use_stereo:=false \
  max_cameras:=1

# 2. 데이터 재생
ros2 bag play smartphone_data
```

### 6.3 Launch 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `verbosity` | string | INFO | 로그 레벨 |
| `config_path` | string | - | 설정 파일 경로 |
| `use_stereo` | bool | true | 스테레오 모드 |
| `max_cameras` | int | 2 | 최대 카메라 수 |

---

## 7. 트러블슈팅

### 7.1 초기화 실패

**증상**: 시스템이 "Waiting for initialization..." 상태 지속

**해결책**:

| 원인 | 해결책 |
|------|--------|
| 움직임 부족 | 다양한 방향으로 이동/회전 |
| IMU 임계값 높음 | `init_imu_thresh: 0.5` 감소 |
| 특징점 부족 | `num_pts: 250` 증가, `fast_threshold: 12` 감소 |
| 정적 초기화 실패 | `init_max_disparity: 2.0` 증가 |

```yaml
# 초기화 완화 설정
init_window_time: 2.0
init_imu_thresh: 0.5
init_max_disparity: 2.0
init_max_features: 75
```

### 7.2 필터 발산 (Divergence)

**증상**: 궤적이 급격히 벗어남, 포즈 추정 불안정

**해결책**:

```yaml
# 1. IMU 노이즈 증가 (필터 강건성 향상)
accelerometer_noise_density: 0.2    # 2배 증가
gyroscope_noise_density: 0.02       # 2배 증가

# 2. Chi2 배율 증가 (아웃라이어 허용)
up_msckf_chi2_multipler: 2.0
up_slam_chi2_multipler: 2.0

# 3. 특징점 설정 조정
num_pts: 250                        # 증가
min_px_dist: 8                      # 감소
```

### 7.3 드리프트 심함

**증상**: 시간이 지남에 따라 점진적 위치/방향 오차 누적

**체크리스트**:

```yaml
# 1. IMU 노이즈 파라미터 확인 (너무 작으면 드리프트)
accelerometer_random_walk: 0.005    # 증가 시도
gyroscope_random_walk: 0.0005       # 증가 시도

# 2. 온라인 캘리브레이션 활성화 확인
calib_cam_extrinsics: true
calib_cam_timeoffset: true

# 3. ZUPT 활성화 (정지 시 드리프트 보정)
try_zupt: true

# 4. 슬라이딩 윈도우 크기 증가
max_clones: 15                      # 기본 11에서 증가
```

### 7.4 특징점 추적 불안정

**증상**: 특징점 수가 급변, 추적 끊김

```yaml
# 저조도 환경
histogram_method: "CLAHE"
fast_threshold: 12                  # 감소

# 텍스처 부족
num_pts: 300                        # 증가
grid_x: 7                           # 더 촘촘한 그리드
grid_y: 7

# 빠른 움직임
track_frequency: 30.0               # 증가
min_px_dist: 8                      # 감소
```

### 7.5 시간 오프셋 이상

**증상**: `timeshift_cam_imu` 값이 비정상적

**정상 범위**: -0.05초 ~ +0.05초

```yaml
# 초기값 조정
timeshift_cam_imu: 0.02             # 20ms로 시작

# 온라인 추정 활성화 확인
calib_cam_timeoffset: true

# 데이터 품질 확인
# - 카메라: 최소 15Hz, 권장 20-30Hz
# - IMU: 최소 100Hz, 권장 200Hz
```

---

## 8. VINS-Mono vs OpenVINS 비교

| 항목 | VINS-Mono | OpenVINS |
|------|-----------|----------|
| **필터 방식** | 최적화 (Ceres) | MSCKF (EKF) |
| **CPU 사용량** | 중간 | **낮음** |
| **메모리 사용량** | 중간 | **낮음** |
| **정확도** | 약간 높음 | 높음 |
| **초기화** | 자체 구현 | 정적/동적 모두 지원 |
| **Loop Closure** | **지원** | 미지원 |
| **Rolling Shutter** | **지원** | 미지원 |
| **온라인 캘리브레이션** | 외부+시간 | **외부+내부+시간+IMU** |
| **문서화** | 중간 | **최고** |
| **Kalibr 호환** | 부분 | **완전** |

**스마트폰 선택 가이드**:
- **VINS-Mono 선택**: Loop Closure 필요, Rolling Shutter 보정 필요
- **OpenVINS 선택**: 저사양 PC, 포괄적 캘리브레이션 필요, 연구/학습 목적

---

## 9. 참고 자료

### 공식 문서
- [OpenVINS Documentation](https://docs.openvins.com/)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [OpenVINS Tutorial](https://docs.openvins.com/gs-tutorial.html)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)

### 캘리브레이션 도구
- [Kalibr](https://github.com/ethz-asl/kalibr)
- [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)

### 논문
- [OpenVINS: A Research Platform for Visual-Inertial Estimation](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)
- [OpenVINS State Initialization](https://pgeneva.com/downloads/reports/tr_init.pdf)

### 관련 문서
- [05_vislam_frameworks.md](05_vislam_frameworks.md)
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [12_smartphone_imu_drift.md](12_smartphone_imu_drift.md)
- [14_vins_mono_smartphone_config.md](14_vins_mono_smartphone_config.md)
