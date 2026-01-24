# ORB-SLAM3 스마트폰 설정 가이드

## 개요

ORB-SLAM3은 Visual, Visual-Inertial, Multi-Map SLAM을 지원하는 가장 정확한 오픈소스 SLAM 라이브러리 중 하나입니다. 이 문서에서는 스마트폰 카메라와 IMU 데이터를 사용하여 ORB-SLAM3의 Mono-Inertial 모드를 설정하는 방법을 설명합니다.

### ORB-SLAM3 특징

| 항목 | 내용 |
|------|------|
| **지원 모드** | Mono, Stereo, RGB-D, Mono-Inertial, Stereo-Inertial |
| **렌즈 모델** | Pinhole, Fisheye |
| **장점** | 최고 정확도, Loop Closure, Multi-Map 지원 |
| **단점** | 시간 오프셋 자동 추정 미지원, Rolling Shutter 미지원 |

### 스마트폰 사용 시 제약사항

⚠️ **중요**: ORB-SLAM3은 VINS-Mono/OpenVINS와 달리 **카메라-IMU 시간 오프셋을 자동 추정하지 않습니다**. 따라서:

1. **정확한 캘리브레이션 필수**: Kalibr 등으로 사전에 정확한 시간 오프셋 측정 필요
2. **데이터 전처리**: 측정된 오프셋을 기반으로 타임스탬프 보정 후 입력
3. **Rolling Shutter 미지원**: Global Shutter 카메라 또는 느린 움직임 권장

---

## 설정 파일 구조

ORB-SLAM3의 설정은 단일 YAML 파일로 구성됩니다.

```yaml
# smartphone_mono_inertial.yaml

%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters (스마트폰 카메라 파라미터)
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

# 카메라 Intrinsic 파라미터 (Kalibr 또는 OpenCV로 캘리브레이션)
Camera1.fx: 500.0
Camera1.fy: 500.0
Camera1.cx: 320.0
Camera1.cy: 240.0

# 왜곡 계수 (Radial-Tangential)
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

# 이미지 크기
Camera.width: 640
Camera.height: 480

# Rectification을 위한 새 이미지 크기 (왜곡 제거 후)
Camera.newWidth: 640
Camera.newHeight: 480

# 카메라 프레임 레이트
Camera.fps: 30

# RGB 이미지 여부 (0: BGR, 1: RGB)
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters (스마트폰 IMU 파라미터)
#--------------------------------------------------------------------------------------------

# IMU-Camera 변환 행렬 (Tbc: Camera frame -> Body/IMU frame)
# 4x4 Homogeneous Transformation Matrix in SE(3)
# Kalibr의 T_cam_imu 출력을 사용
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0,  0.0,  1.0,  0.0,
          -1.0,  0.0,  0.0,  0.0,
           0.0, -1.0,  0.0,  0.0,
           0.0,  0.0,  0.0,  1.0]

# IMU 노이즈 파라미터 (연속 시간 기준)
# 단위: Gyro [rad/s/sqrt(Hz)], Acc [m/s^2/sqrt(Hz)]
IMU.NoiseGyro: 1.7e-4      # 자이로스코프 백색 잡음
IMU.NoiseAcc: 2.0e-3       # 가속도계 백색 잡음

# IMU Random Walk 파라미터
# 단위: Gyro [rad/s^2/sqrt(Hz)], Acc [m/s^3/sqrt(Hz)]
IMU.GyroWalk: 1.9e-5       # 자이로스코프 바이어스 랜덤 워크
IMU.AccWalk: 3.0e-3        # 가속도계 바이어스 랜덤 워크

# IMU 샘플링 주파수 [Hz]
IMU.Frequency: 200.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters (특징점 추출)
#--------------------------------------------------------------------------------------------

# 프레임당 추출할 ORB 특징점 수
ORBextractor.nFeatures: 1200

# 이미지 피라미드 스케일 팩터
ORBextractor.scaleFactor: 1.2

# 이미지 피라미드 레벨 수
ORBextractor.nLevels: 8

# FAST 특징점 검출 임계값
ORBextractor.iniThFAST: 20    # 초기 임계값
ORBextractor.minThFAST: 7     # 최소 임계값 (특징점 부족 시 사용)

#--------------------------------------------------------------------------------------------
# Viewer Parameters (시각화)
#--------------------------------------------------------------------------------------------

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

---

## 스마트폰별 IMU 노이즈 파라미터

### 권장 노이즈 파라미터

스마트폰 MEMS IMU는 산업용 IMU보다 노이즈가 크므로, 제조사 스펙보다 **10배 이상 큰 값**을 사용하는 것이 일반적입니다.

| 스마트폰 등급 | NoiseGyro | NoiseAcc | GyroWalk | AccWalk |
|--------------|-----------|----------|----------|---------|
| **플래그십** (iPhone 15, Galaxy S24) | 1.0e-3 | 1.5e-2 | 1.0e-4 | 3.0e-3 |
| **미드레인지** | 2.0e-3 | 3.0e-2 | 2.0e-4 | 5.0e-3 |
| **보급형** | 5.0e-3 | 5.0e-2 | 5.0e-4 | 1.0e-2 |

### 파라미터 튜닝 가이드

```yaml
# 플래그십 스마트폰 권장 설정
IMU.NoiseGyro: 1.0e-3
IMU.NoiseAcc: 1.5e-2
IMU.GyroWalk: 1.0e-4
IMU.AccWalk: 3.0e-3
IMU.Frequency: 200.0

# 미드레인지 스마트폰 권장 설정
IMU.NoiseGyro: 2.0e-3
IMU.NoiseAcc: 3.0e-2
IMU.GyroWalk: 2.0e-4
IMU.AccWalk: 5.0e-3
IMU.Frequency: 100.0
```

💡 **팁**: ORB-SLAM3 초기화가 잘 안 되면 `GyroWalk`와 `AccWalk`를 2-10배 증가시켜 보세요.

---

## 스마트폰 카메라 설정

### Pinhole 모델 (일반 스마트폰)

```yaml
Camera.type: "PinHole"

# 일반적인 스마트폰 카메라 파라미터 예시 (1080p 기준)
Camera1.fx: 1000.0
Camera1.fy: 1000.0
Camera1.cx: 540.0
Camera1.cy: 960.0

# 왜곡 계수 (Kalibr 캘리브레이션 결과 사용)
Camera1.k1: -0.1
Camera1.k2: 0.05
Camera1.p1: 0.001
Camera1.p2: -0.001

Camera.width: 1080
Camera.height: 1920
Camera.fps: 30
```

### Fisheye 모델 (광각 카메라)

일부 스마트폰의 광각 카메라는 Fisheye 모델 사용:

```yaml
Camera.type: "KannalaBrandt8"

# Fisheye 왜곡 계수
Camera1.k1: 0.5
Camera1.k2: -0.1
Camera1.k3: 0.05
Camera1.k4: -0.01
```

---

## IMU-Camera 변환 행렬 (Tbc)

### 개념 설명

`IMU.T_b_c1`은 카메라 좌표계에서 Body(IMU) 좌표계로의 변환 행렬입니다.

```
좌표계 정의:
- Camera (C): zC가 광축 방향(전방), yC가 아래, xC가 오른쪽
- Body/IMU (B): 센서 제조사 정의 따름
```

### Kalibr에서 Tbc 얻기

```bash
# Kalibr 캘리브레이션 실행
kalibr_calibrate_imu_camera \
    --target april_grid.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calibration.bag

# 결과 파일에서 T_cam_imu 확인
# Kalibr 출력: T_cam_imu (Camera <- IMU)
# ORB-SLAM3 필요: T_b_c1 = T_imu_cam = inv(T_cam_imu)
```

### 변환 행렬 변환

Kalibr 출력을 ORB-SLAM3 형식으로 변환:

```python
import numpy as np

# Kalibr 출력 (T_cam_imu)
T_cam_imu = np.array([
    [r11, r12, r13, tx],
    [r21, r22, r23, ty],
    [r31, r32, r33, tz],
    [0,   0,   0,   1]
])

# ORB-SLAM3 필요 (T_b_c1 = T_imu_cam)
T_b_c1 = np.linalg.inv(T_cam_imu)

# YAML 형식으로 출력
print("IMU.T_b_c1: !!opencv-matrix")
print("   rows: 4")
print("   cols: 4")
print("   dt: f")
print(f"   data: {T_b_c1.flatten().tolist()}")
```

### 일반적인 스마트폰 Tbc 예시

```yaml
# 후면 카메라 + IMU (일반적인 배치)
# 카메라가 세로 모드로 장착된 경우
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, -1.0,  0.0,  0.0,
           1.0,  0.0,  0.0,  0.0,
           0.0,  0.0,  1.0,  0.0,
           0.0,  0.0,  0.0,  1.0]
```

---

## ORB 특징점 추출 최적화

### 스마트폰 환경 권장 설정

```yaml
# 일반 환경 (실내/실외)
ORBextractor.nFeatures: 1200      # 충분한 특징점 확보
ORBextractor.scaleFactor: 1.2     # 표준 스케일 팩터
ORBextractor.nLevels: 8           # 다양한 스케일 대응

# FAST 임계값 (텍스처가 적은 환경에서 낮춤)
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

### 환경별 파라미터 조정

| 환경 | nFeatures | iniThFAST | minThFAST | 설명 |
|------|-----------|-----------|-----------|------|
| **일반** | 1200 | 20 | 7 | 기본 설정 |
| **텍스처 부족** | 1500 | 15 | 5 | 특징점 감지 민감도 증가 |
| **빠른 동작** | 1500 | 20 | 7 | 더 많은 특징점으로 추적 안정성 확보 |
| **저조도** | 1000 | 12 | 5 | 노이즈 환경에서 임계값 낮춤 |

---

## 시간 동기화 처리

### ⚠️ 핵심 주의사항

ORB-SLAM3은 **시간 오프셋 자동 추정을 지원하지 않습니다**. 따라서 다음 절차가 필수입니다:

### 1. Kalibr로 시간 오프셋 측정

```bash
# imu.yaml에 시간 오프셋 초기값 설정
# timeshift_cam_imu: 0.0  # [s] 초기값

kalibr_calibrate_imu_camera \
    --target april_grid.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calibration.bag \
    --time-calibration  # 시간 오프셋도 추정
```

### 2. 타임스탬프 사전 보정

```python
import numpy as np

class TimestampCorrector:
    def __init__(self, time_offset_cam_imu):
        """
        time_offset_cam_imu: t_imu = t_cam + time_offset
        """
        self.offset = time_offset_cam_imu

    def correct_image_timestamp(self, img_timestamp):
        """이미지 타임스탬프를 IMU 시간 기준으로 보정"""
        return img_timestamp + self.offset

    def prepare_data_for_orbslam3(self, images, imus, time_offset):
        """ORB-SLAM3 입력 전 데이터 전처리"""
        corrector = TimestampCorrector(time_offset)

        corrected_images = []
        for img_ts, img_data in images:
            corrected_ts = corrector.correct_image_timestamp(img_ts)
            corrected_images.append((corrected_ts, img_data))

        return corrected_images, imus  # IMU는 그대로
```

### 3. 데이터 입력 시 동기화

```cpp
// ORB-SLAM3에 데이터 전달 시 타임스탬프가 정렬되어야 함
// IMU 데이터는 이미지 타임스탬프 이전 것들을 모두 전달

// 주의: 이미지 타임스탬프는 IMU 타임스탬프보다 작아야 함
// ERROR: "Frame with a timestamp older than previous frame detected!"
// 이 에러가 발생하면 타임스탬프 순서 확인 필요
```

---

## 캘리브레이션 워크플로우

### 전체 프로세스

```
1. 카메라 Intrinsic 캘리브레이션
   └── OpenCV 또는 Kalibr 사용

2. IMU Intrinsic 캘리브레이션
   └── imu_utils 또는 Allan Variance 분석

3. Camera-IMU Extrinsic 캘리브레이션
   └── Kalibr kalibr_calibrate_imu_camera

4. 시간 오프셋 측정
   └── Kalibr --time-calibration

5. 설정 파일 생성 및 검증
```

### Kalibr 캘리브레이션 파일

**april_grid.yaml** (캘리브레이션 타겟):
```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088      # 태그 크기 [m]
tagSpacing: 0.3     # 태그 간격 비율
```

**cam.yaml** (카메라 초기 설정):
```yaml
cam0:
  camera_model: pinhole
  intrinsics: [500, 500, 320, 240]  # fx, fy, cx, cy
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [640, 480]
  rostopic: /camera/image_raw
```

**imu.yaml** (IMU 설정):
```yaml
imu0:
  accelerometer_noise_density: 0.015      # m/s^2/sqrt(Hz)
  accelerometer_random_walk: 0.003        # m/s^3/sqrt(Hz)
  gyroscope_noise_density: 0.001          # rad/s/sqrt(Hz)
  gyroscope_random_walk: 0.0001           # rad/s^2/sqrt(Hz)
  rostopic: /imu/data
  update_rate: 200.0
```

---

## 실행 방법

### ROS 환경

```bash
# 1. ORB-SLAM3 빌드
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# 2. ROS 워크스페이스 빌드
cd Examples/ROS/ORB_SLAM3
mkdir build && cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4

# 3. 실행
rosrun ORB_SLAM3 Mono_Inertial \
    /path/to/ORBvoc.txt \
    /path/to/smartphone_mono_inertial.yaml \
    true  # Visualization
```

### 데이터 입력 토픽

```
/camera/image_raw     - 이미지 (sensor_msgs/Image)
/imu/data            - IMU (sensor_msgs/Imu)
```

---

## Android 앱 연동

### ORB-SLAM3 AR Android 프로젝트

[ORB_SLAM3_AR-for-Android](https://github.com/Abonaventure/ORB_SLAM3_AR-for-Android) 참고:

```
필요 파일:
/sdcard/SLAM/
├── ORBvoc.bin           # Vocabulary (Binary 형식)
└── PARAconfig.yaml      # 카메라/IMU 설정 파일
```

### 주의사항

- 스마트폰별 카메라 캘리브레이션 필수
- 텍스처가 풍부한 환경에서 초기화
- 초기화에 시간이 걸릴 수 있음

---

## 트러블슈팅

### 1. IMU 초기화 실패

**증상**: "IMU initialization failed" 또는 느린 수렴

**원인 및 해결**:
```yaml
# 1. Random Walk 값 증가 (10배)
IMU.GyroWalk: 1.0e-3    # 기존 대비 10배
IMU.AccWalk: 3.0e-2     # 기존 대비 10배

# 2. IMU 주파수 확인
IMU.Frequency: 200.0    # 실제 수집 주파수와 일치해야 함

# 3. 초기화 시 충분한 움직임 제공
# - 다양한 방향으로 회전
# - 2-3초간 움직인 후 잠시 정지
```

### 2. "Frame with timestamp older than previous frame"

**원인**: 타임스탬프 순서 불일치

**해결**:
```python
# 타임스탬프 정렬 확인
def check_timestamp_order(images, imus):
    prev_img_ts = 0
    for img_ts, _ in images:
        if img_ts < prev_img_ts:
            print(f"ERROR: Image timestamp disorder at {img_ts}")
        prev_img_ts = img_ts

    prev_imu_ts = 0
    for imu_ts, _ in imus:
        if imu_ts < prev_imu_ts:
            print(f"ERROR: IMU timestamp disorder at {imu_ts}")
        prev_imu_ts = imu_ts
```

### 3. 드리프트 심함

**원인**: IMU 파라미터 불일치 또는 캘리브레이션 오차

**해결**:
```yaml
# 1. IMU 노이즈 재측정
# Allan Variance 분석으로 실제 노이즈 특성 파악

# 2. Tbc 행렬 검증
# Kalibr 재캘리브레이션 또는 수동 조정

# 3. 시간 오프셋 재확인
# 정확한 시간 동기화가 매우 중요
```

### 4. 특징점 추적 실패

**증상**: 트래킹 손실이 빈번함

**해결**:
```yaml
# 특징점 수 증가
ORBextractor.nFeatures: 1500

# FAST 임계값 감소 (더 많은 특징점 검출)
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

# 이미지 해상도 확인
# 너무 낮으면 특징점 품질 저하
```

### 5. Segmentation Fault

**원인**: 잘못된 설정 파일 형식

**해결**:
```yaml
# YAML 형식 검증
# opencv-matrix 형식 정확히 지켜야 함

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f      # float 타입
   data: [...]  # 16개 요소
```

---

## VINS-Mono vs ORB-SLAM3 비교

| 항목 | VINS-Mono | ORB-SLAM3 |
|------|-----------|-----------|
| **시간 오프셋 자동 추정** | ✅ 지원 | ❌ 미지원 |
| **Rolling Shutter** | ✅ 지원 | ❌ 미지원 |
| **Loop Closure** | ✅ 지원 | ✅ 지원 (더 강력) |
| **Multi-Map** | ❌ 미지원 | ✅ 지원 |
| **정확도** | 높음 | 최고 수준 |
| **스마트폰 호환성** | 매우 좋음 | 보통 (캘리브레이션 중요) |

### 스마트폰 사용 시 권장 선택

| 상황 | 권장 프레임워크 |
|------|---------------|
| **정확한 캘리브레이션 가능** | ORB-SLAM3 |
| **빠른 프로토타이핑** | VINS-Mono |
| **Rolling Shutter 카메라** | VINS-Mono |
| **최고 정확도 필요** | ORB-SLAM3 |
| **오프라인 처리** | ORB-SLAM3 |

---

## 전체 설정 파일 템플릿

### 플래그십 스마트폰용

<details>
<summary>smartphone_flagship.yaml (클릭하여 펼치기)</summary>

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters - Flagship Smartphone (e.g., iPhone 15, Galaxy S24)
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

Camera1.fx: 1500.0
Camera1.fy: 1500.0
Camera1.cx: 540.0
Camera1.cy: 960.0

Camera1.k1: -0.1
Camera1.k2: 0.05
Camera1.p1: 0.001
Camera1.p2: -0.001

Camera.width: 1080
Camera.height: 1920
Camera.newWidth: 1080
Camera.newHeight: 1920
Camera.fps: 30
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters - Flagship Grade MEMS IMU
#--------------------------------------------------------------------------------------------

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, -1.0,  0.0,  0.0,
           1.0,  0.0,  0.0,  0.0,
           0.0,  0.0,  1.0,  0.0,
           0.0,  0.0,  0.0,  1.0]

IMU.NoiseGyro: 1.0e-3
IMU.NoiseAcc: 1.5e-2
IMU.GyroWalk: 1.0e-4
IMU.AccWalk: 3.0e-3
IMU.Frequency: 200.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters
#--------------------------------------------------------------------------------------------

ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

</details>

### 미드레인지 스마트폰용

<details>
<summary>smartphone_midrange.yaml (클릭하여 펼치기)</summary>

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters - Mid-range Smartphone
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

Camera1.fx: 800.0
Camera1.fy: 800.0
Camera1.cx: 320.0
Camera1.cy: 240.0

Camera1.k1: -0.15
Camera1.k2: 0.08
Camera1.p1: 0.002
Camera1.p2: -0.001

Camera.width: 640
Camera.height: 480
Camera.newWidth: 640
Camera.newHeight: 480
Camera.fps: 30
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters - Mid-range MEMS IMU (Higher noise)
#--------------------------------------------------------------------------------------------

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, -1.0,  0.0,  0.0,
           1.0,  0.0,  0.0,  0.0,
           0.0,  0.0,  1.0,  0.0,
           0.0,  0.0,  0.0,  1.0]

IMU.NoiseGyro: 2.0e-3
IMU.NoiseAcc: 3.0e-2
IMU.GyroWalk: 2.0e-4
IMU.AccWalk: 5.0e-3
IMU.Frequency: 100.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters (More features for lower quality images)
#--------------------------------------------------------------------------------------------

ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
```

</details>

---

## 참고 자료

### 공식 문서 및 코드
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ORB-SLAM3 논문](https://arxiv.org/abs/2007.11898)
- [Calibration Tutorial PDF](http://git.autolabor.com.cn/12345qiupeng/orb_slam3_details/raw/commit/e6e28a86a5e35de35fb3022c881328b88f2bacd2/Calibration_Tutorial.pdf)

### 캘리브레이션 도구
- [Kalibr](https://github.com/ethz-asl/kalibr) - Camera-IMU 캘리브레이션
- [imu_utils](https://github.com/gaowenliang/imu_utils) - IMU Allan Variance 분석

### 관련 프로젝트
- [ORB_SLAM3_AR-for-Android](https://github.com/Abonaventure/ORB_SLAM3_AR-for-Android) - Android AR 구현
- [Camera IMU time offset Issue #78](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/78) - 시간 오프셋 관련 논의

---

*문서 생성일: 2026-01-19*
*버전: 1.0.0*
