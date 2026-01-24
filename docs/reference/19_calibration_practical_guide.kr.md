# 캘리브레이션 실습 가이드

## 개요

이 문서는 스마트폰 카메라와 IMU를 VI-SLAM에 사용하기 위한 캘리브레이션 실습 가이드입니다. 카메라 Intrinsic, IMU 노이즈 파라미터, Camera-IMU Extrinsic 캘리브레이션의 전체 워크플로우를 단계별로 설명합니다.

### 캘리브레이션 전체 흐름

```
┌─────────────────────────────────────────────────────────────────┐
│                    캘리브레이션 워크플로우                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1단계: 캘리브레이션 타겟 준비                                     │
│         └─ AprilGrid 또는 체커보드 출력                           │
│                    ↓                                            │
│  2단계: 카메라 Intrinsic 캘리브레이션                              │
│         └─ OpenCV 또는 Kalibr 사용                               │
│                    ↓                                            │
│  3단계: IMU 노이즈 파라미터 측정                                   │
│         └─ Allan Variance 분석 (imu_utils)                       │
│                    ↓                                            │
│  4단계: Camera-IMU Extrinsic 캘리브레이션                         │
│         └─ Kalibr kalibr_calibrate_imu_camera                   │
│                    ↓                                            │
│  5단계: 결과 검증 및 프레임워크 적용                                │
│         └─ 설정 파일 생성 및 테스트                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 1단계: 캘리브레이션 타겟 준비

### 타겟 종류 비교

| 타겟 종류 | 장점 | 단점 | 권장 사용 |
|----------|------|------|----------|
| **AprilGrid** | 부분 가시성 OK, 포즈 플립 없음 | 복잡한 감지 | ✅ Kalibr 권장 |
| **Checkerboard** | 간단, OpenCV 기본 지원 | 대칭성으로 플립 발생 가능 | OpenCV 단독 사용 시 |
| **ChArUco** | AprilTag + Checkerboard 장점 | 설정 복잡 | 고정밀 필요 시 |

### AprilGrid 생성 및 출력

#### Kalibr로 AprilGrid 생성

```bash
# 6x6 AprilGrid 생성 (A0 사이즈 권장)
kalibr_create_target_pdf \
    --type apriltag \
    --nx 6 \
    --ny 6 \
    --tsize 0.088 \
    --tspace 0.3 \
    --output aprilgrid_6x6.pdf
```

**파라미터 설명:**
- `--nx`, `--ny`: 태그 열/행 수
- `--tsize`: 태그 크기 (미터)
- `--tspace`: 태그 간격 비율 (tsize 대비)

#### AprilGrid 설정 파일 (aprilgrid.yaml)

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088        # 태그 크기 [m] - 실제 출력 후 측정값으로 수정
tagSpacing: 0.3       # 간격 비율 (space = tagSize * tagSpacing)
```

### 체커보드 생성

#### Kalibr로 체커보드 생성

```bash
kalibr_create_target_pdf \
    --type checkerboard \
    --nx 9 \
    --ny 6 \
    --csx 0.03 \
    --csy 0.03 \
    --output checkerboard_9x6.pdf
```

#### 체커보드 설정 파일 (checkerboard.yaml)

```yaml
target_type: 'checkerboard'
targetCols: 9          # 내부 코너 열 수
targetRows: 6          # 내부 코너 행 수
rowSpacingMeters: 0.03 # 행 간격 [m]
colSpacingMeters: 0.03 # 열 간격 [m]
```

### 출력 및 설치 팁

1. **정확한 출력**
   - A0 또는 A1 사이즈 권장
   - "실제 크기로 인쇄" 옵션 선택
   - 출력 후 **실제 크기 측정** 필수

2. **견고한 설치**
   - 평평하고 단단한 보드에 부착
   - 구김이나 휘어짐 방지
   - 폼보드, 아크릴, 알루미늄 보드 권장

3. **크기 측정**
   ```python
   # 실제 측정값으로 설정 파일 수정
   measured_tag_size = 0.0875  # 실측값 [m]
   ```

---

## 2단계: 카메라 Intrinsic 캘리브레이션

### 방법 1: OpenCV 사용 (간단, 빠름)

#### Python 캘리브레이션 스크립트

```python
#!/usr/bin/env python3
"""
camera_calibration_opencv.py
스마트폰 카메라 Intrinsic 캘리브레이션
"""

import cv2
import numpy as np
import glob
import yaml
import os

class CameraCalibrator:
    def __init__(self, checkerboard_size=(9, 6), square_size=0.03):
        """
        Args:
            checkerboard_size: (cols, rows) 내부 코너 수
            square_size: 정사각형 한 변 크기 [m]
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size

        # 3D 월드 좌표 준비
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3),
                             np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0],
                                     0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size

        self.obj_points = []  # 3D 포인트
        self.img_points = []  # 2D 포인트
        self.image_size = None

    def add_images(self, image_paths, show_corners=False):
        """이미지에서 체커보드 코너 검출"""
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001)

        successful = 0
        for path in image_paths:
            img = cv2.imread(path)
            if img is None:
                print(f"Cannot read: {path}")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if self.image_size is None:
                self.image_size = gray.shape[::-1]

            # 코너 찾기
            ret, corners = cv2.findChessboardCorners(
                gray, self.checkerboard_size, None)

            if ret:
                # 서브픽셀 정확도로 코너 개선
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)

                self.obj_points.append(self.objp)
                self.img_points.append(corners2)
                successful += 1

                if show_corners:
                    cv2.drawChessboardCorners(
                        img, self.checkerboard_size, corners2, ret)
                    cv2.imshow('Corners', cv2.resize(img, (640, 480)))
                    cv2.waitKey(500)

        print(f"Detected corners in {successful}/{len(image_paths)} images")
        return successful

    def calibrate(self):
        """캘리브레이션 실행"""
        if len(self.obj_points) < 10:
            raise ValueError("Need at least 10 images with detected corners")

        print("Calibrating...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            self.image_size,
            None,
            None
        )

        # 재투영 오차 계산
        total_error = 0
        for i in range(len(self.obj_points)):
            img_points2, _ = cv2.projectPoints(
                self.obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.img_points[i], img_points2, cv2.NORM_L2)
            total_error += error ** 2

        mean_error = np.sqrt(total_error / len(self.obj_points))

        result = {
            'camera_matrix': mtx,
            'dist_coeffs': dist,
            'image_size': self.image_size,
            'reprojection_error': mean_error,
            'num_images': len(self.obj_points)
        }

        print(f"\nCalibration Results:")
        print(f"  fx: {mtx[0, 0]:.2f}")
        print(f"  fy: {mtx[1, 1]:.2f}")
        print(f"  cx: {mtx[0, 2]:.2f}")
        print(f"  cy: {mtx[1, 2]:.2f}")
        print(f"  k1: {dist[0, 0]:.6f}")
        print(f"  k2: {dist[0, 1]:.6f}")
        print(f"  p1: {dist[0, 2]:.6f}")
        print(f"  p2: {dist[0, 3]:.6f}")
        print(f"  Reprojection Error: {mean_error:.4f} pixels")

        return result

    def save_kalibr_format(self, result, output_path, topic_name="/cam0/image_raw"):
        """Kalibr YAML 형식으로 저장"""
        mtx = result['camera_matrix']
        dist = result['dist_coeffs'].flatten()

        data = {
            'cam0': {
                'camera_model': 'pinhole',
                'intrinsics': [
                    float(mtx[0, 0]),  # fx
                    float(mtx[1, 1]),  # fy
                    float(mtx[0, 2]),  # cx
                    float(mtx[1, 2])   # cy
                ],
                'distortion_model': 'radtan',
                'distortion_coeffs': [
                    float(dist[0]),  # k1
                    float(dist[1]),  # k2
                    float(dist[2]),  # p1
                    float(dist[3])   # p2
                ],
                'resolution': list(result['image_size']),
                'rostopic': topic_name
            }
        }

        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"\nSaved to {output_path}")

    def save_vins_format(self, result, output_path):
        """VINS-Mono 형식으로 저장"""
        mtx = result['camera_matrix']
        dist = result['dist_coeffs'].flatten()

        content = f"""# Camera Intrinsic Parameters (VINS-Mono format)
# Calibrated with OpenCV

model_type: PINHOLE
camera_name: smartphone_camera
image_width: {result['image_size'][0]}
image_height: {result['image_size'][1]}

distortion_parameters:
   k1: {dist[0]:.10f}
   k2: {dist[1]:.10f}
   p1: {dist[2]:.10f}
   p2: {dist[3]:.10f}

projection_parameters:
   fx: {mtx[0, 0]:.10f}
   fy: {mtx[1, 1]:.10f}
   cx: {mtx[0, 2]:.10f}
   cy: {mtx[1, 2]:.10f}
"""

        with open(output_path, 'w') as f:
            f.write(content)

        print(f"Saved VINS format to {output_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Camera Calibration')
    parser.add_argument('--images', required=True, help='Image directory or glob pattern')
    parser.add_argument('--cols', type=int, default=9, help='Checkerboard columns')
    parser.add_argument('--rows', type=int, default=6, help='Checkerboard rows')
    parser.add_argument('--size', type=float, default=0.03, help='Square size in meters')
    parser.add_argument('--output', default='camera_calib.yaml', help='Output file')
    parser.add_argument('--show', action='store_true', help='Show detected corners')
    args = parser.parse_args()

    # 이미지 파일 찾기
    if os.path.isdir(args.images):
        patterns = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
        image_paths = []
        for pattern in patterns:
            image_paths.extend(glob.glob(os.path.join(args.images, pattern)))
    else:
        image_paths = glob.glob(args.images)

    if not image_paths:
        print("No images found!")
        return

    print(f"Found {len(image_paths)} images")

    # 캘리브레이션 실행
    calibrator = CameraCalibrator(
        checkerboard_size=(args.cols, args.rows),
        square_size=args.size
    )

    calibrator.add_images(image_paths, show_corners=args.show)
    result = calibrator.calibrate()

    # 저장
    calibrator.save_kalibr_format(result, args.output)
    calibrator.save_vins_format(result, args.output.replace('.yaml', '_vins.yaml'))

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
```

#### 사용 방법

```bash
# 1. 체커보드 이미지 촬영 (최소 15-20장)
#    - 다양한 각도와 거리에서 촬영
#    - 전체 이미지 영역을 커버

# 2. 캘리브레이션 실행
python camera_calibration_opencv.py \
    --images ./calib_images/ \
    --cols 9 \
    --rows 6 \
    --size 0.03 \
    --output camera_calib.yaml \
    --show
```

### 방법 2: Kalibr 사용 (정밀, 권장)

#### ROS bag 녹화

```bash
# 스마트폰 카메라 토픽을 ROS bag으로 녹화
rosbag record -O camera_calib.bag /camera/image_raw

# 녹화 팁:
# - 20Hz 권장
# - 타겟을 천천히 다양한 각도로 움직임
# - 1-2분 녹화
```

#### Kalibr 카메라 캘리브레이션

```bash
# 단일 카메라 캘리브레이션
kalibr_calibrate_cameras \
    --target aprilgrid.yaml \
    --bag camera_calib.bag \
    --models pinhole-radtan \
    --topics /camera/image_raw \
    --show-extraction

# 결과: camchain-camera_calib.yaml
```

### 이미지 촬영 가이드라인

| 항목 | 권장 사항 |
|------|----------|
| **이미지 수** | 15-30장 |
| **각도 변화** | 다양한 tilt (±45°) |
| **거리 변화** | 가까이 ~ 멀리 |
| **위치** | 이미지 전체 영역 커버 |
| **움직임** | 천천히, 흔들림 없이 |
| **조명** | 균일하고 충분한 밝기 |
| **포커스** | 선명하게 (흐림 없이) |

---

## 3단계: IMU 노이즈 파라미터 측정

### IMU 노이즈 모델

```
측정값 = 실제값 + 백색 잡음 + 바이어스
       = true + n(t) + b(t)

백색 잡음 (White Noise):
  - Gyroscope: σ_g [rad/s/√Hz]
  - Accelerometer: σ_a [m/s²/√Hz]

랜덤 워크 (Random Walk):
  - Gyroscope: σ_bg [rad/s²/√Hz]
  - Accelerometer: σ_ba [m/s³/√Hz]
```

### 방법 1: Allan Variance 분석 (정밀)

#### imu_utils 설치 및 사용

```bash
# 1. imu_utils 설치
cd ~/catkin_ws/src
git clone https://github.com/gaowenliang/imu_utils.git
git clone https://github.com/gaowenliang/code_utils.git
cd ..
catkin_make

# 2. IMU 데이터 수집 (정지 상태에서 2시간 이상)
# 스마트폰을 진동 없는 평평한 곳에 놓고 녹화
rosbag record -O imu_static.bag /imu/data --duration=7200

# 3. launch 파일 생성 (imu_analysis.launch)
```

**imu_analysis.launch:**
```xml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" value="/imu/data"/>
        <param name="imu_name" value="smartphone_imu"/>
        <param name="data_save_path" value="$(find imu_utils)/data/"/>
        <param name="max_time_min" value="120"/>  <!-- 분 단위 -->
        <param name="max_cluster" value="100"/>
    </node>
</launch>
```

```bash
# 4. 분석 실행
roslaunch imu_utils imu_analysis.launch
rosbag play imu_static.bag

# 5. 결과 확인
# data/smartphone_imu_imu_param.yaml에 결과 저장
```

#### kalibr_allan 사용 (대안)

```bash
# 1. 설치
cd ~/catkin_ws/src
git clone https://github.com/rpng/kalibr_allan.git
cd ..
catkin_make

# 2. 분석 실행
rosrun kalibr_allan allan_variance /imu/data imu_static.bag

# 3. MATLAB/Python으로 결과 플롯
```

#### Allan Variance 결과 해석

```
Allan Deviation Plot:
                    │
    σ(τ)            │    /
    [단위]          │   / ← Random Walk (기울기 +1/2)
                    │  /
                    │ ●← Bias Instability (최소점)
                    │  \
                    │   \ ← White Noise (기울기 -1/2)
                    │    \
                    └──────────────────→ τ [s]
                         (적분 시간)

White Noise (N): τ=1s에서의 값
Random Walk (K): 우측 +1/2 기울기 피팅
Bias Instability (B): 최소점
```

### 방법 2: 데이터시트 기반 추정 (빠름)

스마트폰 IMU는 정확한 데이터시트가 없는 경우가 많으므로, 일반적인 MEMS IMU 값을 참고합니다.

#### 스마트폰 등급별 권장값

```yaml
# 플래그십 스마트폰 (iPhone 15, Galaxy S24 등)
imu0:
  accelerometer_noise_density: 0.015      # [m/s²/√Hz]
  accelerometer_random_walk: 0.003        # [m/s³/√Hz]
  gyroscope_noise_density: 0.001          # [rad/s/√Hz]
  gyroscope_random_walk: 0.0001           # [rad/s²/√Hz]
  update_rate: 200.0

# 미드레인지 스마트폰
imu0:
  accelerometer_noise_density: 0.03
  accelerometer_random_walk: 0.005
  gyroscope_noise_density: 0.002
  gyroscope_random_walk: 0.0002
  update_rate: 100.0

# 보급형 스마트폰
imu0:
  accelerometer_noise_density: 0.05
  accelerometer_random_walk: 0.01
  gyroscope_noise_density: 0.005
  gyroscope_random_walk: 0.0005
  update_rate: 100.0
```

### IMU 설정 파일 (Kalibr 형식)

```yaml
# imu.yaml
imu0:
  accelerometer_noise_density: 0.015      # σ_a [m/s²/√Hz]
  accelerometer_random_walk: 0.003        # σ_ba [m/s³/√Hz]
  gyroscope_noise_density: 0.001          # σ_g [rad/s/√Hz]
  gyroscope_random_walk: 0.0001           # σ_bg [rad/s²/√Hz]
  rostopic: /imu/data
  update_rate: 200.0                      # [Hz]
```

### 실용적 팁

> **중요**: Allan Variance 분석은 정지 상태, 일정 온도에서 수행되므로 실제 운용 환경과 다릅니다.
> 따라서 측정된 값에 **안전 계수를 곱하는 것이 일반적**입니다:
> - Noise Density: **× 10**
> - Random Walk: **× 5-10**

---

## 4단계: Camera-IMU Extrinsic 캘리브레이션

### 데이터 수집

#### 데이터 수집 요구사항

| 항목 | 권장 값 |
|------|--------|
| **카메라 프레임 레이트** | 20 Hz |
| **IMU 샘플링 레이트** | 200 Hz |
| **녹화 시간** | 60-120초 |
| **움직임** | 모든 축 회전 + 병진 |

#### 움직임 패턴

```
권장 움직임 순서:

1. X축 회전 (Roll): 좌우로 3회
2. Y축 회전 (Pitch): 위아래로 3회
3. Z축 회전 (Yaw): 시계/반시계로 3회
4. 8자 움직임: 2-3회
5. 복합 움직임: 자유롭게 10-15초

주의사항:
- 모션 블러 최소화 (천천히 움직임)
- 타겟이 항상 시야에 있도록
- IMU의 모든 축을 충분히 자극
```

#### ROS bag 녹화

```bash
# 카메라와 IMU 동시 녹화
rosbag record -O calib_imu_camera.bag \
    /camera/image_raw \
    /imu/data

# 참고: 스마트폰 앱에서 ROS 토픽으로 스트리밍 또는
# 녹화된 데이터를 ROS bag으로 변환
```

### Kalibr Camera-IMU 캘리브레이션 실행

```bash
kalibr_calibrate_imu_camera \
    --target aprilgrid.yaml \
    --cam camchain.yaml \
    --imu imu.yaml \
    --bag calib_imu_camera.bag \
    --show-extraction \
    --time-calibration

# 옵션 설명:
# --target: 캘리브레이션 타겟 설정
# --cam: 카메라 intrinsic (2단계 결과)
# --imu: IMU 노이즈 파라미터 (3단계 결과)
# --bag: 녹화 데이터
# --show-extraction: 코너 검출 시각화
# --time-calibration: 시간 오프셋도 추정
```

### 결과 파일 분석

#### camchain-imucam-*.yaml 구조

```yaml
cam0:
  T_cam_imu:
  - [r11, r12, r13, tx]
  - [r21, r22, r23, ty]
  - [r31, r32, r33, tz]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [k1, k2, p1, p2]
  distortion_model: radtan
  intrinsics: [fx, fy, cx, cy]
  resolution: [width, height]
  rostopic: /camera/image_raw
  timeshift_cam_imu: -0.00523    # 시간 오프셋 [s]
```

### 결과 검증

#### 재투영 오차 확인

```
좋은 결과 기준:
- 재투영 오차: < 0.5 pixels
- 시간 오프셋: 일반적으로 -10ms ~ +10ms
- T_cam_imu: 물리적으로 타당한 변환
```

#### PDF 리포트 분석

Kalibr는 자동으로 PDF 리포트를 생성합니다:
- 코너 검출 결과
- 재투영 오차 분포
- IMU 예측 vs 실제 비교
- 캘리브레이션 수렴 그래프

---

## 5단계: 프레임워크별 설정 파일 생성

### VINS-Mono 설정

```yaml
# config/smartphone/smartphone_config.yaml

%YAML:1.0

# Camera Intrinsics
model_type: PINHOLE
camera_name: smartphone
image_width: 640
image_height: 480

distortion_parameters:
   k1: -0.28340811
   k2: 0.07395907
   p1: 0.00019359
   p2: 1.76187114e-05

projection_parameters:
   fx: 458.654
   fy: 457.296
   cx: 367.215
   cy: 248.375

# Camera-IMU Extrinsic (from Kalibr T_cam_imu)
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [0.0148655, -0.9998809, 0.00414029, -0.0216401,
          0.9995572, 0.0149672, 0.02571552, -0.0646769,
          -0.0257744, 0.00375618, 0.9996607, 0.00981073,
          0, 0, 0, 1]

# IMU Parameters
acc_n: 0.015          # accelerometer noise density
gyr_n: 0.001          # gyroscope noise density
acc_w: 0.003          # accelerometer random walk
gyr_w: 0.0001         # gyroscope random walk

# Time offset
estimate_td: 1        # 온라인 추정 활성화
td: -0.00523          # 초기값 (Kalibr 결과)

# Extrinsic estimation
estimate_extrinsic: 1 # 온라인 미세 조정

# Rolling Shutter
rolling_shutter: 1
rolling_shutter_tr: 0.033  # 라인 읽기 시간 [s]
```

### OpenVINS 설정

```yaml
# config/smartphone/estimator_config.yaml

# Camera intrinsics
cam0_is_fisheye: false
cam0_k: [458.654, 457.296, 367.215, 248.375]
cam0_d: [-0.2834, 0.0739, 0.00019, 1.76e-05]
cam0_wh: [640, 480]

# Camera-IMU transform (T_imu_cam = inv(T_cam_imu))
T_imu_cam:
  - [0.0148655, 0.9995572, -0.0257744, 0.0652229]
  - [-0.9998809, 0.0149672, 0.00375618, 0.0207089]
  - [0.00414029, 0.02571552, 0.9996607, -0.0094768]
  - [0, 0, 0, 1]

# IMU intrinsics
gyroscope_noise_density: 0.001
accelerometer_noise_density: 0.015
gyroscope_random_walk: 0.0001
accelerometer_random_walk: 0.003

# Time offset calibration
calib_cam_timeoffset: true
calib_cam_extrinsics: true
```

### ORB-SLAM3 설정

```yaml
# smartphone.yaml

%YAML:1.0

Camera.type: "PinHole"

Camera1.fx: 458.654
Camera1.fy: 457.296
Camera1.cx: 367.215
Camera1.cy: 248.375

Camera1.k1: -0.28340811
Camera1.k2: 0.07395907
Camera1.p1: 0.00019359
Camera1.p2: 1.76187114e-05

Camera.width: 640
Camera.height: 480
Camera.fps: 30

# IMU-Camera Transform (Tbc = T_imu_cam)
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [0.0148655, 0.9995572, -0.0257744, 0.0652229,
          -0.9998809, 0.0149672, 0.00375618, 0.0207089,
          0.00414029, 0.02571552, 0.9996607, -0.0094768,
          0, 0, 0, 1]

# IMU Parameters (10x 증가 권장)
IMU.NoiseGyro: 0.01       # 0.001 * 10
IMU.NoiseAcc: 0.15        # 0.015 * 10
IMU.GyroWalk: 0.001       # 0.0001 * 10
IMU.AccWalk: 0.03         # 0.003 * 10
IMU.Frequency: 200.0

# ORB Extractor
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

### Basalt 설정

#### calibration.json

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.0652229,
        "py": 0.0207089,
        "pz": -0.0094768,
        "qx": 0.0128,
        "qy": -0.0074,
        "qz": 0.7071,
        "qw": 0.7071
      }
    ],
    "intrinsics": [
      {
        "camera_type": "pinhole",
        "intrinsics": {
          "fx": 458.654,
          "fy": 457.296,
          "cx": 367.215,
          "cy": 248.375
        }
      }
    ],
    "resolution": [[640, 480]],
    "imu_update_rate": 200.0,
    "accel_noise_std": [0.015, 0.015, 0.015],
    "gyro_noise_std": [0.001, 0.001, 0.001],
    "accel_bias_std": [0.003, 0.003, 0.003],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "cam_time_offset_ns": -5230000
  }
}
```

---

## 변환 유틸리티 스크립트

### Kalibr → 각 프레임워크 변환

```python
#!/usr/bin/env python3
"""
kalibr_converter.py
Kalibr 캘리브레이션 결과를 각 프레임워크 형식으로 변환
"""

import yaml
import json
import numpy as np
from scipy.spatial.transform import Rotation

def load_kalibr_result(camchain_path):
    """Kalibr camchain-imucam YAML 로드"""
    with open(camchain_path, 'r') as f:
        data = yaml.safe_load(f)
    return data

def get_T_imu_cam(T_cam_imu):
    """T_cam_imu -> T_imu_cam 변환"""
    T = np.array(T_cam_imu)
    return np.linalg.inv(T)

def matrix_to_quaternion(R):
    """회전 행렬을 쿼터니언으로 변환"""
    rot = Rotation.from_matrix(R)
    return rot.as_quat()  # [qx, qy, qz, qw]

def to_vins_mono(kalibr_data, imu_params, output_path):
    """VINS-Mono 형식으로 변환"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    intrinsics = cam['intrinsics']
    dist = cam['distortion_coeffs']
    resolution = cam['resolution']
    time_offset = cam.get('timeshift_cam_imu', 0.0)

    content = f"""%YAML:1.0

#common parameters
imu_topic: "/imu/data"
image_topic: "/camera/image_raw"
output_path: "./output/"

#camera calibration
model_type: PINHOLE
camera_name: smartphone
image_width: {resolution[0]}
image_height: {resolution[1]}

distortion_parameters:
   k1: {dist[0]:.10f}
   k2: {dist[1]:.10f}
   p1: {dist[2]:.10f}
   p2: {dist[3]:.10f}

projection_parameters:
   fx: {intrinsics[0]:.10f}
   fy: {intrinsics[1]:.10f}
   cx: {intrinsics[2]:.10f}
   cy: {intrinsics[3]:.10f}

# Extrinsic parameter between IMU and Camera
estimate_extrinsic: 1

body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [{T_cam_imu[0][0]}, {T_cam_imu[0][1]}, {T_cam_imu[0][2]}, {T_cam_imu[0][3]},
          {T_cam_imu[1][0]}, {T_cam_imu[1][1]}, {T_cam_imu[1][2]}, {T_cam_imu[1][3]},
          {T_cam_imu[2][0]}, {T_cam_imu[2][1]}, {T_cam_imu[2][2]}, {T_cam_imu[2][3]},
          0, 0, 0, 1]

#imu parameters
acc_n: {imu_params['accelerometer_noise_density']}
gyr_n: {imu_params['gyroscope_noise_density']}
acc_w: {imu_params['accelerometer_random_walk']}
gyr_w: {imu_params['gyroscope_random_walk']}
g_norm: 9.81007

#unsynchronized parameters
estimate_td: 1
td: {time_offset}

#rolling shutter parameters
rolling_shutter: 1
rolling_shutter_tr: 0.033

#visualization parameters
show_track: 1
"""

    with open(output_path, 'w') as f:
        f.write(content)
    print(f"Saved VINS-Mono config to {output_path}")

def to_openvins(kalibr_data, imu_params, output_path):
    """OpenVINS 형식으로 변환"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    T_imu_cam = np.linalg.inv(T_cam_imu)
    intrinsics = cam['intrinsics']
    dist = cam['distortion_coeffs']
    resolution = cam['resolution']

    content = f"""# OpenVINS Configuration

# Camera parameters
cam0_is_fisheye: false
cam0_k: [{intrinsics[0]}, {intrinsics[1]}, {intrinsics[2]}, {intrinsics[3]}]
cam0_d: [{dist[0]}, {dist[1]}, {dist[2]}, {dist[3]}]
cam0_wh: [{resolution[0]}, {resolution[1]}]

# IMU to Camera transform
T_imu_cam:
  - [{T_imu_cam[0][0]}, {T_imu_cam[0][1]}, {T_imu_cam[0][2]}, {T_imu_cam[0][3]}]
  - [{T_imu_cam[1][0]}, {T_imu_cam[1][1]}, {T_imu_cam[1][2]}, {T_imu_cam[1][3]}]
  - [{T_imu_cam[2][0]}, {T_imu_cam[2][1]}, {T_imu_cam[2][2]}, {T_imu_cam[2][3]}]
  - [0, 0, 0, 1]

# IMU noise parameters
gyroscope_noise_density: {imu_params['gyroscope_noise_density']}
accelerometer_noise_density: {imu_params['accelerometer_noise_density']}
gyroscope_random_walk: {imu_params['gyroscope_random_walk']}
accelerometer_random_walk: {imu_params['accelerometer_random_walk']}

# Online calibration
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
"""

    with open(output_path, 'w') as f:
        f.write(content)
    print(f"Saved OpenVINS config to {output_path}")

def to_orbslam3(kalibr_data, imu_params, output_path):
    """ORB-SLAM3 형식으로 변환"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    T_imu_cam = np.linalg.inv(T_cam_imu)
    intrinsics = cam['intrinsics']
    dist = cam['distortion_coeffs']
    resolution = cam['resolution']

    # ORB-SLAM3은 노이즈 파라미터를 10배 증가 권장
    noise_factor = 10

    # 4x4 행렬을 1D 리스트로
    tbc_data = T_imu_cam.flatten().tolist()

    content = f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters
#--------------------------------------------------------------------------------------------

Camera.type: "PinHole"

Camera1.fx: {intrinsics[0]}
Camera1.fy: {intrinsics[1]}
Camera1.cx: {intrinsics[2]}
Camera1.cy: {intrinsics[3]}

Camera1.k1: {dist[0]}
Camera1.k2: {dist[1]}
Camera1.p1: {dist[2]}
Camera1.p2: {dist[3]}

Camera.width: {resolution[0]}
Camera.height: {resolution[1]}
Camera.fps: 30
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# IMU Parameters
#--------------------------------------------------------------------------------------------

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{', '.join([str(x) for x in tbc_data])}]

IMU.NoiseGyro: {imu_params['gyroscope_noise_density'] * noise_factor}
IMU.NoiseAcc: {imu_params['accelerometer_noise_density'] * noise_factor}
IMU.GyroWalk: {imu_params['gyroscope_random_walk'] * noise_factor}
IMU.AccWalk: {imu_params['accelerometer_random_walk'] * noise_factor}
IMU.Frequency: {imu_params.get('update_rate', 200.0)}

#--------------------------------------------------------------------------------------------
# ORB Extractor
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
"""

    with open(output_path, 'w') as f:
        f.write(content)
    print(f"Saved ORB-SLAM3 config to {output_path}")

def to_basalt(kalibr_data, imu_params, output_path):
    """Basalt 형식으로 변환"""
    cam = kalibr_data['cam0']

    T_cam_imu = np.array(cam['T_cam_imu'])
    T_imu_cam = np.linalg.inv(T_cam_imu)

    R = T_imu_cam[:3, :3]
    t = T_imu_cam[:3, 3]
    quat = matrix_to_quaternion(R)

    intrinsics = cam['intrinsics']
    resolution = cam['resolution']
    time_offset = cam.get('timeshift_cam_imu', 0.0)
    time_offset_ns = int(time_offset * 1e9)

    basalt_calib = {
        "value0": {
            "T_imu_cam": [{
                "px": float(t[0]),
                "py": float(t[1]),
                "pz": float(t[2]),
                "qx": float(quat[0]),
                "qy": float(quat[1]),
                "qz": float(quat[2]),
                "qw": float(quat[3])
            }],
            "intrinsics": [{
                "camera_type": "pinhole",
                "intrinsics": {
                    "fx": intrinsics[0],
                    "fy": intrinsics[1],
                    "cx": intrinsics[2],
                    "cy": intrinsics[3]
                }
            }],
            "resolution": [[resolution[0], resolution[1]]],
            "calib_accel_bias": [0.0] * 9,
            "calib_gyro_bias": [0.0] * 12,
            "imu_update_rate": imu_params.get('update_rate', 200.0),
            "accel_noise_std": [imu_params['accelerometer_noise_density']] * 3,
            "gyro_noise_std": [imu_params['gyroscope_noise_density']] * 3,
            "accel_bias_std": [imu_params['accelerometer_random_walk']] * 3,
            "gyro_bias_std": [imu_params['gyroscope_random_walk']] * 3,
            "cam_time_offset_ns": time_offset_ns
        }
    }

    with open(output_path, 'w') as f:
        json.dump(basalt_calib, f, indent=2)
    print(f"Saved Basalt config to {output_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Convert Kalibr calibration')
    parser.add_argument('--kalibr', required=True, help='Kalibr camchain-imucam YAML')
    parser.add_argument('--imu', required=True, help='IMU parameters YAML')
    parser.add_argument('--output-dir', default='.', help='Output directory')
    parser.add_argument('--format', nargs='+',
                        choices=['vins', 'openvins', 'orbslam3', 'basalt', 'all'],
                        default=['all'], help='Output formats')
    args = parser.parse_args()

    # 로드
    kalibr_data = load_kalibr_result(args.kalibr)
    with open(args.imu, 'r') as f:
        imu_data = yaml.safe_load(f)
    imu_params = imu_data['imu0']

    formats = args.format
    if 'all' in formats:
        formats = ['vins', 'openvins', 'orbslam3', 'basalt']

    import os
    os.makedirs(args.output_dir, exist_ok=True)

    if 'vins' in formats:
        to_vins_mono(kalibr_data, imu_params,
                     os.path.join(args.output_dir, 'vins_config.yaml'))

    if 'openvins' in formats:
        to_openvins(kalibr_data, imu_params,
                    os.path.join(args.output_dir, 'openvins_config.yaml'))

    if 'orbslam3' in formats:
        to_orbslam3(kalibr_data, imu_params,
                    os.path.join(args.output_dir, 'orbslam3_config.yaml'))

    if 'basalt' in formats:
        to_basalt(kalibr_data, imu_params,
                  os.path.join(args.output_dir, 'basalt_calib.json'))


if __name__ == '__main__':
    main()
```

#### 사용 방법

```bash
python kalibr_converter.py \
    --kalibr camchain-imucam-calib.yaml \
    --imu imu.yaml \
    --output-dir ./configs \
    --format all
```

---

## 트러블슈팅

### 카메라 캘리브레이션 문제

| 문제 | 원인 | 해결 |
|------|------|------|
| 코너 검출 실패 | 조명 불량, 흐린 이미지 | 조명 개선, 선명한 이미지 촬영 |
| 재투영 오차 큼 (>1.0) | 이미지 부족, 각도 제한 | 더 많은 이미지, 다양한 각도 |
| 비정상적인 왜곡 계수 | 잘못된 체커보드 크기 | 실제 측정값 사용 |

### IMU 캘리브레이션 문제

| 문제 | 원인 | 해결 |
|------|------|------|
| 노이즈 값이 너무 큼 | 진동, 짧은 녹화 시간 | 안정된 환경, 2시간+ 녹화 |
| 랜덤 워크 수렴 안됨 | 녹화 시간 부족 | 더 긴 시간 녹화 (6시간+) |
| Allan plot이 이상함 | 데이터 문제 | IMU 데이터 확인, 재녹화 |

### Camera-IMU 캘리브레이션 문제

| 문제 | 원인 | 해결 |
|------|------|------|
| 수렴하지 않음 | 움직임 부족 | 모든 축 충분히 자극 |
| 시간 오프셋이 큼 (>50ms) | 동기화 문제 | 하드웨어 동기화 확인 |
| 재투영 오차 큼 | 카메라 캘리브 오류 | 2단계 재수행 |
| Kalibr 크래시 | 메모리 부족, 데이터 문제 | bag 파일 확인, 메모리 증가 |

---

## 체크리스트

### 캘리브레이션 전

- [ ] 캘리브레이션 타겟 출력 및 측정
- [ ] 타겟 평평하게 설치
- [ ] 카메라 설정 고정 (노출, 포커스, 줌)
- [ ] IMU 샘플링 레이트 확인 (200Hz 권장)
- [ ] 데이터 동기화 방식 확인

### 캘리브레이션 후

- [ ] 재투영 오차 < 0.5 pixels
- [ ] 시간 오프셋 합리적 (-50ms ~ +50ms)
- [ ] T_cam_imu 물리적으로 타당
- [ ] IMU 노이즈 파라미터 합리적 범위
- [ ] 설정 파일 변환 완료
- [ ] 실제 데이터로 테스트 완료

---

## 참고 자료

### 도구
- [Kalibr GitHub](https://github.com/ethz-asl/kalibr)
- [imu_utils GitHub](https://github.com/gaowenliang/imu_utils)
- [kalibr_allan GitHub](https://github.com/rpng/kalibr_allan)
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

### 문서
- [Kalibr Wiki - Camera IMU Calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [Kalibr Wiki - IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [iPhone Calibration with Kalibr](https://tomas789.medium.com/iphone-calibration-camera-imu-and-kalibr-33b8645fb0aa)

### 튜토리얼
- [Robotics Knowledgebase - IMU-Camera Calibration](https://roboticsknowledgebase.com/wiki/sensing/camera-imu-calibration/)
- [LearnOpenCV - Camera Calibration](https://learnopencv.com/camera-calibration-using-opencv/)

---

*문서 생성일: 2026-01-19*
*버전: 1.0.0*
