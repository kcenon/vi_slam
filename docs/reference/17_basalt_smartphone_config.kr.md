# Basalt 스마트폰 설정 가이드

## 개요

Basalt는 TUM(Technical University of Munich)에서 개발한 고성능 Visual-Inertial Odometry 시스템으로, 특히 **실시간 처리 속도**와 **정확성**에서 뛰어난 성능을 보입니다. 이 문서에서는 스마트폰 카메라와 IMU 데이터를 사용하여 Basalt를 설정하는 방법을 설명합니다.

### Basalt 특징

| 항목 | 내용 |
|------|------|
| **처리 속도** | 최고 수준 (실시간 AR/VR 적합) |
| **카메라 모델** | Double Sphere (광각 렌즈 최적화) |
| **백엔드** | Square Root Marginalization |
| **시간 오프셋** | 캘리브레이션 시 추정 가능 |
| **프론트엔드** | Optical Flow 기반 |

### 스마트폰 사용 시 고려사항

| 항목 | 설명 |
|------|------|
| **장점** | 빠른 처리 속도, 낮은 지연 시간 |
| **단점** | Kalibr 캘리브레이션 변환 필요, Rolling Shutter 미지원 |
| **권장 시나리오** | 실시간 AR/VR, 저지연 필요 환경 |

---

## 설정 파일 구조

Basalt는 **두 개의 JSON 파일**을 사용합니다:

1. **캘리브레이션 파일** (`calibration.json`) - 카메라 intrinsic, IMU-Camera extrinsic
2. **설정 파일** (`config.json`) - VIO 파라미터, Optical Flow 설정

---

## 캘리브레이션 파일 (calibration.json)

### 기본 구조

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.0,
        "py": 0.0,
        "pz": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0
      }
    ],
    "intrinsics": [
      {
        "camera_type": "ds",
        "intrinsics": {
          "fx": 500.0,
          "fy": 500.0,
          "cx": 320.0,
          "cy": 240.0,
          "xi": -0.2,
          "alpha": 0.5
        }
      }
    ],
    "resolution": [[640, 480]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": 200.0,
    "accel_noise_std": [0.016, 0.016, 0.016],
    "gyro_noise_std": [0.000282, 0.000282, 0.000282],
    "accel_bias_std": [0.001, 0.001, 0.001],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "cam_time_offset_ns": 0
  }
}
```

### 카메라 모델 설명

#### Double Sphere 모델 (ds)

Basalt의 기본 카메라 모델로, 광각 렌즈에 최적화되어 있습니다.

| 파라미터 | 설명 | 일반적인 범위 |
|----------|------|--------------|
| `fx`, `fy` | 초점 거리 (픽셀) | 400-1500 |
| `cx`, `cy` | 주점 좌표 (픽셀) | 이미지 중심 |
| `xi` | 첫 번째 구면 파라미터 | -0.5 ~ 0.5 |
| `alpha` | 두 번째 구면 파라미터 | 0.4 ~ 0.6 |

#### Pinhole 모델 (pinhole)

일반적인 스마트폰 카메라에 사용:

```json
{
  "camera_type": "pinhole",
  "intrinsics": {
    "fx": 500.0,
    "fy": 500.0,
    "cx": 320.0,
    "cy": 240.0
  }
}
```

#### Unified Camera 모델 (ucm)

광각 카메라용 대안:

```json
{
  "camera_type": "ucm",
  "intrinsics": {
    "fx": 500.0,
    "fy": 500.0,
    "cx": 320.0,
    "cy": 240.0,
    "alpha": 0.5
  }
}
```

### IMU-Camera 변환 (T_imu_cam)

`T_imu_cam`은 IMU 좌표계에서 카메라 좌표계로의 변환을 나타냅니다.

```json
"T_imu_cam": [
  {
    "px": 0.005,      // x 방향 변위 [m]
    "py": 0.01,       // y 방향 변위 [m]
    "pz": -0.02,      // z 방향 변위 [m]
    "qx": 0.0,        // 쿼터니언 x
    "qy": 0.707,      // 쿼터니언 y
    "qz": 0.0,        // 쿼터니언 z
    "qw": 0.707       // 쿼터니언 w
  }
]
```

### 시간 오프셋

```json
"cam_time_offset_ns": -5000000
```

- 단위: 나노초 (ns)
- 의미: `IMU 시간 = 카메라 시간 + cam_time_offset_ns`
- 음수값: 이미지가 IMU보다 앞섬

---

## IMU 노이즈 파라미터

### 스마트폰별 권장값

| 스마트폰 등급 | gyro_noise_std | accel_noise_std | gyro_bias_std | accel_bias_std |
|--------------|----------------|-----------------|---------------|----------------|
| **플래그십** | 0.001 | 0.02 | 0.0001 | 0.002 |
| **미드레인지** | 0.002 | 0.04 | 0.0002 | 0.004 |
| **보급형** | 0.005 | 0.08 | 0.0005 | 0.008 |

### 플래그십 스마트폰 예시

```json
{
  "imu_update_rate": 200.0,
  "accel_noise_std": [0.02, 0.02, 0.02],
  "gyro_noise_std": [0.001, 0.001, 0.001],
  "accel_bias_std": [0.002, 0.002, 0.002],
  "gyro_bias_std": [0.0001, 0.0001, 0.0001]
}
```

### 미드레인지 스마트폰 예시

```json
{
  "imu_update_rate": 100.0,
  "accel_noise_std": [0.04, 0.04, 0.04],
  "gyro_noise_std": [0.002, 0.002, 0.002],
  "accel_bias_std": [0.004, 0.004, 0.004],
  "gyro_bias_std": [0.0002, 0.0002, 0.0002]
}
```

---

## VIO 설정 파일 (config.json)

### 전체 설정 구조

```json
{
  "config.optical_flow_type": "frame_to_frame",
  "config.optical_flow_detection_grid_size": 50,
  "config.optical_flow_max_recovered_dist2": 0.04,
  "config.optical_flow_pattern": 51,
  "config.optical_flow_max_iterations": 5,
  "config.optical_flow_epipolar_error": 0.005,
  "config.optical_flow_levels": 3,
  "config.optical_flow_skip_frames": 1,

  "config.vio_linearization_type": "ABS_QR",
  "config.vio_sqrt_marg": true,
  "config.vio_max_states": 3,
  "config.vio_max_kfs": 7,
  "config.vio_min_frames_after_kf": 5,
  "config.vio_new_kf_keypoints_thresh": 0.7,
  "config.vio_debug": false,
  "config.vio_extended_logging": false,

  "config.vio_obs_std_dev": 0.5,
  "config.vio_obs_huber_thresh": 1.0,
  "config.vio_min_triangulation_dist": 0.05,
  "config.vio_outlier_threshold": 3.0,
  "config.vio_filter_iteration": 4,
  "config.vio_max_iterations": 7,
  "config.vio_enforce_realtime": false,

  "config.vio_use_lm": true,
  "config.vio_lm_lambda_initial": 1e-4,
  "config.vio_lm_lambda_min": 1e-6,
  "config.vio_lm_lambda_max": 100.0,

  "config.vio_init_pose_weight": 1e8,
  "config.vio_init_ba_weight": 10.0,
  "config.vio_init_bg_weight": 100.0,

  "config.mapper_obs_std_dev": 0.25,
  "config.mapper_obs_huber_thresh": 1.5,
  "config.mapper_detection_num_points": 800,
  "config.mapper_num_frames_to_match": 30,
  "config.mapper_frames_to_match_threshold": 0.04,
  "config.mapper_min_triangulation_dist": 0.07,
  "config.mapper_no_factor_weights": false,
  "config.mapper_use_factors": true,
  "config.mapper_use_lm": true,
  "config.mapper_lm_lambda_min": 1e-32,
  "config.mapper_lm_lambda_max": 1e2
}
```

---

## 파라미터 상세 설명

### Optical Flow 파라미터

| 파라미터 | 기본값 | 설명 | 튜닝 가이드 |
|----------|--------|------|------------|
| `optical_flow_type` | frame_to_frame | Flow 타입 | 변경 불필요 |
| `optical_flow_detection_grid_size` | 50 | 특징점 검출 그리드 크기 | 작을수록 더 많은 특징점 |
| `optical_flow_max_recovered_dist2` | 0.04 | 양방향 일관성 검사 임계값 | 캘리브레이션 좋으면 낮춤 |
| `optical_flow_pattern` | 51 | 패치 패턴 (24, 51, 52) | 51 권장 |
| `optical_flow_max_iterations` | 5 | 최대 반복 횟수 | 5-10 |
| `optical_flow_epipolar_error` | 0.005 | 에피폴라 오차 임계값 | 0.01-0.05 |
| `optical_flow_levels` | 3 | 피라미드 레벨 수 | 3-4 |
| `optical_flow_skip_frames` | 1 | 프레임 스킵 수 | 1=전체, 2=절반 |

### VIO 백엔드 파라미터

| 파라미터 | 기본값 | 설명 | 튜닝 가이드 |
|----------|--------|------|------------|
| `vio_max_states` | 3 | 유지할 IMU-Pose 상태 수 | 3-5 |
| `vio_max_kfs` | 7 | 최대 키프레임 수 | 5-10 |
| `vio_min_frames_after_kf` | 5 | 키프레임 간 최소 프레임 | 3-10 |
| `vio_new_kf_keypoints_thresh` | 0.7 | 새 키프레임 생성 임계값 | 높을수록 자주 생성 |
| `vio_obs_std_dev` | 0.5 | 관측 표준 편차 | 0.1-0.5 (낮을수록 Flow 신뢰) |
| `vio_obs_huber_thresh` | 1.0 | Huber 손실 임계값 | 1.0-5.0 |
| `vio_min_triangulation_dist` | 0.05 | 최소 삼각측량 거리 [m] | 기준선보다 약간 작게 |
| `vio_outlier_threshold` | 3.0 | 아웃라이어 임계값 | 2.0-4.0 |

### 초기화 가중치

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `vio_init_pose_weight` | 1e8 | 초기 포즈 가중치 |
| `vio_init_ba_weight` | 10.0 | 가속도계 바이어스 초기 가중치 |
| `vio_init_bg_weight` | 100.0 | 자이로 바이어스 초기 가중치 |

---

## 스마트폰 환경별 설정

### 일반 환경 (실내/실외)

```json
{
  "config.optical_flow_detection_grid_size": 50,
  "config.optical_flow_epipolar_error": 0.01,
  "config.optical_flow_skip_frames": 1,

  "config.vio_max_states": 3,
  "config.vio_max_kfs": 7,
  "config.vio_obs_std_dev": 0.5,
  "config.vio_new_kf_keypoints_thresh": 0.7
}
```

### 빠른 동작 환경

```json
{
  "config.optical_flow_detection_grid_size": 40,
  "config.optical_flow_max_iterations": 7,
  "config.optical_flow_skip_frames": 1,

  "config.vio_max_states": 5,
  "config.vio_max_kfs": 10,
  "config.vio_obs_std_dev": 0.3,
  "config.vio_new_kf_keypoints_thresh": 0.8
}
```

### 저조도 환경

```json
{
  "config.optical_flow_detection_grid_size": 60,
  "config.optical_flow_epipolar_error": 0.02,
  "config.optical_flow_max_iterations": 7,

  "config.vio_obs_std_dev": 0.7,
  "config.vio_obs_huber_thresh": 2.0,
  "config.vio_outlier_threshold": 4.0
}
```

### 텍스처 부족 환경

```json
{
  "config.optical_flow_detection_grid_size": 35,
  "config.optical_flow_epipolar_error": 0.02,

  "config.vio_min_triangulation_dist": 0.03,
  "config.vio_new_kf_keypoints_thresh": 0.6,
  "config.mapper_detection_num_points": 1200
}
```

---

## Kalibr에서 Basalt로 변환

### 변환 스크립트

Kalibr YAML 출력을 Basalt JSON 형식으로 변환하는 Python 스크립트:

```python
import yaml
import json
import numpy as np
from scipy.spatial.transform import Rotation

def kalibr_to_basalt(kalibr_cam_yaml, kalibr_imu_yaml, output_json):
    """Kalibr 캘리브레이션 결과를 Basalt 형식으로 변환"""

    # Kalibr 캘리브레이션 로드
    with open(kalibr_cam_yaml, 'r') as f:
        cam_calib = yaml.safe_load(f)

    with open(kalibr_imu_yaml, 'r') as f:
        imu_calib = yaml.safe_load(f)

    # 카메라 파라미터 추출
    cam0 = cam_calib['cam0']
    intrinsics = cam0['intrinsics']
    resolution = cam0['resolution']

    # T_cam_imu에서 T_imu_cam 계산
    T_cam_imu = np.array(cam0['T_cam_imu']).reshape(4, 4)
    T_imu_cam = np.linalg.inv(T_cam_imu)

    # 회전 행렬을 쿼터니언으로 변환
    R = T_imu_cam[:3, :3]
    t = T_imu_cam[:3, 3]
    quat = Rotation.from_matrix(R).as_quat()  # [qx, qy, qz, qw]

    # 시간 오프셋 (초 -> 나노초)
    time_offset_s = cam_calib.get('cam0', {}).get('timeshift_cam_imu', 0.0)
    time_offset_ns = int(time_offset_s * 1e9)

    # IMU 노이즈 파라미터
    imu0 = imu_calib['imu0']
    gyro_noise = imu0['gyroscope_noise_density']
    accel_noise = imu0['accelerometer_noise_density']
    gyro_bias = imu0['gyroscope_random_walk']
    accel_bias = imu0['accelerometer_random_walk']
    imu_rate = imu0['update_rate']

    # Basalt 형식으로 구성
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
                "camera_type": "pinhole",  # 또는 "ds"
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
            "imu_update_rate": imu_rate,
            "accel_noise_std": [accel_noise] * 3,
            "gyro_noise_std": [gyro_noise] * 3,
            "accel_bias_std": [accel_bias] * 3,
            "gyro_bias_std": [gyro_bias] * 3,
            "cam_time_offset_ns": time_offset_ns
        }
    }

    with open(output_json, 'w') as f:
        json.dump(basalt_calib, f, indent=2)

    print(f"Basalt calibration saved to {output_json}")
    return basalt_calib


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--cam-yaml', required=True, help='Kalibr camera YAML')
    parser.add_argument('--imu-yaml', required=True, help='Kalibr IMU YAML')
    parser.add_argument('--output', required=True, help='Output Basalt JSON')
    args = parser.parse_args()

    kalibr_to_basalt(args.cam_yaml, args.imu_yaml, args.output)
```

### 사용 방법

```bash
python kalibr2basalt.py \
    --cam-yaml camchain-imucam.yaml \
    --imu-yaml imu.yaml \
    --output smartphone_calib.json
```

---

## 캘리브레이션 워크플로우

### 1. Basalt 네이티브 캘리브레이션

```bash
# 카메라 캘리브레이션
basalt_calibrate \
    --dataset-path ~/calib_data/camera.bag \
    --dataset-type bag \
    --aprilgrid /usr/etc/basalt/aprilgrid_6x6.json \
    --result-path ~/calib_result/ \
    --cam-types ds

# IMU + 카메라 캘리브레이션
basalt_calibrate_imu \
    --dataset-path ~/calib_data/imu_camera.bag \
    --dataset-type bag \
    --aprilgrid /usr/etc/basalt/aprilgrid_6x6.json \
    --result-path ~/calib_result/ \
    --gyro-noise-std 0.001 \
    --accel-noise-std 0.02 \
    --gyro-bias-std 0.0001 \
    --accel-bias-std 0.002
```

### 2. Kalibr 사용 후 변환

```bash
# Kalibr 캘리브레이션
kalibr_calibrate_imu_camera \
    --target april_grid.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calibration.bag

# Basalt 형식으로 변환
python kalibr2basalt.py \
    --cam-yaml camchain-imucam.yaml \
    --imu-yaml imu.yaml \
    --output smartphone_calib.json
```

### AprilGrid 설정

```json
{
  "tagCols": 6,
  "tagRows": 6,
  "tagSize": 0.088,
  "tagSpacing": 0.3
}
```

---

## 실행 방법

### 기본 실행

```bash
# VIO 실행
basalt_vio \
    --dataset-path /path/to/data/ \
    --cam-calib smartphone_calib.json \
    --dataset-type bag \
    --config-path smartphone_config.json \
    --show-gui 1

# Optical Flow만 실행 (디버깅용)
basalt_opt_flow \
    --dataset-path /path/to/data/ \
    --cam-calib smartphone_calib.json \
    --dataset-type bag \
    --config-path smartphone_config.json \
    --show-gui 1
```

### ROS 통합

```bash
# ROS 토픽으로 실시간 실행
roslaunch basalt basalt_vio.launch \
    cam_calib:=smartphone_calib.json \
    config:=smartphone_config.json
```

### 데이터 토픽

```
/camera/image_raw  - 이미지 (sensor_msgs/Image)
/imu/data          - IMU (sensor_msgs/Imu)
```

---

## 트러블슈팅

### 1. "Not enough optical flow patches"

**증상**: 트래킹 실패, 특징점 부족

**해결**:
```json
{
  "config.optical_flow_detection_grid_size": 35,
  "config.optical_flow_epipolar_error": 0.02,
  "config.mapper_detection_num_points": 1200
}
```

### 2. 초기화 실패

**증상**: VIO가 시작되지 않음

**해결**:
```json
{
  "config.vio_init_pose_weight": 1e6,
  "config.vio_init_ba_weight": 1.0,
  "config.vio_init_bg_weight": 10.0
}
```

그리고 충분한 움직임을 제공하세요 (2-3초간 다양한 방향으로).

### 3. 드리프트 심함

**원인**: IMU 파라미터 불일치 또는 캘리브레이션 오류

**해결**:
```json
{
  "config.vio_obs_std_dev": 0.3,
  "config.vio_obs_huber_thresh": 2.0
}
```

IMU 노이즈 파라미터를 재측정하거나 값을 증가시키세요.

### 4. 처리 속도 저하

**해결**:
```json
{
  "config.optical_flow_skip_frames": 2,
  "config.vio_max_states": 2,
  "config.vio_max_kfs": 5,
  "config.vio_max_iterations": 5
}
```

### 5. 시간 동기화 문제

**증상**: 포즈 추정 불안정

**해결**:
- `cam_time_offset_ns` 값 확인
- Kalibr에서 `--time-calibration` 옵션으로 재캘리브레이션
- 음수값은 이미지가 IMU보다 앞섬을 의미

```json
{
  "cam_time_offset_ns": -5000000
}
```

---

## 전체 설정 파일 템플릿

### 플래그십 스마트폰용 캘리브레이션

<details>
<summary>smartphone_flagship_calib.json (클릭하여 펼치기)</summary>

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.005,
        "py": 0.01,
        "pz": -0.02,
        "qx": 0.0,
        "qy": 0.707,
        "qz": 0.0,
        "qw": 0.707
      }
    ],
    "intrinsics": [
      {
        "camera_type": "pinhole",
        "intrinsics": {
          "fx": 1000.0,
          "fy": 1000.0,
          "cx": 540.0,
          "cy": 960.0
        }
      }
    ],
    "resolution": [[1080, 1920]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": 200.0,
    "accel_noise_std": [0.02, 0.02, 0.02],
    "gyro_noise_std": [0.001, 0.001, 0.001],
    "accel_bias_std": [0.002, 0.002, 0.002],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "cam_time_offset_ns": 0
  }
}
```

</details>

### 플래그십 스마트폰용 VIO 설정

<details>
<summary>smartphone_flagship_config.json (클릭하여 펼치기)</summary>

```json
{
  "config.optical_flow_type": "frame_to_frame",
  "config.optical_flow_detection_grid_size": 50,
  "config.optical_flow_max_recovered_dist2": 0.04,
  "config.optical_flow_pattern": 51,
  "config.optical_flow_max_iterations": 5,
  "config.optical_flow_epipolar_error": 0.01,
  "config.optical_flow_levels": 3,
  "config.optical_flow_skip_frames": 1,

  "config.vio_linearization_type": "ABS_QR",
  "config.vio_sqrt_marg": true,
  "config.vio_max_states": 3,
  "config.vio_max_kfs": 7,
  "config.vio_min_frames_after_kf": 5,
  "config.vio_new_kf_keypoints_thresh": 0.7,
  "config.vio_debug": false,
  "config.vio_extended_logging": false,

  "config.vio_obs_std_dev": 0.5,
  "config.vio_obs_huber_thresh": 1.0,
  "config.vio_min_triangulation_dist": 0.05,
  "config.vio_outlier_threshold": 3.0,
  "config.vio_filter_iteration": 4,
  "config.vio_max_iterations": 7,
  "config.vio_enforce_realtime": false,

  "config.vio_use_lm": true,
  "config.vio_lm_lambda_initial": 1e-4,
  "config.vio_lm_lambda_min": 1e-6,
  "config.vio_lm_lambda_max": 100.0,

  "config.vio_init_pose_weight": 1e8,
  "config.vio_init_ba_weight": 10.0,
  "config.vio_init_bg_weight": 100.0,

  "config.mapper_obs_std_dev": 0.25,
  "config.mapper_obs_huber_thresh": 1.5,
  "config.mapper_detection_num_points": 800,
  "config.mapper_num_frames_to_match": 30,
  "config.mapper_frames_to_match_threshold": 0.04,
  "config.mapper_min_triangulation_dist": 0.07,
  "config.mapper_no_factor_weights": false,
  "config.mapper_use_factors": true,
  "config.mapper_use_lm": true,
  "config.mapper_lm_lambda_min": 1e-32,
  "config.mapper_lm_lambda_max": 1e2
}
```

</details>

### 미드레인지 스마트폰용 캘리브레이션

<details>
<summary>smartphone_midrange_calib.json (클릭하여 펼치기)</summary>

```json
{
  "value0": {
    "T_imu_cam": [
      {
        "px": 0.005,
        "py": 0.01,
        "pz": -0.02,
        "qx": 0.0,
        "qy": 0.707,
        "qz": 0.0,
        "qw": 0.707
      }
    ],
    "intrinsics": [
      {
        "camera_type": "pinhole",
        "intrinsics": {
          "fx": 600.0,
          "fy": 600.0,
          "cx": 320.0,
          "cy": 240.0
        }
      }
    ],
    "resolution": [[640, 480]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": 100.0,
    "accel_noise_std": [0.04, 0.04, 0.04],
    "gyro_noise_std": [0.002, 0.002, 0.002],
    "accel_bias_std": [0.004, 0.004, 0.004],
    "gyro_bias_std": [0.0002, 0.0002, 0.0002],
    "cam_time_offset_ns": 0
  }
}
```

</details>

---

## 프레임워크 비교

| 항목 | Basalt | VINS-Mono | OpenVINS | ORB-SLAM3 |
|------|--------|-----------|----------|-----------|
| **처리 속도** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| **정확도** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **시간 오프셋 자동 추정** | 캘리브레이션 시 | ✅ 런타임 | ✅ 런타임 | ❌ |
| **Rolling Shutter** | ❌ | ✅ | ✅ | ❌ |
| **Loop Closure** | ❌ | ✅ | ❌ | ✅ |
| **스마트폰 호환성** | 좋음 | 매우 좋음 | 매우 좋음 | 보통 |

### Basalt 권장 시나리오

| 시나리오 | 적합도 | 이유 |
|----------|--------|------|
| **실시간 AR/VR** | ⭐⭐⭐⭐⭐ | 최고 처리 속도 |
| **드론/로봇** | ⭐⭐⭐⭐⭐ | 저지연 |
| **스마트폰 일반 사용** | ⭐⭐⭐ | 캘리브레이션 변환 필요 |
| **오프라인 처리** | ⭐⭐⭐ | 다른 프레임워크가 더 적합 |

---

## 참고 자료

### 공식 문서 및 코드
- [Basalt GitLab](https://gitlab.com/VladyslavUsenko/basalt)
- [Basalt GitHub Mirror](https://github.com/VladyslavUsenko/basalt)
- [Basalt Headers Documentation](https://vladyslavusenko.gitlab.io/basalt-headers/)
- [TUM CVG - Basalt](https://cvg.cit.tum.de/research/vslam/basalt)

### 논문
- [Basalt: Visual-Inertial Mapping with Non-Linear Factor Recovery](https://arxiv.org/abs/1904.06504)
- [The Double Sphere Camera Model](https://arxiv.org/abs/1807.08957)

### 튜닝 가이드
- [Basalt VIO Tuning & Configs](https://tlab-uav.github.io/tech-details/docs/research/vio/basalt-tuning)

### 캘리브레이션 도구
- [Kalibr](https://github.com/ethz-asl/kalibr) - Camera-IMU 캘리브레이션
- [OpenImuCameraCalibrator](https://github.com/urbste/OpenImuCameraCalibrator)

---

*문서 생성일: 2026-01-19*
*버전: 1.0.0*
