# Open-source VI-SLAM Frameworks Comparison

## 1. 주요 프레임워크 개요

| 프레임워크 | 개발 기관 | 방식 | 라이선스 | 최신 버전 |
|-----------|----------|------|---------|----------|
| ORB-SLAM3 | Univ. Zaragoza | Optimization | GPLv3 | 1.0 (2021) |
| VINS-Mono | HKUST | Optimization | GPLv3 | - |
| VINS-Fusion | HKUST | Optimization | GPLv3 | - |
| OpenVINS | Univ. Delaware | Filter (MSCKF) | GPLv3 | 2.7+ |
| Basalt | TUM | Optimization | BSD-3 | - |
| Kimera | MIT | Hybrid | BSD-2 | - |
| ROVIO | ETH Zurich | Filter (EKF) | BSD | - |

## 2. 상세 비교

### 2.1 ORB-SLAM3

**GitHub**: [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

#### 특징
- Feature-based SLAM의 가장 완성된 구현
- Multi-map 시스템 지원
- Loop closing 및 relocalization

#### 지원 센서
- Monocular / Stereo / RGB-D
- Monocular-Inertial / Stereo-Inertial

#### 장점
- 최고 수준의 정확도
- 풍부한 기능 (Loop closing, Map saving/loading)
- 다양한 센서 조합 지원

#### 단점
- **센서 동기화 필수** (시간 오프셋 자동 추정 미지원)
- 복잡한 설정
- 무거운 계산량
- GPLv3 라이선스

#### 성능 (EuRoC 데이터셋)
```
ATE RMSE: 0.02-0.05m (최상위)
처리 속도: ~20-30 FPS
```

#### 설치
```bash
# Dependencies
sudo apt install libopencv-dev libeigen3-dev

# Build
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

---

### 2.2 VINS-Mono

**GitHub**: [HKUST-Aerial-Robotics/VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)

#### 특징
- Tightly-coupled monocular VIO
- Online temporal calibration 지원
- Pose graph optimization

#### 지원 센서
- Monocular camera + IMU

#### 장점
- **시간 오프셋 자동 추정** (비동기 센서 허용)
- 강건한 초기화
- 실시간 성능

#### 단점
- Monocular만 지원 (Stereo는 VINS-Fusion)
- Loop closing 제한적

#### 성능 (EuRoC 데이터셋)
```
ATE RMSE: 0.05-0.15m
처리 속도: ~30 FPS
```

#### 설치 (ROS)
```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ../
catkin_make
```

---

### 2.3 VINS-Fusion

**GitHub**: [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

#### 특징
- VINS-Mono의 확장판
- Stereo 및 GPS 융합 지원
- Loop closing 강화

#### 지원 센서
- Mono/Stereo + IMU
- GPS 융합 가능

#### 장점
- 다양한 센서 조합
- GPS 융합으로 drift 보정
- Online temporal calibration

#### 단점
- VINS-Mono보다 복잡한 설정
- 리소스 사용량 증가

#### 성능
```
ATE RMSE: 0.04-0.12m (Stereo-Inertial)
GPS 융합 시 장거리 drift 최소화
```

---

### 2.4 OpenVINS

**GitHub**: [rpng/open_vins](https://github.com/rpng/open_vins)
**문서**: [docs.openvins.com](https://docs.openvins.com/)

#### 특징
- MSCKF 기반 filter 방식
- 연구용으로 최적화된 모듈화 설계
- 우수한 문서화

#### 지원 센서
- Mono/Stereo + IMU

#### 장점
- **최고 수준의 문서화**
- 모듈화된 코드 구조
- Online calibration 지원
- CPU 효율적

#### 단점
- Loop closing 미지원 (VIO만)
- Optimization 기반 대비 정확도 낮음

#### 성능
```
ATE RMSE: 0.06-0.20m
CPU 사용량: 최저 수준
```

#### 설치
```bash
# ROS 1
cd ~/catkin_ws/src
git clone https://github.com/rpng/open_vins.git
cd ../
catkin_make

# ROS 2
colcon build --packages-select ov_core ov_msckf
```

---

### 2.5 Basalt

**GitHub**: [VladyslavUsenko/basalt-mirror](https://gitlab.com/VladyslavUsenko/basalt)

#### 특징
- Graph-based VIO
- 매우 빠른 처리 속도
- XR 애플리케이션에 적합

#### 지원 센서
- Stereo + IMU
- 피쉬아이 카메라 지원

#### 장점
- **최고의 처리 속도**
- 좋은 소프트웨어 엔지니어링
- BSD 라이선스 (상업적 사용 가능)
- CI/문서화 우수

#### 단점
- Monocular 미지원
- Loop closing 미지원

#### 성능
```
ATE RMSE: 0.05-0.15m
처리 속도: ~100+ FPS (최고)
```

#### 설치
```bash
# Dependencies
sudo apt install libopencv-dev libfmt-dev

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

### 2.6 Kimera

**GitHub**: [MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera)

#### 특징
- Metric-Semantic SLAM
- 모듈화된 아키텍처
- 3D 메시 재구성

#### 구성 모듈
1. **Kimera-VIO**: Visual-Inertial Odometry
2. **Kimera-RPGO**: Robust Pose Graph Optimization
3. **Kimera-Mesher**: 3D Mesh 생성
4. **Kimera-Semantics**: Semantic Reconstruction

#### 장점
- 3D 메시 및 시맨틱 맵 생성
- 모듈별 독립 사용 가능
- 연구 확장성

#### 단점
- 설정 복잡
- 리소스 사용량 높음
- 실환경 성능 편차

#### 성능
```
ATE RMSE: 0.10-0.30m (환경 의존)
3D 메시 품질: 우수
```

---

### 2.7 ROVIO

**GitHub**: [ethz-asl/rovio](https://github.com/ethz-asl/rovio)

#### 특징
- EKF 기반 direct VIO
- Feature extraction 불필요
- 경량 구현

#### 장점
- 빠른 처리 속도
- 낮은 리소스 사용
- 텍스처 적은 환경에서도 동작

#### 단점
- Scale drift 발생 가능
- Loop closing 미지원
- 정확도 한계

---

## 3. 성능 벤치마크 비교

### EuRoC MAV Dataset 결과

| 알고리즘 | MH_01 | MH_03 | MH_05 | V1_02 | V2_03 | 평균 |
|---------|-------|-------|-------|-------|-------|------|
| ORB-SLAM3 VI | 0.033 | 0.027 | 0.061 | 0.016 | 0.020 | **0.031** |
| VINS-Fusion | 0.078 | 0.091 | 0.128 | 0.066 | 0.158 | 0.104 |
| OpenVINS | 0.086 | 0.103 | 0.220 | 0.064 | 0.148 | 0.124 |
| Basalt | 0.065 | 0.047 | 0.095 | 0.037 | 0.102 | 0.069 |

*(ATE RMSE, 단위: m)*

### 리소스 사용량 비교

| 알고리즘 | CPU 사용률 | 메모리 사용량 | 실시간 가능 |
|---------|-----------|--------------|------------|
| ORB-SLAM3 | 높음 | 높음 | 조건부 |
| VINS-Fusion | 중간 | 중간 | 예 |
| OpenVINS | **낮음** | 낮음 | 예 |
| Basalt | 낮음 | **낮음** | **예** |
| Kimera | 높음 | 높음 | 조건부 |

## 4. 선택 가이드

### 사용 목적별 추천

| 목적 | 추천 프레임워크 | 이유 |
|------|---------------|------|
| 최고 정확도 | ORB-SLAM3 | 벤치마크 최상위 |
| 실시간 AR/VR | Basalt | 최고 속도, BSD 라이선스 |
| 연구/학습 | OpenVINS | 최고 문서화, 모듈화 |
| 비동기 센서 | VINS-Mono/Fusion | Online temporal calibration |
| 3D 재구성 | Kimera | 메시 및 시맨틱 지원 |
| 상업용 | Basalt | BSD 라이선스 |

### 스마트폰 데이터 적합성

| 프레임워크 | 스마트폰 적합성 | 이유 |
|-----------|---------------|------|
| VINS-Mono | **높음** | 시간 오프셋 자동 추정 |
| VINS-Fusion | **높음** | VINS-Mono + Stereo/GPS |
| OpenVINS | 중간 | 캘리브레이션 필요 |
| ORB-SLAM3 | 낮음 | 정확한 동기화 필수 |
| Basalt | 중간 | Stereo 필요 |

## 5. 설치 요구사항

### 공통 의존성

```bash
# Ubuntu 20.04/22.04 기준
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libopencv-dev libeigen3-dev \
  libboost-all-dev libgoogle-glog-dev \
  libceres-dev
```

### ROS 의존성 (ROS 기반 프레임워크)

```bash
# ROS 1 (Noetic)
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport

# ROS 2 (Humble)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

## 6. 데이터셋

### 벤치마크 데이터셋

| 데이터셋 | 환경 | 센서 | 링크 |
|---------|------|------|------|
| EuRoC MAV | 실내 드론 | Stereo + IMU | [projects.asl.ethz.ch](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) |
| TUM VI | 실내/실외 | Stereo + IMU | [vision.in.tum.de](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) |
| KITTI | 자율주행 | Stereo + GPS/IMU | [cvlibs.net](https://www.cvlibs.net/datasets/kitti/) |
| OpenLORIS | 실내 로봇 | RGB-D + IMU | [lifelong-robotic-vision.github.io](https://lifelong-robotic-vision.github.io/) |

## References

- [Comparison of modern open-source Visual SLAM approaches](https://arxiv.org/pdf/2108.01654)
- [Visual-Inertial SLAM Comparison (Bharat Joshi)](https://joshi-bharat.github.io/projects/visual_slam_comparison/)
- [VI-SLAM Benchmarking (Journal of Field Robotics 2025)](https://onlinelibrary.wiley.com/doi/10.1002/rob.22581)
- [HybVIO Paper (WACV 2022)](https://openaccess.thecvf.com/content/WACV2022/papers/Seiskari_HybVIO_Pushing_the_Limits_of_Real-Time_Visual-Inertial_Odometry_WACV_2022_paper.pdf)
- [Kimera Paper (arXiv 2019)](https://arxiv.org/abs/1910.02490)
