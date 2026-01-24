# VI-SLAM (Visual-Inertial SLAM) Overview

## 1. VI-SLAM이란?

Visual-Inertial SLAM(VI-SLAM)은 카메라(시각 센서)와 IMU(관성 측정 장치)를 결합하여 동시적 위치추정 및 지도작성(SLAM)을 수행하는 기술이다.

### 핵심 개념

- **Visual SLAM**: 카메라 이미지만으로 환경을 인식하고 위치를 추정
- **IMU**: 가속도계(accelerometer)와 자이로스코프(gyroscope)로 구성, 관성 데이터 제공
- **VI-SLAM**: 두 센서의 상호보완적 특성을 활용한 융합 시스템

### VI-SLAM의 장점

| 특성 | Visual만 | IMU만 | VI-SLAM |
|------|---------|-------|---------|
| Scale 추정 | 불가능(단안) | 가능 | 가능 |
| 빠른 움직임 | 취약 | 강건 | 강건 |
| Motion blur | 취약 | 무관 | 보완 가능 |
| Drift 누적 | 있음 | 심함 | 최소화 |
| 텍스처 없는 환경 | 취약 | 무관 | 보완 가능 |

## 2. VI-SLAM 분류

### 2.1 센서 융합 방식에 따른 분류

#### Loosely Coupled (느슨한 결합)
- Visual과 Inertial 시스템이 독립적으로 동작
- 각 시스템의 출력을 후처리로 결합
- 구현이 간단하나 정확도 한계

#### Tightly Coupled (밀접한 결합)
- Visual과 Inertial 데이터를 하나의 최적화 프레임워크에서 처리
- 더 높은 정확도와 강건성
- 현대 VI-SLAM의 주류 방식

### 2.2 추정 방식에 따른 분류

#### Filter-based (필터 기반)
- **방식**: EKF(Extended Kalman Filter), MSCKF 등 사용
- **대표 알고리즘**: ROVIO, MSCKF, OpenVINS
- **특징**: 실시간 처리에 유리, 계산 효율적
- **단점**: 선형화 오차 누적 가능

#### Optimization-based (최적화 기반)
- **방식**: Bundle Adjustment, Graph Optimization 사용
- **대표 알고리즘**: ORB-SLAM3, VINS-Mono, VINS-Fusion, Basalt
- **특징**: 더 높은 정확도, 비선형성 처리 우수
- **단점**: 계산량 많음

## 3. 주요 VI-SLAM 알고리즘

### 3.1 ORB-SLAM3
- **개발**: University of Zaragoza
- **특징**: 가장 완성도 높은 feature-based SLAM
- **지원**: Monocular, Stereo, RGB-D + IMU
- **장점**: 높은 정확도, 루프 클로징, 맵 재사용
- **단점**: 센서 동기화 필수, 설정 복잡

### 3.2 VINS-Mono / VINS-Fusion
- **개발**: HKUST (Hong Kong University of Science and Technology)
- **특징**: Tightly-coupled optimization-based
- **장점**: 온라인 temporal calibration 지원, 센서 비동기 허용
- **VINS-Fusion 추가 기능**: Stereo 지원, GPS 융합 가능

### 3.3 OpenVINS
- **개발**: University of Delaware
- **특징**: MSCKF 기반 filter 방식
- **장점**: 모듈화 우수, 연구용으로 적합, 문서화 잘됨
- **지원**: Monocular/Stereo + IMU

### 3.4 Basalt
- **개발**: TUM (Technical University of Munich)
- **특징**: Graph-based VIO
- **장점**: 매우 빠른 처리 속도, XR 애플리케이션에 적합
- **특징**: 좋은 소프트웨어 엔지니어링 관행, CI 사용

### 3.5 Kimera
- **개발**: MIT
- **특징**: Metric-Semantic SLAM
- **구성**: VIO 모듈 + Pose Graph Optimizer + 3D Mesher + Semantic Reconstruction
- **장점**: 모듈화 설계, 3D 메시 재구성 가능

### 3.6 ROVIO
- **개발**: ETH Zurich
- **특징**: EKF 기반 direct method
- **장점**: Feature extraction 불필요, 빠른 처리
- **단점**: Scale drift 발생 가능

## 4. 성능 비교 요약

| 알고리즘 | 방식 | 정확도 | 속도 | 강건성 | 구현 난이도 |
|---------|------|--------|------|--------|------------|
| ORB-SLAM3 | Optimization | 최상 | 중간 | 높음 | 높음 |
| VINS-Fusion | Optimization | 상 | 중간 | 높음 | 중간 |
| OpenVINS | Filter | 상 | 빠름 | 중간 | 낮음 |
| Basalt | Optimization | 상 | 최상 | 높음 | 중간 |
| Kimera | Hybrid | 중간 | 중간 | 중간 | 높음 |

## 5. 응용 분야

- **자율주행**: 드론, 로봇, 자율주행차
- **AR/VR**: 증강현실, 가상현실 헤드셋
- **모바일 매핑**: 실내/실외 3D 맵 생성
- **산업용 로봇**: 물류 로봇, AGV

## 6. 주요 도전 과제

1. **동적 환경**: 움직이는 물체가 많은 환경에서의 강건성
2. **조명 변화**: 급격한 조명 변화에 대한 대응
3. **Scale Drift**: 장기간 운용 시 누적 오차
4. **초기화**: 정확한 IMU 바이어스 추정 및 시스템 초기화
5. **캘리브레이션**: 카메라-IMU 간 정확한 시공간 캘리브레이션

## References

- [A review of visual SLAM for robotics (Frontiers 2024)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1347985/full)
- [Visual and Visual-Inertial SLAM Benchmarking (Wiley 2021)](https://onlinelibrary.wiley.com/doi/10.1155/2021/2054828)
- [RPG - Visual-Inertial Odometry and SLAM](https://rpg.ifi.uzh.ch/research_vo.html)
- [A Comprehensive Survey of Visual SLAM Algorithms (MDPI)](https://www.mdpi.com/2218-6581/11/1/24)
- [Comparison of modern open-source Visual SLAM approaches](https://arxiv.org/pdf/2108.01654)
