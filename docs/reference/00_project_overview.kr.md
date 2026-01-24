# VI-SLAM Project: Smartphone to PC Streaming

## 프로젝트 개요

스마트폰의 카메라와 IMU 데이터를 수집하여 실시간(또는 약간의 지연 허용) 스트리밍 환경에서 PC로 수신하여 VI-SLAM을 구성하는 프로젝트.

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                        스마트폰 (Android/iOS)                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │   카메라     │    │     IMU      │    │   타임스탬프  │       │
│  │  (30Hz)     │    │  (200Hz)     │    │    동기화     │       │
│  └──────┬───────┘    └──────┬───────┘    └──────────────┘       │
│         │                   │                                    │
│         ▼                   ▼                                    │
│  ┌──────────────────────────────────────┐                       │
│  │         Data Collection App           │                       │
│  │  (OpenCamera Sensors / MARS Logger)   │                       │
│  └──────────────────┬───────────────────┘                       │
│                     │                                            │
└─────────────────────┼────────────────────────────────────────────┘
                      │
          ┌───────────┴───────────┐
          │    Network Layer      │
          │  WebRTC / RTSP / UDP  │
          └───────────┬───────────┘
                      │
┌─────────────────────┼────────────────────────────────────────────┐
│                     ▼                        PC (Ubuntu/Windows)  │
│  ┌──────────────────────────────────────┐                       │
│  │           Data Receiver              │                       │
│  │    ┌─────────┐    ┌─────────┐       │                       │
│  │    │  Video  │    │   IMU   │       │                       │
│  │    │ Buffer  │    │ Buffer  │       │                       │
│  │    └────┬────┘    └────┬────┘       │                       │
│  │         └──────┬───────┘            │                       │
│  │                ▼                     │                       │
│  │    ┌─────────────────────┐          │                       │
│  │    │ Time Synchronization │          │                       │
│  │    └──────────┬──────────┘          │                       │
│  └───────────────┼──────────────────────┘                       │
│                  ▼                                               │
│  ┌──────────────────────────────────────┐                       │
│  │           VI-SLAM Engine             │                       │
│  │   (VINS-Mono / OpenVINS / Basalt)    │                       │
│  └──────────────────────────────────────┘                       │
│                  │                                               │
│                  ▼                                               │
│  ┌──────────────────────────────────────┐                       │
│  │             Output                    │                       │
│  │  • 6DoF Pose (position + rotation)   │                       │
│  │  • 3D Point Cloud / Map              │                       │
│  │  • Trajectory Visualization          │                       │
│  └──────────────────────────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
```

## 참고 자료 목록

### 기초 자료 (VI-SLAM 개념 및 시스템)

| # | 문서명 | 내용 |
|---|--------|------|
| 01 | [VI-SLAM Overview](01_vi_slam_overview.md) | VI-SLAM 기본 개념, 분류, 주요 알고리즘 |
| 02 | [Smartphone Data Collection](02_smartphone_data_collection.md) | 스마트폰 센서 데이터 수집 앱 및 방법 |
| 03 | [Streaming Protocols](03_streaming_protocols.md) | 실시간 스트리밍 프로토콜 비교 (WebRTC, RTSP 등) |
| 04 | [Calibration & Synchronization](04_calibration_synchronization.md) | 카메라-IMU 캘리브레이션 및 시간 동기화 |
| 05 | [VI-SLAM Frameworks](05_vislam_frameworks.md) | 오픈소스 VI-SLAM 프레임워크 비교 |

### 앱 개발 가이드 (플랫폼별 API 및 구현)

| # | 문서명 | 내용 |
|---|--------|------|
| 06 | [Android Camera2 API](06_android_camera2_api.md) | Camera2 API 상세 가이드, 타임스탬프, 설정 |
| 07 | [Android Sensor API (IMU)](07_android_sensor_api.md) | SensorManager, 고주파수 샘플링, 데이터 저장 |
| 08 | [iOS AVFoundation & CoreMotion](08_ios_avfoundation_coremotion.md) | iOS 카메라/IMU API, Swift 구현 예제 |
| 09 | [Timestamp Synchronization](09_timestamp_synchronization.md) | 카메라-IMU 시간 동기화 구현 방법 |
| 10 | [Data Format & Streaming](10_data_format_streaming.md) | 데이터 포맷(CSV, Binary, Protobuf), 스트리밍 구현 |
| 11 | [App Development Guide](11_app_development_guide.md) | 앱 아키텍처, 핵심 컴포넌트, 캘리브레이션 워크플로우 |

### 센서 심화 (스마트폰 IMU 특성 및 최적화)

| # | 문서명 | 내용 |
|---|--------|------|
| 12 | [Smartphone IMU Drift](12_smartphone_imu_drift.md) | MEMS IMU 오차 유형, Allan Variance, 드리프트 보정 기법 |
| 13 | [IMU Advanced Considerations](13_imu_advanced_considerations.md) | 샘플링 지터, 자력계 융합, ZUPT, 전력 관리, 실패 복구 |

### VI-SLAM 프레임워크 설정 가이드

| # | 문서명 | 내용 |
|---|--------|------|
| 14 | [VINS-Mono Smartphone Config](14_vins_mono_smartphone_config.md) | VINS-Mono 스마트폰 설정, 파라미터 튜닝, 트러블슈팅 |
| 15 | [OpenVINS Smartphone Config](15_openvins_smartphone_config.md) | OpenVINS MSCKF 설정, Kalibr 연동, 캘리브레이션 워크플로우 |
| 16 | [ORB-SLAM3 Smartphone Config](16_orbslam3_smartphone_config.md) | ORB-SLAM3 Mono-Inertial 설정, Tbc 변환, 시간 동기화 |
| 17 | [Basalt Smartphone Config](17_basalt_smartphone_config.md) | Basalt VIO 설정, Optical Flow 튜닝, 실시간 처리 최적화 |
| 18 | [Framework Selection Guide](18_framework_selection_guide.md) | 프레임워크 비교, 시나리오별 선택 가이드, 벤치마크 |

### 실습 가이드

| # | 문서명 | 내용 |
|---|--------|------|
| 19 | [Calibration Practical Guide](19_calibration_practical_guide.md) | 캘리브레이션 단계별 실습, Kalibr/OpenCV 사용법, 변환 스크립트 |

## 권장 기술 스택

### 스마트폰 앱 (데이터 수집)

| 옵션 | 플랫폼 | 장점 | 단점 |
|------|--------|------|------|
| **OpenCamera Sensors** | Android | 오픈소스, 원격제어, 다중기기 | iOS 미지원 |
| **MARS Logger** | Android/iOS | 양 플랫폼 지원, AR 최적화 | 커스터마이징 제한 |
| **Custom App** | 선택 | 완전한 제어 | 개발 시간 필요 |

### 스트리밍 프로토콜

| 시나리오 | 권장 프로토콜 | 지연 시간 |
|---------|-------------|----------|
| 실시간 (< 100ms) | WebRTC | ~300ms |
| 저지연 허용 | RTSP + UDP(IMU) | ~500ms-1s |
| 오프라인 처리 | 파일 저장 후 전송 | N/A |

### VI-SLAM 프레임워크

| 시나리오 | 권장 프레임워크 | 이유 |
|---------|---------------|------|
| **스마트폰 데이터 (권장)** | VINS-Mono/Fusion | 시간 오프셋 자동 추정 |
| 최고 정확도 필요 | ORB-SLAM3 | 벤치마크 최상위 |
| 연구/학습 목적 | OpenVINS | 우수한 문서화 |
| 실시간 AR/VR | Basalt | 최고 처리 속도 |

## 구현 로드맵 (제안)

### Phase 1: 환경 구축 및 오프라인 테스트
1. PC에 VI-SLAM 프레임워크 설치 (VINS-Mono 권장)
2. 공개 데이터셋(EuRoC)으로 동작 확인
3. 스마트폰 데이터 수집 앱 설치 및 테스트
4. 오프라인 데이터로 VI-SLAM 테스트

### Phase 2: 캘리브레이션
1. 카메라 intrinsic 캘리브레이션
2. Camera-IMU extrinsic 캘리브레이션
3. 시간 동기화 검증

### Phase 3: 실시간 스트리밍 구현
1. 스트리밍 프로토콜 선택 및 구현
2. PC 수신 모듈 개발
3. 데이터 동기화 모듈 구현

### Phase 4: 통합 및 최적화
1. 전체 파이프라인 통합
2. 지연 시간 최적화
3. 안정성 테스트

## 주요 챌린지 및 대응

| 챌린지 | 영향 | 대응 방안 |
|--------|------|----------|
| 카메라-IMU 시간 오프셋 | 정확도 저하 | VINS 계열 사용 (자동 추정) |
| 네트워크 지연/불안정 | 실시간 처리 불가 | 버퍼링 + 타임스탬프 동기화 |
| Rolling Shutter | 빠른 동작 시 오차 | VINS-Mono 사용 (RS 모델 지원) |
| OIS 렌즈 움직임 | 캘리브레이션 오차 | OIS 데이터 활용 또는 외장 카메라 |
| 조명 변화 | 트래킹 실패 | 노출 고정, 히스토그램 정규화 |

## 필요 하드웨어

### 최소 요구사항

**스마트폰:**
- Android 8.0+ (Camera2 API 지원)
- 또는 iOS 11+
- 자이로스코프 및 가속도계 내장

**PC:**
- Ubuntu 18.04/20.04/22.04 (ROS 지원)
- CPU: Intel i5 이상
- RAM: 8GB 이상
- GPU: 선택사항 (가속용)

**네트워크:**
- 5GHz WiFi (권장)
- 또는 USB 테더링

### 권장 사양

**스마트폰:**
- Camera2 API 타임스탬프 동기화 지원 기기
- 최근 플래그십 기기 (OIS 데이터 제공 가능성 높음)

**PC:**
- Ubuntu 20.04 + ROS Noetic
- CPU: Intel i7 또는 AMD Ryzen 7 이상
- RAM: 16GB 이상
- SSD 저장장치

## 유용한 링크

### VI-SLAM 프레임워크
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)
- [Basalt GitLab](https://gitlab.com/VladyslavUsenko/basalt)

### 데이터 수집
- [OpenCamera Sensors GitHub](https://github.com/prime-slam/OpenCamera-Sensors)
- [MARS Logger GitHub](https://github.com/OSUPCVLab/mobile-ar-sensor-logger)

### 캘리브레이션
- [Kalibr GitHub](https://github.com/ethz-asl/kalibr)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)

### 데이터셋
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM VI Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)

---

*문서 생성일: 2026-01-19*
*버전: 0.1.8.0 (캘리브레이션 실습 가이드 추가)*

<!-- 버전 정책: x.major.minor.hotfix -->
<!-- 0.1.0.0: 초기 문서 구성 (01-11) -->
<!-- 0.1.1.0: IMU 드리프트 문서 (12) -->
<!-- 0.1.2.0: IMU 고급 고려사항 (13) -->
<!-- 0.1.3.0: VINS-Mono 설정 (14) -->
<!-- 0.1.4.0: OpenVINS 설정 (15) -->
<!-- 0.1.5.0: ORB-SLAM3 설정 (16) -->
<!-- 0.1.6.0: Basalt 설정 (17) -->
<!-- 0.1.7.0: 프레임워크 선택 가이드 (18) -->
<!-- 0.1.8.0: 캘리브레이션 실습 가이드 (19) -->
