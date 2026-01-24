# VI-SLAM 프레임워크 선택 가이드

## 개요

이 문서는 스마트폰 데이터를 활용한 Visual-Inertial SLAM 구현 시 적합한 프레임워크를 선택하기 위한 종합 가이드입니다. VINS-Mono, OpenVINS, ORB-SLAM3, Basalt 네 가지 주요 오픈소스 프레임워크를 다양한 관점에서 비교 분석합니다.

---

## 프레임워크 개요

### 한눈에 보는 비교

| 프레임워크 | 개발 기관 | 알고리즘 | 최적 사용 시나리오 |
|-----------|----------|---------|------------------|
| **VINS-Mono** | HKUST | 슬라이딩 윈도우 최적화 | 스마트폰, 드론 |
| **OpenVINS** | Univ. of Delaware | MSCKF (EKF 기반) | 연구, 학습, 커스터마이징 |
| **ORB-SLAM3** | Univ. of Zaragoza | 그래프 최적화 + BoW | 고정밀 매핑, Loop Closure |
| **Basalt** | TUM | Square Root VIO | 실시간 AR/VR, 저지연 |

---

## 상세 비교표

### 1. 기능 비교

| 기능 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **Mono-Inertial** | ✅ | ✅ | ✅ | ✅ |
| **Stereo-Inertial** | ✅ (Fusion) | ✅ | ✅ | ✅ |
| **시간 오프셋 자동 추정** | ✅ | ✅ | ❌ | ⚠️ (캘리브레이션 시) |
| **Rolling Shutter 지원** | ✅ | ✅ | ❌ | ❌ |
| **Loop Closure** | ✅ | ❌ | ✅ | ❌ |
| **Relocalization** | ✅ | ❌ | ✅ | ❌ |
| **Multi-Map** | ❌ | ❌ | ✅ | ❌ |
| **Global BA** | ✅ | ❌ | ✅ | ❌ |
| **온라인 캘리브레이션** | ✅ | ✅ | ❌ | ❌ |
| **Fisheye 지원** | ✅ | ✅ | ✅ | ✅ |

### 2. 성능 비교 (EuRoC/TUM-VI 벤치마크 기준)

| 지표 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **정확도 (ATE)** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **처리 속도** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **CPU 사용량** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **메모리 사용량** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **초기화 속도** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| **강건성** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |

### 3. 벤치마크 수치 (RMSE ATE)

#### EuRoC 데이터셋

| 시퀀스 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|--------|-----------|----------|-----------|--------|
| MH_01_easy | 0.15m | 0.12m | **0.035m** | 0.08m |
| MH_03_medium | 0.19m | 0.15m | **0.037m** | 0.11m |
| MH_05_difficult | 0.30m | 0.22m | **0.062m** | 0.15m |
| V1_01_easy | 0.08m | 0.07m | **0.035m** | **0.04m** |
| V2_03_difficult | 0.28m | 0.20m | **0.10m** | 0.18m |

#### TUM-VI 데이터셋

| 시퀀스 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|--------|-----------|----------|-----------|--------|
| room1 | 0.05m | 0.04m | **0.009m** | 0.03m |
| room4 | 0.08m | 0.06m | **0.012m** | 0.05m |
| corridor1 | 0.15m | 0.12m | **0.08m** | 0.10m |

> **참고**: 위 수치는 대표적인 벤치마크 결과이며, 실제 성능은 파라미터 튜닝과 환경에 따라 달라질 수 있습니다.

### 4. 스마트폰 호환성

| 항목 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **스마트폰 데이터 적합성** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Rolling Shutter 대응** | ✅ 자동 보정 | ✅ 자동 보정 | ❌ 미지원 | ❌ 미지원 |
| **시간 동기화 유연성** | ✅ 자동 추정 | ✅ 자동 추정 | ❌ 사전 캘리브 필수 | ⚠️ 캘리브 시 추정 |
| **저품질 IMU 대응** | ✅ 강건함 | ✅ 강건함 | ⚠️ 정확한 파라미터 필요 | ⚠️ 정확한 파라미터 필요 |
| **설정 난이도** | 중간 | 중간 | 높음 | 중간 |

---

## 시나리오별 권장 프레임워크

### 의사결정 플로우차트

```
시작
  │
  ├─ 실시간 AR/VR이 목표인가?
  │   ├─ 예 → 처리 속도가 가장 중요
  │   │       └─ ✅ Basalt 권장
  │   │
  │   └─ 아니오 ─┐
  │              │
  ├─ Loop Closure가 필요한가? (대규모 환경, 재방문)
  │   ├─ 예 → 전역 일관성이 중요
  │   │       └─ ✅ ORB-SLAM3 또는 VINS-Mono 권장
  │   │
  │   └─ 아니오 ─┐
  │              │
  ├─ 스마트폰 데이터를 사용하는가?
  │   ├─ 예 → Rolling Shutter + 시간 오프셋 대응 필요
  │   │       │
  │   │       ├─ 정확한 캘리브레이션 가능?
  │   │       │   ├─ 예 → ✅ 모든 프레임워크 가능
  │   │       │   └─ 아니오 → ✅ VINS-Mono 또는 OpenVINS 권장
  │   │       │
  │   │       └─ 빠른 프로토타이핑 목적?
  │   │           └─ ✅ VINS-Mono 권장
  │   │
  │   └─ 아니오 ─┐
  │              │
  ├─ 연구/학습 목적인가?
  │   ├─ 예 → 문서화 + 모듈화 중요
  │   │       └─ ✅ OpenVINS 권장
  │   │
  │   └─ 아니오 ─┐
  │              │
  └─ 최고 정확도가 필요한가?
      ├─ 예 → ✅ ORB-SLAM3 권장
      └─ 아니오 → ✅ 상황에 맞게 선택
```

### 시나리오별 상세 가이드

#### 1. 스마트폰 실시간 SLAM

| 우선순위 | 프레임워크 | 이유 |
|---------|-----------|------|
| **1순위** | VINS-Mono | Rolling Shutter 지원, 시간 오프셋 자동 추정, 강건한 초기화 |
| **2순위** | OpenVINS | VINS와 유사한 장점, 더 나은 문서화 |
| **3순위** | Basalt | 처리 속도 우수, 단 RS/시간 오프셋 미지원 |
| **4순위** | ORB-SLAM3 | 정확도 최고, 단 사전 캘리브레이션 필수 |

#### 2. 드론/로봇 자율주행

| 우선순위 | 프레임워크 | 이유 |
|---------|-----------|------|
| **1순위** | Basalt | 최저 지연 시간, 고속 처리 |
| **2순위** | VINS-Fusion | Stereo + GPS 융합 가능 |
| **3순위** | ORB-SLAM3 | Loop Closure로 대규모 환경 대응 |

#### 3. AR/VR 애플리케이션

| 우선순위 | 프레임워크 | 이유 |
|---------|-----------|------|
| **1순위** | Basalt | 최고 처리 속도, 저지연 |
| **2순위** | VINS-Mono | 스마트폰 호환성 우수 |

#### 4. 고정밀 3D 매핑

| 우선순위 | 프레임워크 | 이유 |
|---------|-----------|------|
| **1순위** | ORB-SLAM3 | 최고 정확도, Loop Closure, Multi-Map |
| **2순위** | VINS-Fusion | Global BA 지원 |

#### 5. 연구/교육 목적

| 우선순위 | 프레임워크 | 이유 |
|---------|-----------|------|
| **1순위** | OpenVINS | 우수한 문서화, 모듈화, ROS 통합 |
| **2순위** | VINS-Mono | 활발한 커뮤니티, 많은 참고 자료 |

#### 6. 오프라인 데이터 처리

| 우선순위 | 프레임워크 | 이유 |
|---------|-----------|------|
| **1순위** | ORB-SLAM3 | 전체 최적화 가능, 최고 정확도 |
| **2순위** | OpenVINS | 재처리 및 분석 용이 |

---

## 프레임워크별 장단점 상세

### VINS-Mono / VINS-Fusion

#### 장점
- ✅ **시간 오프셋 자동 추정** (`estimate_td: 1`)
- ✅ **Rolling Shutter 보정** (`rolling_shutter: 1`)
- ✅ **Camera-IMU Extrinsic 온라인 추정** (`estimate_extrinsic: 2`)
- ✅ 강건한 초기화 (2-3초 내)
- ✅ Loop Closure 및 Global BA 지원
- ✅ VINS-Fusion: GPS, Stereo 융합 지원
- ✅ 활발한 커뮤니티 및 풍부한 참고 자료

#### 단점
- ❌ 처리 속도가 Basalt보다 느림
- ❌ CPU/메모리 사용량 상대적으로 높음
- ❌ 코드 모듈화가 OpenVINS보다 부족

#### 권장 설정 (스마트폰)
```yaml
estimate_extrinsic: 2    # 온라인 추정
estimate_td: 1           # 시간 오프셋 추정
rolling_shutter: 1       # RS 보정 활성화
```

---

### OpenVINS

#### 장점
- ✅ **우수한 문서화** (공식 문서, 튜토리얼)
- ✅ **모듈화된 코드 구조** (연구/학습에 적합)
- ✅ 시간 오프셋 및 Extrinsic 온라인 추정
- ✅ Rolling Shutter 지원
- ✅ MSCKF 기반으로 계산 효율적
- ✅ Kalibr와의 원활한 연동
- ✅ ROS1/ROS2 지원

#### 단점
- ❌ Loop Closure 미지원
- ❌ Global 최적화 미지원
- ❌ 대규모 환경에서 드리프트 누적

#### 권장 설정 (스마트폰)
```yaml
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
use_mask: false
```

---

### ORB-SLAM3

#### 장점
- ✅ **최고 정확도** (EuRoC/TUM-VI 벤치마크 1위)
- ✅ **강력한 Loop Closure** (BoW 기반)
- ✅ **Multi-Map 지원** (세션 간 맵 재사용)
- ✅ Relocalization 지원
- ✅ Visual-only, Visual-Inertial, RGB-D 모두 지원
- ✅ 견고한 특징점 추적 (ORB)

#### 단점
- ❌ **시간 오프셋 자동 추정 미지원** (사전 캘리브레이션 필수)
- ❌ **Rolling Shutter 미지원**
- ❌ 초기화가 까다로움 (정확한 파라미터 필요)
- ❌ 스마트폰 데이터에 민감함

#### 권장 설정 (스마트폰)
```yaml
# IMU 노이즈를 10배 증가시켜 초기화 안정성 확보
IMU.NoiseGyro: 1.0e-3
IMU.NoiseAcc: 1.5e-2
IMU.GyroWalk: 1.0e-4
IMU.AccWalk: 3.0e-3
```

---

### Basalt

#### 장점
- ✅ **최고 처리 속도** (실시간 AR/VR에 최적)
- ✅ **최저 CPU/메모리 사용량**
- ✅ Square Root Marginalization (수치적 안정성)
- ✅ Optical Flow 기반 프론트엔드 (효율적)
- ✅ Double Sphere 카메라 모델 (광각 렌즈 지원)
- ✅ 캘리브레이션 시 시간 오프셋 추정 가능

#### 단점
- ❌ **Rolling Shutter 미지원**
- ❌ **Loop Closure 미지원**
- ❌ Kalibr 캘리브레이션 변환 필요
- ❌ 커뮤니티/문서화가 상대적으로 부족

#### 권장 설정 (스마트폰)
```json
{
  "config.optical_flow_epipolar_error": 0.01,
  "config.vio_obs_std_dev": 0.5,
  "config.vio_max_kfs": 7
}
```

---

## 리소스 요구사항 비교

### 최소 하드웨어 사양

| 항목 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|-----------|----------|-----------|--------|
| **CPU** | i5 이상 | i5 이상 | i5 이상 | i3 이상 |
| **RAM** | 8GB | 4GB | 8GB | 4GB |
| **GPU** | 불필요 | 불필요 | 불필요 | 불필요 |

### 권장 하드웨어 사양

| 항목 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|-----------|----------|-----------|--------|
| **CPU** | i7/Ryzen 7 | i5/Ryzen 5 | i7/Ryzen 7 | i5/Ryzen 5 |
| **RAM** | 16GB | 8GB | 16GB | 8GB |
| **저장장치** | SSD | SSD | SSD | SSD |

### 프레임당 처리 시간 (i7 기준)

| 프레임워크 | 평균 처리 시간 | 30fps 실시간 가능 |
|-----------|--------------|-----------------|
| **Basalt** | ~15ms | ✅ 여유 있음 |
| **OpenVINS** | ~25ms | ✅ 가능 |
| **VINS-Mono** | ~35ms | ⚠️ 경계 |
| **ORB-SLAM3** | ~40ms | ⚠️ 경계 |

---

## 설치 및 의존성

### ROS 버전 지원

| 프레임워크 | ROS1 | ROS2 | 독립 실행 |
|-----------|:----:|:----:|:--------:|
| VINS-Mono | ✅ | ⚠️ (포팅 필요) | ❌ |
| OpenVINS | ✅ | ✅ | ✅ |
| ORB-SLAM3 | ✅ | ⚠️ (래퍼 필요) | ✅ |
| Basalt | ✅ | ⚠️ | ✅ |

### 주요 의존성

| 프레임워크 | OpenCV | Eigen | Ceres | g2o | Pangolin |
|-----------|:------:|:-----:|:-----:|:---:|:--------:|
| VINS-Mono | ✅ | ✅ | ✅ | ❌ | ❌ |
| OpenVINS | ✅ | ✅ | ❌ | ❌ | ❌ |
| ORB-SLAM3 | ✅ | ✅ | ❌ | ✅ | ✅ |
| Basalt | ✅ | ✅ | ❌ | ❌ | ❌ |

### 빌드 난이도

| 프레임워크 | 난이도 | 예상 소요 시간 | 주요 이슈 |
|-----------|:------:|--------------|----------|
| VINS-Mono | ⭐⭐ | 30분 | Ceres 버전 호환성 |
| OpenVINS | ⭐ | 15분 | 거의 없음 |
| ORB-SLAM3 | ⭐⭐⭐ | 1시간 | Pangolin, DBoW2 빌드 |
| Basalt | ⭐⭐ | 30분 | 서브모듈 초기화 |

---

## 캘리브레이션 요구사항

### 필수 캘리브레이션 항목

| 항목 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **Camera Intrinsic** | ✅ 필수 | ✅ 필수 | ✅ 필수 | ✅ 필수 |
| **Camera-IMU Extrinsic** | ⚠️ 초기값만 | ⚠️ 초기값만 | ✅ 정확히 필수 | ✅ 정확히 필수 |
| **시간 오프셋** | ⚠️ 자동 추정 | ⚠️ 자동 추정 | ✅ 사전 보정 필수 | ⚠️ 캘리브 시 추정 |
| **IMU 노이즈** | ✅ 필수 | ✅ 필수 | ✅ 필수 | ✅ 필수 |

### 캘리브레이션 도구 호환성

| 도구 | VINS-Mono | OpenVINS | ORB-SLAM3 | Basalt |
|------|:---------:|:--------:|:---------:|:------:|
| **Kalibr** | ✅ 직접 사용 | ✅ 직접 사용 | ⚠️ 변환 필요 | ⚠️ 변환 필요 |
| **OpenCV** | ✅ | ✅ | ✅ | ✅ |
| **자체 도구** | ❌ | ❌ | ❌ | ✅ (basalt_calibrate) |

---

## 빠른 선택 가이드

### 질문 기반 선택

| 질문 | 권장 프레임워크 |
|------|---------------|
| 스마트폰으로 빠르게 테스트하고 싶다 | **VINS-Mono** |
| VI-SLAM을 공부하고 싶다 | **OpenVINS** |
| 최고 정확도가 필요하다 | **ORB-SLAM3** |
| 실시간 AR/VR을 만들고 싶다 | **Basalt** |
| 대규모 환경에서 매핑하고 싶다 | **ORB-SLAM3** |
| 드론에 탑재하고 싶다 | **Basalt** 또는 **VINS-Fusion** |
| 정확한 캘리브레이션이 어렵다 | **VINS-Mono** 또는 **OpenVINS** |
| Global Shutter 카메라를 사용한다 | 모든 프레임워크 가능 |
| Rolling Shutter 카메라를 사용한다 | **VINS-Mono** 또는 **OpenVINS** |

### 원페이지 요약

```
┌─────────────────────────────────────────────────────────────────┐
│                    VI-SLAM 프레임워크 선택 요약                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  🏆 정확도 최고     →  ORB-SLAM3                                │
│  ⚡ 속도 최고       →  Basalt                                   │
│  📱 스마트폰 최적   →  VINS-Mono                                │
│  📚 학습/연구      →  OpenVINS                                  │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Rolling Shutter 필수  →  VINS-Mono, OpenVINS                   │
│  Loop Closure 필수    →  ORB-SLAM3, VINS-Mono                   │
│  실시간 저지연 필수   →  Basalt                                  │
│  캘리브레이션 어려움  →  VINS-Mono, OpenVINS                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 마이그레이션 가이드

### VINS-Mono에서 다른 프레임워크로

#### VINS-Mono → OpenVINS

```yaml
# VINS-Mono 설정
estimate_extrinsic: 2
estimate_td: 1

# OpenVINS 동등 설정
calib_cam_extrinsics: true
calib_cam_timeoffset: true
```

#### VINS-Mono → ORB-SLAM3

**주의**: 시간 오프셋을 사전에 측정하여 데이터 전처리 필요

```python
# 타임스탬프 보정 코드
time_offset = 0.005  # Kalibr에서 측정한 값 [s]
corrected_img_time = img_time + time_offset
```

### Kalibr 결과 활용

| 프레임워크 | Kalibr 결과 사용 방법 |
|-----------|---------------------|
| VINS-Mono | YAML 직접 복사 |
| OpenVINS | YAML 직접 사용 또는 약간 수정 |
| ORB-SLAM3 | T_cam_imu 역변환 → Tbc |
| Basalt | Python 스크립트로 JSON 변환 |

---

## 결론 및 권장사항

### 스마트폰 VI-SLAM 프로젝트 시작 시

1. **초보자/프로토타이핑**: VINS-Mono로 시작
   - 시간 오프셋, Rolling Shutter 자동 처리
   - 설정이 비교적 관대함

2. **연구/학습 목적**: OpenVINS 권장
   - 우수한 문서화
   - 코드 분석 및 수정 용이

3. **최종 제품/고정밀**: ORB-SLAM3
   - 정확한 캘리브레이션 후 사용
   - 최고 성능 달성 가능

4. **실시간 AR/VR**: Basalt
   - 처리 속도 최우선 시
   - 캘리브레이션 정확도 확보 필수

### 단계별 접근 권장

```
1단계: VINS-Mono로 시스템 검증
       ↓
2단계: 캘리브레이션 정밀화 (Kalibr)
       ↓
3단계: 목적에 맞는 프레임워크로 전환
       - 정확도 → ORB-SLAM3
       - 속도 → Basalt
       - 연구 → OpenVINS
```

---

## 참고 자료

### 논문
- [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator](https://arxiv.org/abs/1708.03852)
- [OpenVINS: A Research Platform for Visual-Inertial Estimation](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)
- [ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM](https://arxiv.org/abs/2007.11898)
- [Basalt: Visual-Inertial Mapping with Non-Linear Factor Recovery](https://arxiv.org/abs/1904.06504)

### 벤치마크 비교
- [Comparison of modern open-source Visual SLAM approaches](https://arxiv.org/abs/2108.01654)
- [Visual-Inertial SLAM Comparison (Bharat Joshi)](https://joshi-bharat.github.io/projects/visual_slam_comparison/)
- [The TUM VI Benchmark for Evaluating Visual-Inertial Odometry](https://arxiv.org/abs/1804.06120)

### 공식 저장소
- [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)
- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Basalt GitLab](https://gitlab.com/VladyslavUsenko/basalt)

### 데이터셋
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM VI Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)

---

*문서 생성일: 2026-01-19*
*버전: 1.0.0*
