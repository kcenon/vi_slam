# Smartphone Camera/IMU Data Collection

## 1. 개요

스마트폰은 고품질 카메라와 IMU 센서를 내장하고 있어 VI-SLAM 연구 및 개발에 활용 가능한 플랫폼이다.

### 스마트폰 센서 특성

| 센서 | 일반적 사양 | 용도 |
|------|------------|------|
| 카메라 | 30-60Hz, 1080p+ | Visual feature 추출 |
| 가속도계 | 100-500Hz | 선형 가속도 측정 |
| 자이로스코프 | 100-500Hz | 각속도 측정 |
| 자력계 | 50-100Hz | 방향 보정 (선택) |

## 2. 주요 오픈소스 데이터 수집 앱

### 2.1 OpenCamera Sensors (Android)

**GitHub**: [prime-slam/OpenCamera-Sensors](https://github.com/prime-slam/OpenCamera-Sensors)

#### 특징
- Open Camera 기반 오픈소스 앱
- 동기화된 비디오 및 IMU 데이터 기록
- 네트워크를 통한 원격 제어 지원
- 다중 스마트폰 동시 기록 가능

#### 수집 데이터
- 비디오: H.264/MP4
- IMU: 가속도계, 자이로스코프, 자력계
- 타임스탬프: 동일 클럭에 동기화

#### 주의사항
- Camera2 API 지원 필수
- 모든 기기에서 동기화 타임스탬프 지원하지 않음
- 설정에서 기기 호환성 확인 필요

### 2.2 MARS Logger (Mobile AR Sensor Logger)

**GitHub**: [OSUPCVLab/mobile-ar-sensor-logger](https://github.com/OSUPCVLab/mobile-ar-sensor-logger)

#### 특징
- Android와 iOS 모두 지원
- AR 센서 데이터 수집에 최적화
- 카메라-IMU 간 최상의 동기화

#### 성능
- 카메라: ~30Hz
- IMU: 50-100Hz
- 기록 시간: 10분 이상 가능

#### Android 구현
```
- Camera2 API 사용
- OpenGL ES (GLSurfaceView)
- MediaCodec/MediaMuxer로 비디오 인코딩
- 백그라운드 HandlerThread에서 IMU 기록
```

#### iOS 구현
```
- AVCaptureVideoDataOutput 사용
- AVAssetWriter로 비디오 저장
- CSV 파일로 메타데이터 저장
```

#### 출력 파일
- `video.mp4`: H.264 인코딩 비디오
- `movie_metadata.csv`: 프레임별 타임스탬프, 초점거리, 노출 시간
- `gyro_accel.csv`: IMU 데이터
- `imu_timestamps.txt`: IMU 타임스탬프

### 2.3 VideoIMUCapture-Android

**GitHub**: [DavidGillsjo/VideoIMUCapture-Android](https://github.com/DavidGillsjo/VideoIMUCapture-Android)

#### 특징
- SLAM/SfM 연구용으로 설계
- OIS(광학 이미지 안정화) 데이터 제공
- DVS(디지털 영상 안정화) 구분

#### 성능
- 카메라: ~30Hz
- IMU: ~100Hz
- 동일 클럭 동기화

#### 출력 형식
- 비디오: H.264/MP4
- 메타데이터: Protocol Buffers 3 형식

#### OIS 관련 주의사항
현대 스마트폰의 대부분은 OIS를 탑재하고 있어 프레임별로 카메라 파라미터가 다름:
- 많은 Android 기기에서 OIS 비활성화 불가
- 일부 기기만 렌즈 움직임 데이터 제공
- 3D 재구성 시 문제 발생 가능

### 2.4 Sensors-Data-Logger

**GitHub**: [PyojinKim/Sensors-Data-Logger](https://github.com/PyojinKim/Sensors-Data-Logger)

#### 특징
- Android 전용 (Java)
- IMU + WiFi 데이터 기록
- 텍스트 파일로 오프라인 사용 가능

#### 지원 센서
- 가속도계
- 자이로스코프
- 자력계
- WiFi 신호 강도

## 3. 데이터 수집 시 고려사항

### 3.1 타임스탬프 동기화

VI-SLAM 성능에 가장 중요한 요소:

```
문제점:
- 카메라와 IMU 클럭 불일치
- 트리거 지연
- 전송 지연

해결책:
- Hardware synchronization (최선)
- Software synchronization (차선)
- Online temporal calibration (후처리)
```

### 3.2 권장 수집 설정

| 파라미터 | 권장값 | 비고 |
|---------|-------|------|
| 카메라 FPS | 30Hz | 20-60Hz 범위 |
| IMU 샘플링 | 200Hz | 최소 100Hz |
| 해상도 | 720p-1080p | 처리 속도와 타협 |
| 노출 시간 | 낮게 유지 | 모션 블러 최소화 |
| 자동 노출 | 고정 권장 | 일관된 이미지 품질 |

### 3.3 데이터 품질 체크리스트

- [ ] 일정한 조명 환경
- [ ] 충분한 텍스처가 있는 장면
- [ ] 모션 블러 최소화
- [ ] 급격한 회전 동작 피하기
- [ ] IMU 데이터 연속성 확인
- [ ] 타임스탬프 단조 증가 확인

## 4. 플랫폼별 API

### Android

```java
// Camera2 API - 타임스탬프 획득
CameraCaptureSession.CaptureCallback callback =
    new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureCompleted(CameraCaptureSession session,
                CaptureRequest request, TotalCaptureResult result) {
            Long timestamp = result.get(CaptureResult.SENSOR_TIMESTAMP);
        }
    };

// SensorManager - IMU 데이터
sensorManager.registerListener(listener,
    sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
    SensorManager.SENSOR_DELAY_FASTEST);
```

### iOS

```swift
// AVCaptureVideoDataOutput
func captureOutput(_ output: AVCaptureOutput,
                   didOutput sampleBuffer: CMSampleBuffer,
                   from connection: AVCaptureConnection) {
    let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
}

// CMMotionManager - IMU 데이터
motionManager.startDeviceMotionUpdates(to: .main) { motion, error in
    let attitude = motion?.attitude
    let rotationRate = motion?.rotationRate
    let userAcceleration = motion?.userAcceleration
}
```

## 5. 실시간 스트리밍 vs 오프라인 기록

### 오프라인 기록
- **장점**: 최고 품질 데이터, 동기화 정확도 높음
- **단점**: 실시간 처리 불가, 저장 공간 필요
- **용도**: 연구, 알고리즘 개발, 데이터셋 구축

### 실시간 스트리밍
- **장점**: 즉각적인 피드백, 실시간 응용 가능
- **단점**: 네트워크 지연, 데이터 손실 가능성
- **용도**: 실시간 AR/VR, 로봇 제어

## References

- [Mobile AR Sensor Logger Paper (IEEE 2020)](https://ieeexplore.ieee.org/document/8956816/)
- [OpenCamera Sensors GitHub](https://github.com/prime-slam/OpenCamera-Sensors)
- [VideoIMUCapture-Android GitHub](https://github.com/DavidGillsjo/VideoIMUCapture-Android)
- [MARS Logger arXiv](https://arxiv.org/pdf/2001.00470)
