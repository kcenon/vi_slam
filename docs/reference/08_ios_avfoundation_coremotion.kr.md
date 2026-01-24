# iOS AVFoundation & CoreMotion Guide for VI-SLAM

## 1. 개요

iOS에서 카메라와 IMU 데이터를 수집하기 위해 AVFoundation(카메라)과 CoreMotion(IMU) 프레임워크를 사용한다.

### 프레임워크 개요

| 프레임워크 | 용도 | 주요 클래스 |
|-----------|------|------------|
| AVFoundation | 카메라 캡처, 비디오 녹화 | AVCaptureSession, AVCaptureDevice |
| CoreMotion | IMU 데이터 수집 | CMMotionManager, CMDeviceMotion |
| CoreMedia | 미디어 타이밍/포맷 | CMSampleBuffer, CMTime |

## 2. AVFoundation 카메라 캡처

### 2.1 캡처 세션 설정

```swift
import AVFoundation
import UIKit

class CameraService: NSObject {

    private let captureSession = AVCaptureSession()
    private var videoOutput: AVCaptureVideoDataOutput?
    private var videoDevice: AVCaptureDevice?

    private let videoQueue = DispatchQueue(label: "com.vislam.videoQueue",
                                           qos: .userInteractive)

    // 프레임 콜백
    var onFrameCaptured: ((CMSampleBuffer, CVPixelBuffer, CMTime) -> Void)?

    func setupCamera() throws {
        captureSession.beginConfiguration()

        // 1. 세션 프리셋 설정
        if captureSession.canSetSessionPreset(.hd1280x720) {
            captureSession.sessionPreset = .hd1280x720
        }

        // 2. 카메라 디바이스 선택
        guard let device = AVCaptureDevice.default(
            .builtInWideAngleCamera,
            for: .video,
            position: .back
        ) else {
            throw CameraError.deviceNotFound
        }
        videoDevice = device

        // 3. 입력 추가
        let input = try AVCaptureDeviceInput(device: device)
        if captureSession.canAddInput(input) {
            captureSession.addInput(input)
        }

        // 4. 출력 설정
        let output = AVCaptureVideoDataOutput()
        output.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        output.alwaysDiscardsLateVideoFrames = true
        output.setSampleBufferDelegate(self, queue: videoQueue)

        if captureSession.canAddOutput(output) {
            captureSession.addOutput(output)
        }
        videoOutput = output

        // 5. 비디오 안정화 비활성화 (VI-SLAM에 중요!)
        if let connection = output.connection(with: .video) {
            if connection.isVideoStabilizationSupported {
                connection.preferredVideoStabilizationMode = .off
            }
        }

        captureSession.commitConfiguration()
    }

    func configureForVISLAM() throws {
        guard let device = videoDevice else { return }

        try device.lockForConfiguration()

        // 프레임 레이트 설정 (30fps)
        device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)
        device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 30)

        // 자동 초점 설정
        if device.isFocusModeSupported(.continuousAutoFocus) {
            device.focusMode = .continuousAutoFocus
        }

        // 노출 설정 (자동 또는 고정)
        if device.isExposureModeSupported(.continuousAutoExposure) {
            device.exposureMode = .continuousAutoExposure
        }

        // OIS 비활성화 (가능한 경우)
        // iOS에서는 직접 비활성화 불가능한 경우가 많음

        device.unlockForConfiguration()
    }

    func startCapture() {
        if !captureSession.isRunning {
            videoQueue.async {
                self.captureSession.startRunning()
            }
        }
    }

    func stopCapture() {
        if captureSession.isRunning {
            captureSession.stopRunning()
        }
    }
}

// MARK: - AVCaptureVideoDataOutputSampleBufferDelegate
extension CameraService: AVCaptureVideoDataOutputSampleBufferDelegate {

    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {

        // 타임스탬프 추출
        let presentationTime = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)

        // 픽셀 버퍼 추출
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }

        // 메타데이터 추출
        if let formatDescription = CMSampleBufferGetFormatDescription(sampleBuffer) {
            let dimensions = CMVideoFormatDescriptionGetDimensions(formatDescription)
            // width: dimensions.width, height: dimensions.height
        }

        // 콜백 호출
        onFrameCaptured?(sampleBuffer, pixelBuffer, presentationTime)
    }

    func captureOutput(_ output: AVCaptureOutput,
                       didDrop sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        // 프레임 드롭 발생
        print("Frame dropped")
    }
}

enum CameraError: Error {
    case deviceNotFound
    case configurationFailed
}
```

### 2.2 타임스탬프 처리

```swift
// CMTime을 나노초로 변환
extension CMTime {
    var nanoseconds: Int64 {
        return Int64(CMTimeGetSeconds(self) * 1_000_000_000)
    }

    // 부팅 이후 시간 (CoreMotion과 비교 가능)
    var hostTimeNanoseconds: Int64 {
        // CMSampleBuffer 타임스탬프는 hostTime 기준
        return nanoseconds
    }
}

// 프레임 메타데이터 구조
struct FrameMetadata {
    let timestamp: Int64          // 나노초
    let frameNumber: Int64
    let exposureDuration: Double  // 초
    let iso: Float
    let focalLength: Float        // mm
    let width: Int32
    let height: Int32
}

func extractMetadata(from sampleBuffer: CMSampleBuffer,
                     frameNumber: Int64) -> FrameMetadata? {

    let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)

    // 노출 정보 (가능한 경우)
    var exposureDuration: Double = 0
    var iso: Float = 0

    if let attachments = CMSampleBufferGetSampleAttachmentsArray(sampleBuffer,
                                                                  createIfNecessary: false) as? [[String: Any]],
       let attachment = attachments.first {
        // 메타데이터 추출 시도
    }

    guard let formatDesc = CMSampleBufferGetFormatDescription(sampleBuffer) else {
        return nil
    }

    let dimensions = CMVideoFormatDescriptionGetDimensions(formatDesc)

    return FrameMetadata(
        timestamp: timestamp.nanoseconds,
        frameNumber: frameNumber,
        exposureDuration: exposureDuration,
        iso: iso,
        focalLength: 4.25,  // 대략적인 값, 기기별로 다름
        width: dimensions.width,
        height: dimensions.height
    )
}
```

### 2.3 비디오 녹화

```swift
import AVFoundation

class VideoRecorder {

    private var assetWriter: AVAssetWriter?
    private var videoInput: AVAssetWriterInput?
    private var adaptor: AVAssetWriterInputPixelBufferAdaptor?

    private var isRecording = false
    private var startTime: CMTime?

    func startRecording(to url: URL, width: Int, height: Int) throws {
        assetWriter = try AVAssetWriter(url: url, fileType: .mp4)

        let videoSettings: [String: Any] = [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: width,
            AVVideoHeightKey: height,
            AVVideoCompressionPropertiesKey: [
                AVVideoAverageBitRateKey: 4_000_000,  // 4 Mbps
                AVVideoProfileLevelKey: AVVideoProfileLevelH264HighAutoLevel
            ]
        ]

        videoInput = AVAssetWriterInput(mediaType: .video, outputSettings: videoSettings)
        videoInput?.expectsMediaDataInRealTime = true

        let sourcePixelBufferAttributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            kCVPixelBufferWidthKey as String: width,
            kCVPixelBufferHeightKey as String: height
        ]

        adaptor = AVAssetWriterInputPixelBufferAdaptor(
            assetWriterInput: videoInput!,
            sourcePixelBufferAttributes: sourcePixelBufferAttributes
        )

        if let input = videoInput, assetWriter!.canAdd(input) {
            assetWriter!.add(input)
        }

        assetWriter!.startWriting()
        isRecording = true
    }

    func appendFrame(_ pixelBuffer: CVPixelBuffer, at time: CMTime) {
        guard isRecording, let adaptor = adaptor, let input = videoInput else { return }

        if startTime == nil {
            startTime = time
            assetWriter?.startSession(atSourceTime: time)
        }

        if input.isReadyForMoreMediaData {
            adaptor.append(pixelBuffer, withPresentationTime: time)
        }
    }

    func stopRecording(completion: @escaping (URL?) -> Void) {
        guard isRecording else {
            completion(nil)
            return
        }

        isRecording = false
        videoInput?.markAsFinished()

        assetWriter?.finishWriting { [weak self] in
            completion(self?.assetWriter?.outputURL)
        }
    }
}
```

## 3. CoreMotion IMU 데이터

### 3.1 CMMotionManager 설정

```swift
import CoreMotion

class MotionService {

    private let motionManager = CMMotionManager()
    private let motionQueue = OperationQueue()

    // IMU 데이터 버퍼
    private var imuBuffer: [ImuReading] = []
    private let bufferLock = NSLock()
    private let maxBufferSize = 10000

    // 콜백
    var onImuData: ((ImuReading) -> Void)?

    init() {
        motionQueue.name = "com.vislam.motionQueue"
        motionQueue.maxConcurrentOperationCount = 1
    }

    func checkAvailability() -> (accel: Bool, gyro: Bool, deviceMotion: Bool) {
        return (
            accel: motionManager.isAccelerometerAvailable,
            gyro: motionManager.isGyroAvailable,
            deviceMotion: motionManager.isDeviceMotionAvailable
        )
    }

    // 개별 센서 사용 (최대 100Hz)
    func startRawSensors(frequency: Double = 100.0) {
        let interval = 1.0 / frequency

        // 가속도계
        if motionManager.isAccelerometerAvailable {
            motionManager.accelerometerUpdateInterval = interval
            motionManager.startAccelerometerUpdates(to: motionQueue) { [weak self] data, error in
                guard let data = data else { return }

                let reading = ImuReading(
                    timestamp: self?.hostTimeToNanos(data.timestamp) ?? 0,
                    sensorType: .accelerometer,
                    x: data.acceleration.x * 9.81,  // g → m/s²
                    y: data.acceleration.y * 9.81,
                    z: data.acceleration.z * 9.81
                )
                self?.addToBuffer(reading)
                self?.onImuData?(reading)
            }
        }

        // 자이로스코프
        if motionManager.isGyroAvailable {
            motionManager.gyroUpdateInterval = interval
            motionManager.startGyroUpdates(to: motionQueue) { [weak self] data, error in
                guard let data = data else { return }

                let reading = ImuReading(
                    timestamp: self?.hostTimeToNanos(data.timestamp) ?? 0,
                    sensorType: .gyroscope,
                    x: data.rotationRate.x,  // rad/s
                    y: data.rotationRate.y,
                    z: data.rotationRate.z
                )
                self?.addToBuffer(reading)
                self?.onImuData?(reading)
            }
        }
    }

    // Device Motion 사용 (센서 퓨전된 데이터)
    func startDeviceMotion(frequency: Double = 100.0) {
        guard motionManager.isDeviceMotionAvailable else { return }

        motionManager.deviceMotionUpdateInterval = 1.0 / frequency

        // 참조 프레임 선택
        motionManager.startDeviceMotionUpdates(
            using: .xArbitraryZVertical,  // 중력 방향 기준
            to: motionQueue
        ) { [weak self] motion, error in
            guard let motion = motion else { return }

            // 사용자 가속도 (중력 제외)
            let userAccel = ImuReading(
                timestamp: self?.hostTimeToNanos(motion.timestamp) ?? 0,
                sensorType: .userAcceleration,
                x: motion.userAcceleration.x * 9.81,
                y: motion.userAcceleration.y * 9.81,
                z: motion.userAcceleration.z * 9.81
            )

            // 회전 속도
            let rotationRate = ImuReading(
                timestamp: self?.hostTimeToNanos(motion.timestamp) ?? 0,
                sensorType: .rotationRate,
                x: motion.rotationRate.x,
                y: motion.rotationRate.y,
                z: motion.rotationRate.z
            )

            // 자세 (쿼터니언)
            let attitude = motion.attitude
            // attitude.quaternion.w, x, y, z

            self?.addToBuffer(userAccel)
            self?.addToBuffer(rotationRate)
        }
    }

    func stop() {
        motionManager.stopAccelerometerUpdates()
        motionManager.stopGyroUpdates()
        motionManager.stopDeviceMotionUpdates()
    }

    // CoreMotion timestamp → 나노초
    private func hostTimeToNanos(_ timestamp: TimeInterval) -> Int64 {
        // CoreMotion timestamp는 시스템 부팅 이후 초 단위
        return Int64(timestamp * 1_000_000_000)
    }

    private func addToBuffer(_ reading: ImuReading) {
        bufferLock.lock()
        defer { bufferLock.unlock() }

        imuBuffer.append(reading)
        if imuBuffer.count > maxBufferSize {
            imuBuffer.removeFirst()
        }
    }

    func getBufferedData() -> [ImuReading] {
        bufferLock.lock()
        defer { bufferLock.unlock() }
        return imuBuffer
    }

    func clearBuffer() {
        bufferLock.lock()
        defer { bufferLock.unlock() }
        imuBuffer.removeAll()
    }
}
```

### 3.2 데이터 구조

```swift
enum SensorType: String, Codable {
    case accelerometer
    case gyroscope
    case magnetometer
    case userAcceleration
    case rotationRate
    case gravity
}

struct ImuReading: Codable {
    let timestamp: Int64    // 나노초
    let sensorType: SensorType
    let x: Double
    let y: Double
    let z: Double

    // CSV 형식 출력
    var csvLine: String {
        return "\(timestamp),\(sensorType.rawValue),\(x),\(y),\(z)"
    }
}

// 확장: 결합된 IMU 프레임
struct ImuFrame: Codable {
    let timestamp: Int64
    let accelerometer: SIMD3<Double>  // m/s²
    let gyroscope: SIMD3<Double>      // rad/s
    let magnetometer: SIMD3<Double>?  // μT (선택)
}
```

### 3.3 고주파수 샘플링 (CMBatchedSensorManager)

```swift
// iOS 17+, Apple Watch Series 8+ 에서 사용 가능
import CoreMotion

@available(iOS 17.0, *)
class HighFrequencyMotionService {

    private let batchedManager = CMBatchedSensorManager()

    func startHighFrequencySampling() async throws {
        // 최대 800Hz 가속도계, 200Hz 디바이스 모션
        // 단, HealthKit 운동 세션 필요

        for await batch in batchedManager.accelerometerUpdates() {
            for data in batch {
                // data.timestamp, data.acceleration
                print("Accel: \(data.acceleration) at \(data.timestamp)")
            }
        }
    }
}
```

## 4. 카메라-IMU 동기화

### 4.1 타임스탬프 기준

```swift
/*
 iOS 타임스탬프 기준:

 1. CMSampleBuffer (카메라)
    - CMSampleBufferGetPresentationTimeStamp()
    - hostTime 기준 (mach_absolute_time 변환)

 2. CoreMotion (IMU)
    - CMAccelerometerData.timestamp
    - CMDeviceMotion.timestamp
    - 시스템 부팅 이후 초 단위 (TimeInterval)

 두 타임스탬프 모두 같은 시스템 클럭 기반이므로 직접 비교 가능
*/

class TimestampSynchronizer {

    // hostTime을 나노초로 변환
    func hostTimeToNanos() -> Int64 {
        var timebaseInfo = mach_timebase_info_data_t()
        mach_timebase_info(&timebaseInfo)

        let hostTime = mach_absolute_time()
        let nanos = hostTime * UInt64(timebaseInfo.numer) / UInt64(timebaseInfo.denom)
        return Int64(nanos)
    }

    // CoreMotion timestamp (초) → 나노초
    func motionTimestampToNanos(_ timestamp: TimeInterval) -> Int64 {
        return Int64(timestamp * 1_000_000_000)
    }

    // CMTime → 나노초
    func cmTimeToNanos(_ time: CMTime) -> Int64 {
        return Int64(CMTimeGetSeconds(time) * 1_000_000_000)
    }
}
```

### 4.2 동기화된 데이터 수집

```swift
class SynchronizedDataCollector {

    private let cameraService = CameraService()
    private let motionService = MotionService()
    private let synchronizer = TimestampSynchronizer()

    private var frameMetadataList: [FrameMetadata] = []
    private var imuDataList: [ImuReading] = []

    func startCollection() throws {
        // 카메라 프레임 콜백
        cameraService.onFrameCaptured = { [weak self] sampleBuffer, pixelBuffer, time in
            let timestamp = self?.synchronizer.cmTimeToNanos(time) ?? 0

            // 프레임 저장 및 메타데이터 기록
            self?.processFrame(pixelBuffer: pixelBuffer, timestamp: timestamp)
        }

        // IMU 데이터 콜백
        motionService.onImuData = { [weak self] reading in
            self?.imuDataList.append(reading)
        }

        // 시작
        try cameraService.setupCamera()
        try cameraService.configureForVISLAM()
        motionService.startRawSensors(frequency: 200.0)
        cameraService.startCapture()
    }

    func stopCollection() {
        cameraService.stopCapture()
        motionService.stop()
    }

    // 프레임에 가까운 IMU 데이터 매칭
    func matchImuToFrame(frameTimestamp: Int64,
                         windowNs: Int64 = 50_000_000) -> [ImuReading] {
        return imuDataList.filter { imu in
            abs(imu.timestamp - frameTimestamp) <= windowNs
        }
    }

    private func processFrame(pixelBuffer: CVPixelBuffer, timestamp: Int64) {
        // 프레임 처리 로직
    }
}
```

## 5. 데이터 저장

### 5.1 파일 저장

```swift
class DataExporter {

    private let fileManager = FileManager.default
    private let documentsDirectory: URL

    init() {
        documentsDirectory = fileManager.urls(for: .documentDirectory,
                                               in: .userDomainMask).first!
    }

    func createSessionDirectory() -> URL {
        let timestamp = ISO8601DateFormatter().string(from: Date())
        let sessionDir = documentsDirectory.appendingPathComponent("session_\(timestamp)")

        try? fileManager.createDirectory(at: sessionDir,
                                         withIntermediateDirectories: true)
        return sessionDir
    }

    func saveImuData(_ data: [ImuReading], to directory: URL) throws {
        let imuFile = directory.appendingPathComponent("imu_data.csv")

        var csvContent = "timestamp_ns,sensor_type,x,y,z\n"
        for reading in data {
            csvContent += reading.csvLine + "\n"
        }

        try csvContent.write(to: imuFile, atomically: true, encoding: .utf8)
    }

    func saveFrameMetadata(_ metadata: [FrameMetadata], to directory: URL) throws {
        let metadataFile = directory.appendingPathComponent("frame_metadata.csv")

        var csvContent = "frame_number,timestamp_ns,exposure_duration,iso,focal_length,width,height\n"
        for frame in metadata {
            csvContent += "\(frame.frameNumber),\(frame.timestamp),\(frame.exposureDuration),"
            csvContent += "\(frame.iso),\(frame.focalLength),\(frame.width),\(frame.height)\n"
        }

        try csvContent.write(to: metadataFile, atomically: true, encoding: .utf8)
    }
}
```

## 6. iOS 좌표계

```
iOS 기기 좌표계 (세로 모드, 홈 버튼 아래):

        Y (위쪽, 귀 방향)
        ^
        |
        |
        +-------> X (오른쪽)
       /
      /
     v
    Z (사용자 방향, 화면 바깥)

- 오른손 좌표계
- Android와 동일한 컨벤션
```

## 7. 권한 설정

### Info.plist

```xml
<key>NSCameraUsageDescription</key>
<string>VI-SLAM 데이터 수집을 위해 카메라 접근이 필요합니다.</string>

<key>NSMotionUsageDescription</key>
<string>VI-SLAM 데이터 수집을 위해 모션 센서 접근이 필요합니다.</string>

<key>UIBackgroundModes</key>
<array>
    <string>location</string>
</array>
```

## References

- [AVFoundation - Apple Developer](https://developer.apple.com/documentation/avfoundation/)
- [Core Motion - Apple Developer](https://developer.apple.com/documentation/coremotion/)
- [CMMotionManager - Apple Developer](https://developer.apple.com/documentation/coremotion/cmmotionmanager)
- [AVCam Sample Code](https://developer.apple.com/documentation/avfoundation/avcam-building-a-camera-app)
- [Camera Capture on iOS - objc.io](https://www.objc.io/issues/21-camera-and-photos/camera-capture-on-ios/)
- [CMDeviceMotion - NSHipster](https://nshipster.com/cmdevicemotion/)
