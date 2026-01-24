# iOS AVFoundation & CoreMotion Guide for VI-SLAM

## 1. Overview

To collect camera and IMU data on iOS, we use the AVFoundation (camera) and CoreMotion (IMU) frameworks.

### Framework Overview

| Framework | Purpose | Key Classes |
|-----------|------|------------|
| AVFoundation | Camera capture, video recording | AVCaptureSession, AVCaptureDevice |
| CoreMotion | IMU data collection | CMMotionManager, CMDeviceMotion |
| CoreMedia | Media timing/format | CMSampleBuffer, CMTime |

## 2. AVFoundation Camera Capture

### 2.1 Capture Session Setup

```swift
import AVFoundation
import UIKit

class CameraService: NSObject {

    private let captureSession = AVCaptureSession()
    private var videoOutput: AVCaptureVideoDataOutput?
    private var videoDevice: AVCaptureDevice?

    private let videoQueue = DispatchQueue(label: "com.vislam.videoQueue",
                                           qos: .userInteractive)

    // Frame callback
    var onFrameCaptured: ((CMSampleBuffer, CVPixelBuffer, CMTime) -> Void)?

    func setupCamera() throws {
        captureSession.beginConfiguration()

        // 1. Set session preset
        if captureSession.canSetSessionPreset(.hd1280x720) {
            captureSession.sessionPreset = .hd1280x720
        }

        // 2. Select camera device
        guard let device = AVCaptureDevice.default(
            .builtInWideAngleCamera,
            for: .video,
            position: .back
        ) else {
            throw CameraError.deviceNotFound
        }
        videoDevice = device

        // 3. Add input
        let input = try AVCaptureDeviceInput(device: device)
        if captureSession.canAddInput(input) {
            captureSession.addInput(input)
        }

        // 4. Configure output
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

        // 5. Disable video stabilization (important for VI-SLAM!)
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

        // Set frame rate (30fps)
        device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)
        device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 30)

        // Auto focus settings
        if device.isFocusModeSupported(.continuousAutoFocus) {
            device.focusMode = .continuousAutoFocus
        }

        // Exposure settings (auto or fixed)
        if device.isExposureModeSupported(.continuousAutoExposure) {
            device.exposureMode = .continuousAutoExposure
        }

        // Disable OIS (if possible)
        // On iOS, direct disabling may not be possible in many cases

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

        // Extract timestamp
        let presentationTime = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)

        // Extract pixel buffer
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }

        // Extract metadata
        if let formatDescription = CMSampleBufferGetFormatDescription(sampleBuffer) {
            let dimensions = CMVideoFormatDescriptionGetDimensions(formatDescription)
            // width: dimensions.width, height: dimensions.height
        }

        // Call callback
        onFrameCaptured?(sampleBuffer, pixelBuffer, presentationTime)
    }

    func captureOutput(_ output: AVCaptureOutput,
                       didDrop sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        // Frame drop occurred
        print("Frame dropped")
    }
}

enum CameraError: Error {
    case deviceNotFound
    case configurationFailed
}
```

### 2.2 Timestamp Processing

```swift
// Convert CMTime to nanoseconds
extension CMTime {
    var nanoseconds: Int64 {
        return Int64(CMTimeGetSeconds(self) * 1_000_000_000)
    }

    // Time since boot (comparable to CoreMotion)
    var hostTimeNanoseconds: Int64 {
        // CMSampleBuffer timestamps are based on hostTime
        return nanoseconds
    }
}

// Frame metadata structure
struct FrameMetadata {
    let timestamp: Int64          // nanoseconds
    let frameNumber: Int64
    let exposureDuration: Double  // seconds
    let iso: Float
    let focalLength: Float        // mm
    let width: Int32
    let height: Int32
}

func extractMetadata(from sampleBuffer: CMSampleBuffer,
                     frameNumber: Int64) -> FrameMetadata? {

    let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)

    // Exposure info (if available)
    var exposureDuration: Double = 0
    var iso: Float = 0

    if let attachments = CMSampleBufferGetSampleAttachmentsArray(sampleBuffer,
                                                                  createIfNecessary: false) as? [[String: Any]],
       let attachment = attachments.first {
        // Attempt metadata extraction
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
        focalLength: 4.25,  // Approximate value, varies by device
        width: dimensions.width,
        height: dimensions.height
    )
}
```

### 2.3 Video Recording

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

## 3. CoreMotion IMU Data

### 3.1 CMMotionManager Setup

```swift
import CoreMotion

class MotionService {

    private let motionManager = CMMotionManager()
    private let motionQueue = OperationQueue()

    // IMU data buffer
    private var imuBuffer: [ImuReading] = []
    private let bufferLock = NSLock()
    private let maxBufferSize = 10000

    // Callback
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

    // Using individual sensors (max 100Hz)
    func startRawSensors(frequency: Double = 100.0) {
        let interval = 1.0 / frequency

        // Accelerometer
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

        // Gyroscope
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

    // Using Device Motion (sensor-fused data)
    func startDeviceMotion(frequency: Double = 100.0) {
        guard motionManager.isDeviceMotionAvailable else { return }

        motionManager.deviceMotionUpdateInterval = 1.0 / frequency

        // Select reference frame
        motionManager.startDeviceMotionUpdates(
            using: .xArbitraryZVertical,  // Gravity direction reference
            to: motionQueue
        ) { [weak self] motion, error in
            guard let motion = motion else { return }

            // User acceleration (excluding gravity)
            let userAccel = ImuReading(
                timestamp: self?.hostTimeToNanos(motion.timestamp) ?? 0,
                sensorType: .userAcceleration,
                x: motion.userAcceleration.x * 9.81,
                y: motion.userAcceleration.y * 9.81,
                z: motion.userAcceleration.z * 9.81
            )

            // Rotation rate
            let rotationRate = ImuReading(
                timestamp: self?.hostTimeToNanos(motion.timestamp) ?? 0,
                sensorType: .rotationRate,
                x: motion.rotationRate.x,
                y: motion.rotationRate.y,
                z: motion.rotationRate.z
            )

            // Attitude (quaternion)
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

    // CoreMotion timestamp → nanoseconds
    private func hostTimeToNanos(_ timestamp: TimeInterval) -> Int64 {
        // CoreMotion timestamp is in seconds since system boot
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

### 3.2 Data Structures

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
    let timestamp: Int64    // nanoseconds
    let sensorType: SensorType
    let x: Double
    let y: Double
    let z: Double

    // CSV format output
    var csvLine: String {
        return "\(timestamp),\(sensorType.rawValue),\(x),\(y),\(z)"
    }
}

// Extension: Combined IMU frame
struct ImuFrame: Codable {
    let timestamp: Int64
    let accelerometer: SIMD3<Double>  // m/s²
    let gyroscope: SIMD3<Double>      // rad/s
    let magnetometer: SIMD3<Double>?  // μT (optional)
}
```

### 3.3 High-Frequency Sampling (CMBatchedSensorManager)

```swift
// Available on iOS 17+, Apple Watch Series 8+
import CoreMotion

@available(iOS 17.0, *)
class HighFrequencyMotionService {

    private let batchedManager = CMBatchedSensorManager()

    func startHighFrequencySampling() async throws {
        // Max 800Hz accelerometer, 200Hz device motion
        // Note: HealthKit workout session required

        for await batch in batchedManager.accelerometerUpdates() {
            for data in batch {
                // data.timestamp, data.acceleration
                print("Accel: \(data.acceleration) at \(data.timestamp)")
            }
        }
    }
}
```

## 4. Camera-IMU Synchronization

### 4.1 Timestamp Basis

```swift
/*
 iOS timestamp basis:

 1. CMSampleBuffer (Camera)
    - CMSampleBufferGetPresentationTimeStamp()
    - Based on hostTime (mach_absolute_time conversion)

 2. CoreMotion (IMU)
    - CMAccelerometerData.timestamp
    - CMDeviceMotion.timestamp
    - Seconds since system boot (TimeInterval)

 Both timestamps are based on the same system clock, so direct comparison is possible
*/

class TimestampSynchronizer {

    // Convert hostTime to nanoseconds
    func hostTimeToNanos() -> Int64 {
        var timebaseInfo = mach_timebase_info_data_t()
        mach_timebase_info(&timebaseInfo)

        let hostTime = mach_absolute_time()
        let nanos = hostTime * UInt64(timebaseInfo.numer) / UInt64(timebaseInfo.denom)
        return Int64(nanos)
    }

    // CoreMotion timestamp (seconds) → nanoseconds
    func motionTimestampToNanos(_ timestamp: TimeInterval) -> Int64 {
        return Int64(timestamp * 1_000_000_000)
    }

    // CMTime → nanoseconds
    func cmTimeToNanos(_ time: CMTime) -> Int64 {
        return Int64(CMTimeGetSeconds(time) * 1_000_000_000)
    }
}
```

### 4.2 Synchronized Data Collection

```swift
class SynchronizedDataCollector {

    private let cameraService = CameraService()
    private let motionService = MotionService()
    private let synchronizer = TimestampSynchronizer()

    private var frameMetadataList: [FrameMetadata] = []
    private var imuDataList: [ImuReading] = []

    func startCollection() throws {
        // Camera frame callback
        cameraService.onFrameCaptured = { [weak self] sampleBuffer, pixelBuffer, time in
            let timestamp = self?.synchronizer.cmTimeToNanos(time) ?? 0

            // Save frame and record metadata
            self?.processFrame(pixelBuffer: pixelBuffer, timestamp: timestamp)
        }

        // IMU data callback
        motionService.onImuData = { [weak self] reading in
            self?.imuDataList.append(reading)
        }

        // Start
        try cameraService.setupCamera()
        try cameraService.configureForVISLAM()
        motionService.startRawSensors(frequency: 200.0)
        cameraService.startCapture()
    }

    func stopCollection() {
        cameraService.stopCapture()
        motionService.stop()
    }

    // Match IMU data close to frame
    func matchImuToFrame(frameTimestamp: Int64,
                         windowNs: Int64 = 50_000_000) -> [ImuReading] {
        return imuDataList.filter { imu in
            abs(imu.timestamp - frameTimestamp) <= windowNs
        }
    }

    private func processFrame(pixelBuffer: CVPixelBuffer, timestamp: Int64) {
        // Frame processing logic
    }
}
```

## 5. Data Storage

### 5.1 File Storage

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

## 6. iOS Coordinate System

```
iOS device coordinate system (portrait mode, home button at bottom):

        Y (up, towards ear)
        ^
        |
        |
        +-------> X (right)
       /
      /
     v
    Z (towards user, out of screen)

- Right-handed coordinate system
- Same convention as Android
```

## 7. Permission Settings

### Info.plist

```xml
<key>NSCameraUsageDescription</key>
<string>Camera access is required for VI-SLAM data collection.</string>

<key>NSMotionUsageDescription</key>
<string>Motion sensor access is required for VI-SLAM data collection.</string>

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
