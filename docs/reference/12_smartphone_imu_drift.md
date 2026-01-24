# Smartphone IMU Drift: Causes, Characteristics, and Mitigation

## 1. Overview

MEMS (Micro-Electro-Mechanical Systems) IMUs embedded in smartphones are mass-produced at low cost, resulting in performance limitations. Drift phenomena in particular have a direct impact on the accuracy of VI-SLAM systems.

### Why Are Smartphone IMUs Problematic?

| Characteristic | High-End IMU (Tactical Grade) | Smartphone MEMS |
|------|--------------------------|---------------|
| Price | $1,000 - $10,000+ | $1 - $5 |
| Bias Stability | 0.01 - 1 deg/h | 10 - 1000 deg/h |
| Noise Density (Gyro) | 0.001 - 0.01 deg/s/sqrt(Hz) | 0.005 - 0.05 deg/s/sqrt(Hz) |
| Position Error After 60s | < 1m | > 150m |

**Core Issue**: In a simple MEMS IMU-based INS, average position error can exceed 150m after 60 seconds of operation.

---

## 2. IMU Error Types and Causes

### 2.1 Deterministic Errors

Systematic errors that can be removed through calibration.

#### Bias
```
Measurement = True Value + Bias

Accelerometer: a_measured = a_true + b_a
Gyroscope: omega_measured = omega_true + b_g
```

**Impact**:
- 1mg accelerometer bias -> 0.5mm position error after 1s, **5m error** after 100s
- 1 deg/h gyro bias -> 1 degree attitude error after 1 hour

#### Scale Factor Error
```
Measurement = (1 + deltaS) x True Value

deltaS: Scale factor error (typically 0.1% ~ 1%)
```

**Smartphone MEMS Characteristics**:
- Uncalibrated scale factor error: up to 10% (0.1)
- After calibration: 0.1% ~ 1%

#### Axis Misalignment / Cross-coupling
```
    [ 1      delta_xy   delta_xz ]
M = [ delta_yx   1      delta_yz ]
    [ delta_zx   delta_zy   1    ]

delta: Misalignment angle (typically 0.01 ~ 0.02 rad)
```

### 2.2 Stochastic Errors

Random errors that cannot be removed through calibration.

#### White Noise
```
Noise caused by high-frequency thermoelectric reactions

Accelerometer: Velocity Random Walk (VRW) - m/s/sqrt(Hz)
Gyroscope: Angle Random Walk (ARW) - rad/s/sqrt(Hz)
```

**Typical Smartphone MEMS Values**:
```yaml
# Low-cost smartphone IMU
accelerometer_noise_density: 0.01     # m/s^2/sqrt(Hz)
gyroscope_noise_density: 0.005        # rad/s/sqrt(Hz) (approx. 0.3 deg/s/sqrt(Hz))
```

#### Bias Instability
```
Phenomenon where sensor bias slowly varies over time
= "In-run bias stability"

Physical limitation that occurs even at constant temperature
```

**Impact of Bias Instability**:
- Most important IMU specification
- Main cause of accumulated error during long-term operation

#### Bias Random Walk
```
Bias randomly walks over time

Accelerometer: sigma_ba (m/s^3/sqrt(Hz))
Gyroscope: sigma_bg (rad/s^2/sqrt(Hz))
```

**Typical Smartphone MEMS Values**:
```yaml
accelerometer_random_walk: 0.0002    # m/s^3/sqrt(Hz)
gyroscope_random_walk: 4.0e-06       # rad/s^2/sqrt(Hz)
```

### 2.3 Environment-Dependent Errors

#### Temperature Effects
```
Bias = b_0 + k_T x deltaT

k_T: Temperature coefficient (deg/h/degC or mg/degC)
deltaT: Temperature change
```

**Smartphone Characteristics**:
- Temperature rise during use (processor heat)
- Bias drift occurs with temperature changes

#### g-Sensitivity (Acceleration Sensitivity)
```
Effect of acceleration on MEMS gyroscope

Error = g-sensitivity x acceleration x time

Example: 20g, 10 seconds, 0.05 (deg/s)/g -> approximately 10 degree angle error
```

---

## 3. Allan Variance Analysis

Standard method for quantifying IMU noise characteristics.

### 3.1 Allan Variance Overview

```
              +--------------------------------------------+
              |         Allan Deviation Plot               |
log(sigma(tau))                                            |
    |         |     ARW                                    |
    |         |    slope = -1/2                           |
    |         |        \                                   |
    |         |         \     Bias Instability            |
    |         |          \____  (minimum, slope ~ 0)      |
    |         |               \_____                       |
    |         |                     \___  RRW              |
    |         |                         \ slope = +1/2    |
    +---------+--------------------------------------------+
                              log(tau)
```

### 3.2 Key Parameter Extraction

| Parameter | Allan Plot Characteristic | Meaning |
|----------|----------------|------|
| ARW (N) | slope = -1/2, value at tau=1 | White noise (high frequency) |
| Bias Instability (B) | Minimum point (slope ~ 0) | Long-term stability |
| RRW (K) | slope = +1/2 | Bias random walk |

### 3.3 Data Collection and Analysis

```python
# Data collection for Allan Variance analysis
# Recommended: 15-24 hours of stationary recording

import numpy as np
from allantools import adev

def compute_allan_variance(gyro_data, sample_rate):
    """
    Args:
        gyro_data: Gyroscope data (rad/s)
        sample_rate: Sampling frequency (Hz)

    Returns:
        tau: Cluster time array
        adev: Allan deviation array
    """
    # Calculate Allan deviation
    tau, adev_values, _, _ = adev(
        gyro_data,
        rate=sample_rate,
        data_type="freq"
    )

    return tau, adev_values

def extract_noise_params(tau, adev):
    """
    Extract noise parameters from Allan deviation plot
    """
    # ARW: Value on slope=-1/2 line at tau=1
    arw_idx = np.argmin(np.abs(tau - 1.0))
    arw = adev[arw_idx]

    # Bias Instability: Minimum value
    bias_instability = np.min(adev)

    return {
        'arw': arw,          # rad/s/sqrt(Hz)
        'bias_instability': bias_instability  # rad/s
    }
```

### 3.4 Measurement Results by Smartphone (Research Data)

| Smartphone | Gyro Noise SD (rad/s) | Accel Noise SD (m/s^2) |
|----------|----------------------|------------------------|
| Google Pixel 7 Pro | 0.0018 | 0.0089 |
| iPhone XR | **0.0027** (max) | 0.0095 |
| Samsung A53 | 0.0021 | **0.0106** (max) |
| OnePlus 7 Pro | 0.0023 | 0.0098 |

**Source**: [Smartphone MEMS Accelerometer and Gyroscope Measurement Errors](https://www.mdpi.com/1424-8220/23/17/7609)

---

## 4. Drift Correction Techniques

### 4.1 Filter-Based Methods

#### Extended Kalman Filter (EKF)

```
State vector includes bias:
x = [position, velocity, orientation, gyro_bias, accel_bias]^T

Prediction step:
x_pred = f(x, u)  // State transition
P_pred = F*P*F^T + Q  // Covariance prediction

Update step (when visual measurement available):
K = P_pred*H^T*(H*P_pred*H^T + R)^(-1)
x = x_pred + K*(z - h(x_pred))
P = (I - K*H)*P_pred
```

**Python Implementation Example**:
```python
import numpy as np

class IMUBiasEKF:
    def __init__(self):
        # State: [roll, pitch, yaw, bg_x, bg_y, bg_z]
        self.x = np.zeros(6)

        # Covariance matrix
        self.P = np.eye(6) * 0.1

        # Process noise
        self.Q = np.diag([
            1e-4, 1e-4, 1e-4,    # Attitude noise
            1e-6, 1e-6, 1e-6     # Bias random walk
        ])

        # Measurement noise (visual measurement)
        self.R = np.eye(3) * 0.01

    def predict(self, gyro, dt):
        """
        IMU prediction step
        """
        # Bias-corrected angular velocity
        omega = gyro - self.x[3:6]

        # Attitude update
        self.x[0:3] += omega * dt

        # State transition matrix
        F = np.eye(6)
        F[0:3, 3:6] = -np.eye(3) * dt

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, visual_orientation):
        """
        Update with visual measurement
        """
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)

        # Innovation
        y = visual_orientation - self.x[0:3]

        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

        return self.x[3:6]  # Return estimated bias
```

#### Error State Kalman Filter (ESKF)

A more commonly used method in VI-SLAM.

```
True state = Nominal state (+) Error state

x_true = x_nominal (+) delta_x

Advantages:
- Improved linearization accuracy for small errors
- Easier to avoid singularities
```

### 4.2 Bias Handling in VI-SLAM Frameworks

#### VINS-Mono / VINS-Fusion

```cpp
// IMU Preintegration with Bias
class IMUPreintegration {
    // Linearization for bias changes
    // Jacobian: d(preintegration)/d(bias)

    Matrix3d J_bg;  // gyro bias jacobian
    Matrix3d J_ba;  // accel bias jacobian

    void repropagate(const Vector3d& new_bg,
                     const Vector3d& new_ba) {
        // Fast recalculation using first-order approximation when bias changes
        delta_p += J_ba * (new_ba - linearized_ba);
        delta_v += J_ba * (new_ba - linearized_ba);
        delta_q = delta_q * Quaternion(J_bg * (new_bg - linearized_bg));
    }
};
```

**VINS-Mono Bias Estimation Features**:
- Online automatic bias estimation
- Simultaneous time offset estimation (`estimate_td: 1`)
- Asynchronous sensor handling support

#### OpenVINS

```
Based on MSCKF (Multi-State Constraint Kalman Filter)

State vector:
x = [q, p, v, bg, ba, ..., camera_states]

Features:
- On-manifold sliding window Kalman filter
- Online camera intrinsic/extrinsic calibration
- Camera-IMU time offset calibration
```

**Reference**: [OpenVINS Documentation](https://docs.openvins.com/)

### 4.3 Deep Learning-Based Methods

#### IMU Denoising Network

```
Input: Raw IMU sequence [N x 6] (acceleration + gyro)
Output: Denoised IMU or corrected bias

Architecture:
+-------------+    +--------------+    +-------------+
| Raw IMU     |-->| LSTM / TCN   |-->| Denoised    |
| Sequence    |    | Network      |    | IMU         |
+-------------+    +--------------+    +-------------+
```

**Key Research**:
- **CNN-based**: Time-segmentation method to fit IMU output to CNN input format
- **LSTM/GRU Hybrid**: Up to 72% attitude error reduction
- **NGC-Net**: Gyroscope calibration using Temporal Convolutional Network

```python
import torch
import torch.nn as nn

class IMUDenoiser(nn.Module):
    """
    LSTM-based IMU denoising network
    """
    def __init__(self, input_size=6, hidden_size=128, num_layers=2):
        super().__init__()

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            bidirectional=True
        )

        self.fc = nn.Sequential(
            nn.Linear(hidden_size * 2, 64),
            nn.ReLU(),
            nn.Linear(64, 6)  # Denoised IMU
        )

    def forward(self, x):
        # x: [batch, seq_len, 6]
        lstm_out, _ = self.lstm(x)
        # lstm_out: [batch, seq_len, hidden*2]

        output = self.fc(lstm_out)
        return output

# Training data: Ground Truth IMU or visual-inertial fusion results
```

#### Bias Prediction Network (IPNet)

```
Can be plugged into VINS-Mono/OpenVINS

Raw IMU -> IPNet -> Average bias estimation
                      |
              Used as initial value for traditional VIO

Advantages:
- Removes dependency on traditional recursive prediction
- Reduces initial bias error impact
```

**Reference**: [A Plug-and-Play Learning-based IMU Bias Factor](https://arxiv.org/html/2503.12527v1)

---

## 5. Practical Mitigation Strategies

### 5.1 Calibration Pipeline

```
+--------------------------------------------------------------+
|               IMU Calibration Workflow                        |
+--------------------------------------------------------------+
|                                                              |
|  1. Stationary bias measurement (15-24 hours)                |
|     +-> Allan Variance analysis                              |
|     +-> Noise parameter extraction (ARW, Bias Instability, RRW)|
|                                                              |
|  2. Dynamic calibration (Kalibr)                             |
|     +-> Camera-IMU extrinsic parameters                      |
|     +-> Time offset                                          |
|                                                              |
|  3. Online bias estimation                                   |
|     +-> Real-time bias updates in VINS/OpenVINS              |
|                                                              |
+--------------------------------------------------------------+
```

### 5.2 Using Uncalibrated Sensors on Android

```kotlin
// Uncalibrated sensors recommended for VI-SLAM
// Android's automatic calibration may be unsuitable for VI-SLAM

val gyroscopeUncalibrated = sensorManager.getDefaultSensor(
    Sensor.TYPE_GYROSCOPE_UNCALIBRATED
)

val accelerometerUncalibrated = sensorManager.getDefaultSensor(
    Sensor.TYPE_ACCELEROMETER_UNCALIBRATED
)

// Uncalibrated sensor data structure
// values[0-2]: Measurements
// values[3-5]: Android estimated bias (for reference)
```

**Why Use Uncalibrated Sensors?**
- Android's automatic calibration conflicts with VI-SLAM's bias estimation
- Enables consistent bias model application
- Leverages VI-SLAM framework's online calibration

### 5.3 Recommended VI-SLAM Framework Settings

#### VINS-Mono Settings

```yaml
# config/smartphone.yaml

# IMU parameters (reflecting Allan Variance results)
acc_n: 0.1          # Accelerometer noise (m/s^2)
gyr_n: 0.01         # Gyroscope noise (rad/s)
acc_w: 0.002        # Accelerometer random walk
gyr_w: 0.0002       # Gyroscope random walk

# Enable online calibration
estimate_extrinsic: 2   # Online extrinsic parameter estimation
estimate_td: 1          # Online time offset estimation
td: 0.02                # Initial time offset estimate (seconds)
```

#### OpenVINS Settings

```yaml
# config/smartphone/estimator_config.yaml

# IMU noise parameters
gyroscope_noise_density: 0.005      # rad/s/sqrt(Hz)
accelerometer_noise_density: 0.01   # m/s^2/sqrt(Hz)
gyroscope_random_walk: 4.0e-06      # rad/s^2/sqrt(Hz)
accelerometer_random_walk: 0.0002   # m/s^3/sqrt(Hz)

# Calibration options
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true

# Bias initialization
init_window_time: 1.0       # Initialization window (seconds)
init_imu_thresh: 1.0        # IMU movement threshold
```

### 5.4 Drift Detection and Recovery

```kotlin
class DriftDetector(
    private val maxBiasGyro: Float = 0.1f,      // rad/s
    private val maxBiasAccel: Float = 1.0f,     // m/s^2
    private val maxVelocityStatic: Float = 0.1f // m/s
) {
    private var lastValidState: SLAMState? = null

    fun checkDrift(currentState: SLAMState): DriftStatus {
        // 1. Detect bias anomaly
        if (currentState.gyroBias.norm() > maxBiasGyro ||
            currentState.accelBias.norm() > maxBiasAccel) {
            return DriftStatus.BIAS_ANOMALY
        }

        // 2. Detect velocity drift in stationary state
        if (currentState.isStationary &&
            currentState.velocity.norm() > maxVelocityStatic) {
            return DriftStatus.VELOCITY_DRIFT
        }

        // 3. Detect scale drift (based on visual features)
        if (detectScaleDrift(currentState)) {
            return DriftStatus.SCALE_DRIFT
        }

        lastValidState = currentState
        return DriftStatus.NORMAL
    }

    fun recoverFromDrift(status: DriftStatus): RecoveryAction {
        return when (status) {
            DriftStatus.BIAS_ANOMALY -> RecoveryAction.REINITIALIZE_BIAS
            DriftStatus.VELOCITY_DRIFT -> RecoveryAction.ZERO_VELOCITY_UPDATE
            DriftStatus.SCALE_DRIFT -> RecoveryAction.VISUAL_RELOCALIZATION
            DriftStatus.NORMAL -> RecoveryAction.NONE
        }
    }
}

enum class DriftStatus {
    NORMAL, BIAS_ANOMALY, VELOCITY_DRIFT, SCALE_DRIFT
}

enum class RecoveryAction {
    NONE, REINITIALIZE_BIAS, ZERO_VELOCITY_UPDATE, VISUAL_RELOCALIZATION
}
```

---

## 6. Performance Expectations

### 6.1 Pure INS vs VI-SLAM

| Time | Pure MEMS INS Error | VI-SLAM Error |
|------|------------------|--------------|
| 10s | ~5m | < 0.1m |
| 60s | **> 150m** | < 0.5m |
| 10min | Several km | < 2m |

### 6.2 Expected Performance by Smartphone Tier

| Smartphone Tier | VI-SLAM Accuracy | Key Limitations |
|-------------|---------------|----------|
| Flagship (Latest) | 0.5 - 2% of travel distance | Rolling Shutter, OIS |
| Mid-range | 1 - 5% of travel distance | High IMU noise |
| Budget | 3 - 10% of travel distance | Unstable sampling, high drift |

**Note**: Accuracy can vary significantly depending on environment, motion patterns, and lighting conditions.

---

## 7. Key Challenges and Solutions Summary

| Challenge | Impact | Solution |
|------|------|--------|
| Gyro bias drift | Attitude error accumulation | Observe/correct bias with visual measurements |
| Accelerometer bias | Rapid position error growth | Online estimation via EKF/optimization |
| Temperature changes | Bias variation | Temperature compensation model, frequent recalibration |
| High-frequency noise | Short-term accuracy degradation | Low-pass filter, deep learning denoising |
| g-sensitivity | Error during fast motion | Motion pattern constraints, compensation model |
| Rolling Shutter | Distortion during fast rotation | RS-supporting frameworks (VINS, OpenVINS) |

---

## References

### Papers and Technical Reports
- [Smartphone MEMS Accelerometer and Gyroscope Measurement Errors](https://www.mdpi.com/1424-8220/23/17/7609)
- [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator](https://arxiv.org/abs/1708.03852)
- [OpenVINS: A Research Platform for Visual-Inertial Estimation](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)
- [Deep Learning for Inertial Positioning: A Survey](https://arxiv.org/html/2303.03757v3)
- [A Plug-and-Play Learning-based IMU Bias Factor](https://arxiv.org/html/2503.12527v1)
- [Inertial Navigation Meets Deep Learning: A Survey](https://arxiv.org/html/2307.00014v2)

### Tools and Libraries
- [Kalibr IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [imu_utils - Allan Variance Analysis](https://github.com/gaowenliang/imu_utils)
- [OpenVINS Calibration Guide](https://docs.openvins.com/gs-calibration.html)
- [MATLAB Allan Variance Analysis](https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html)

### Related Documents
- [04_calibration_synchronization.md](04_calibration_synchronization.md)
- [07_android_sensor_api.md](07_android_sensor_api.md)
- [05_vislam_frameworks.md](05_vislam_frameworks.md)
