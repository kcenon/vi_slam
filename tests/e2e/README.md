# End-to-End Integration Tests

Comprehensive end-to-end integration tests for the VI-SLAM system.

## Overview

This test suite validates the complete VI-SLAM pipeline from Android device capture through SLAM processing on PC. Tests cover:

- **Basic Streaming**: E2E data flow validation with latency checks
- **EuRoC Benchmark**: Accuracy validation using standard datasets
- **Stability**: Long-duration robustness testing (30+ minutes)
- **Framework Comparison**: Multi-framework evaluation and hot-switching

## Prerequisites

### Build Requirements

- CMake 3.16+
- C++17 compiler
- OpenCV 4.x
- Eigen3
- All VI-SLAM dependencies

### Test Data (Optional)

- **EuRoC Dataset**: Download MH_01 sequence for accuracy benchmarking
  - Download: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/
  - Extract to `/tmp/euroc_mh_01/` or specify path

## Building Tests

```bash
# Configure with tests enabled
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON

# Build all tests
cmake --build build

# Tests are located in: build/tests/e2e/
```

## Running Tests

### Quick Start

Run basic tests only (fastest):

```bash
./tests/e2e/run_e2e_tests.sh --basic-only
```

### Full Test Suite

```bash
# Run all tests except stability (recommended for CI)
./tests/e2e/run_e2e_tests.sh

# Run all tests including 30-minute stability test
./tests/e2e/run_e2e_tests.sh --all --stability

# Run with EuRoC benchmark
./tests/e2e/run_e2e_tests.sh --euroc-path /path/to/euroc_mh_01
```

### Individual Tests

Run specific tests directly:

```bash
# Basic streaming test
./build/tests/e2e/test_basic_streaming

# EuRoC benchmark (requires dataset)
./build/tests/e2e/test_euroc_benchmark /path/to/euroc_mh_01

# Framework comparison
./build/tests/e2e/test_framework_comparison

# Stability test (30 minutes)
./build/tests/e2e/test_stability
```

### Using CTest

```bash
cd build

# Run all E2E tests
ctest -R "test_.*" --output-on-failure

# Run specific test
ctest -R "test_basic_streaming" --output-on-failure

# Exclude long-running tests
ctest --exclude-regex "test_stability|test_euroc_benchmark" --output-on-failure
```

## Test Descriptions

### 1. Basic Streaming Test

**File**: `test_basic_streaming.cpp`

**Purpose**: Validates basic E2E data flow

**Requirements**:
- Average latency < 100ms (p95)
- Frame drop rate < 1%
- No crashes or hangs

**Duration**: ~10 seconds

**Usage**:
```bash
./build/tests/e2e/test_basic_streaming
```

### 2. EuRoC Benchmark Test

**File**: `test_euroc_benchmark.cpp`

**Purpose**: Validates SLAM accuracy using standard datasets

**Requirements**:
- ATE RMSE < 0.1m on EuRoC MH_01
- Trajectory completeness > 95%
- No tracking failures

**Duration**: ~5 minutes (depends on dataset size)

**Usage**:
```bash
./build/tests/e2e/test_euroc_benchmark /path/to/euroc_mh_01 [ground_truth.csv]
```

**Arguments**:
- `arg1`: Path to EuRoC sequence directory
- `arg2` (optional): Path to ground truth CSV file

### 3. Stability Test

**File**: `test_stability.cpp`

**Purpose**: Long-duration robustness validation

**Requirements**:
- Run continuously for 30 minutes
- No crashes or memory leaks
- Drop rate < 1%
- Stable latency throughout

**Duration**: ~30 minutes

**Usage**:
```bash
./build/tests/e2e/test_stability
```

**Warning**: This test is resource-intensive and time-consuming. Recommended to run separately from CI.

### 4. Framework Comparison Test

**File**: `test_framework_comparison.cpp`

**Purpose**: Multi-framework evaluation and hot-switching

**Requirements**:
- All frameworks initialize successfully
- Framework switch time < 5 seconds
- Consistent performance across frameworks

**Duration**: ~30 seconds per framework

**Usage**:
```bash
./build/tests/e2e/test_framework_comparison
```

## CI Integration

### GitHub Actions

E2E tests are integrated into the CI pipeline (`.github/workflows/ci.yml`):

```yaml
- name: Run E2E tests
  run: ./tests/e2e/run_e2e_tests.sh --basic-only
```

**Note**: Long-running tests (stability, EuRoC) are excluded from CI by default.

### Local Development

Recommended workflow for local development:

```bash
# Before committing: quick smoke test
./tests/e2e/run_e2e_tests.sh --basic-only

# Before merging: comprehensive test
./tests/e2e/run_e2e_tests.sh --euroc-path /path/to/euroc

# Before release: full validation
./tests/e2e/run_e2e_tests.sh --all --stability --euroc-path /path/to/euroc
```

## Test Results

Test results are saved to `tests/e2e/results/`:

```
results/
├── basic_streaming_20260125_143022.log
├── euroc_benchmark_20260125_143045.log
├── framework_comparison_20260125_143120.log
└── stability_20260125_150130.log
```

Each log contains:
- Test execution details
- Performance metrics
- Pass/fail status
- Error messages (if any)

## Troubleshooting

### Test Failures

**"Test setup failed"**
- Check that VI-SLAM is properly built
- Verify all dependencies are installed
- Ensure configuration files exist

**"Failed to load test data"**
- Some tests can generate synthetic data
- For EuRoC tests, provide dataset path
- Check file permissions

**"Latency exceeds target"**
- System may be under load
- Try running with `nice` priority
- Close other applications

**"ATE RMSE exceeds target"**
- Verify ground truth file format
- Check trajectory alignment
- May indicate SLAM accuracy issue

### Performance Issues

If tests are slow:
1. Build in Release mode: `-DCMAKE_BUILD_TYPE=Release`
2. Reduce test data size (edit test files)
3. Use faster system for CI

### Memory Issues

For stability test memory errors:
1. Close other applications
2. Increase swap space
3. Monitor with `htop` or `Activity Monitor`

## Development

### Adding New Tests

1. Create test file in `tests/e2e/`
2. Add executable in `CMakeLists.txt`
3. Link against `e2e_test_fixture` library
4. Add to `run_e2e_tests.sh`

Example CMakeLists.txt entry:

```cmake
add_executable(test_my_feature
    test_my_feature.cpp
)

target_link_libraries(test_my_feature
    e2e_test_fixture
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_my_feature COMMAND test_my_feature)
```

### Extending Test Fixture

The `E2ETestFixture` class provides common functionality:

```cpp
#include "e2e_test_fixture.hpp"

class MyTestFixture : public E2ETestFixture {
public:
    bool myCustomSetup() {
        // Custom initialization
        return true;
    }
};
```

## References

- Issue: [#30 - End-to-end integration testing](https://github.com/kcenon/vi_slam/issues/30)
- EuRoC Dataset: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/
- VI-SLAM Documentation: ../../docs/

## License

Same as the VI-SLAM project.
