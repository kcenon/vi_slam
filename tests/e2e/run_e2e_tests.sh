#!/bin/bash
# End-to-End Integration Test Runner
# Executes comprehensive E2E tests for VI-SLAM system

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test configuration
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${TEST_DIR}/../../build"
RESULTS_DIR="${TEST_DIR}/results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create results directory
mkdir -p "${RESULTS_DIR}"

echo "==================================="
echo "VI-SLAM E2E Integration Test Suite"
echo "==================================="
echo "Test directory: ${TEST_DIR}"
echo "Build directory: ${BUILD_DIR}"
echo "Results directory: ${RESULTS_DIR}"
echo "Timestamp: ${TIMESTAMP}"
echo ""

# Check if tests are built
if [ ! -f "${BUILD_DIR}/tests/e2e/test_basic_streaming" ]; then
    echo -e "${RED}ERROR: Tests not built. Please run cmake and make first.${NC}"
    exit 1
fi

# Test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Run a test and capture results
run_test() {
    local test_name=$1
    local test_executable=$2
    shift 2
    local test_args="$@"

    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    echo "-----------------------------------"
    echo "Running: ${test_name}"
    echo "-----------------------------------"

    local log_file="${RESULTS_DIR}/${test_name}_${TIMESTAMP}.log"

    if ${test_executable} ${test_args} > "${log_file}" 2>&1; then
        echo -e "${GREEN}✓ PASS${NC}: ${test_name}"
        PASSED_TESTS=$((PASSED_TESTS + 1))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}: ${test_name}"
        echo "  Log: ${log_file}"
        FAILED_TESTS=$((FAILED_TESTS + 1))

        # Show last 20 lines of log
        echo "  Last 20 lines of output:"
        tail -n 20 "${log_file}" | sed 's/^/    /'

        return 1
    fi
}

# Parse command line arguments
RUN_BASIC=true
RUN_EUROC=false
RUN_STABILITY=false
RUN_FRAMEWORK=true
EUROC_PATH=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --euroc-path)
            EUROC_PATH="$2"
            RUN_EUROC=true
            shift 2
            ;;
        --stability)
            RUN_STABILITY=true
            shift
            ;;
        --all)
            RUN_BASIC=true
            RUN_EUROC=true
            RUN_STABILITY=true
            RUN_FRAMEWORK=true
            shift
            ;;
        --basic-only)
            RUN_BASIC=true
            RUN_EUROC=false
            RUN_STABILITY=false
            RUN_FRAMEWORK=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--euroc-path <path>] [--stability] [--all] [--basic-only]"
            exit 1
            ;;
    esac
done

# Run tests
echo ""
echo "Starting E2E test execution..."
echo ""

# Test 1: Basic Streaming
if [ "$RUN_BASIC" = true ]; then
    run_test "basic_streaming" "${BUILD_DIR}/tests/e2e/test_basic_streaming"
fi

# Test 2: EuRoC Benchmark (conditional)
if [ "$RUN_EUROC" = true ]; then
    if [ -n "$EUROC_PATH" ]; then
        run_test "euroc_benchmark" "${BUILD_DIR}/tests/e2e/test_euroc_benchmark" "${EUROC_PATH}"
    else
        echo -e "${YELLOW}⊘ SKIP${NC}: euroc_benchmark (no dataset path provided)"
        echo "  Use --euroc-path <path> to enable this test"
    fi
fi

# Test 3: Framework Comparison
if [ "$RUN_FRAMEWORK" = true ]; then
    run_test "framework_comparison" "${BUILD_DIR}/tests/e2e/test_framework_comparison"
fi

# Test 4: Stability (long-running, optional)
if [ "$RUN_STABILITY" = true ]; then
    echo -e "${YELLOW}WARNING${NC}: Stability test will run for ~30 minutes"
    read -p "Continue? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        run_test "stability" "${BUILD_DIR}/tests/e2e/test_stability"
    else
        echo -e "${YELLOW}⊘ SKIP${NC}: stability (user canceled)"
    fi
fi

# Summary
echo ""
echo "==================================="
echo "Test Execution Summary"
echo "==================================="
echo "Total tests:  ${TOTAL_TESTS}"
echo -e "Passed:       ${GREEN}${PASSED_TESTS}${NC}"
if [ $FAILED_TESTS -gt 0 ]; then
    echo -e "Failed:       ${RED}${FAILED_TESTS}${NC}"
else
    echo -e "Failed:       ${FAILED_TESTS}"
fi
echo ""
echo "Results saved to: ${RESULTS_DIR}"

# Exit with failure if any test failed
if [ $FAILED_TESTS -gt 0 ]; then
    exit 1
else
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
fi
