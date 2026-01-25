cmake_minimum_required(VERSION 3.16)

# E2E Test Fixture Library
add_library(e2e_test_fixture
    e2e_test_fixture.cpp
    latency_measurement.cpp
)

target_include_directories(e2e_test_fixture PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(e2e_test_fixture
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

# Basic Streaming Test
add_executable(test_basic_streaming
    test_basic_streaming.cpp
)

target_link_libraries(test_basic_streaming
    e2e_test_fixture
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_basic_streaming COMMAND test_basic_streaming)

# EuRoC Benchmark Test
add_executable(test_euroc_benchmark
    test_euroc_benchmark.cpp
)

target_link_libraries(test_euroc_benchmark
    e2e_test_fixture
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_euroc_benchmark COMMAND test_euroc_benchmark)

# Stability Test
add_executable(test_stability
    test_stability.cpp
)

target_link_libraries(test_stability
    e2e_test_fixture
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_stability COMMAND test_stability)

# Framework Comparison Test
add_executable(test_framework_comparison
    test_framework_comparison.cpp
)

target_link_libraries(test_framework_comparison
    e2e_test_fixture
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_framework_comparison COMMAND test_framework_comparison)
