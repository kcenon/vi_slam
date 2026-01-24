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
