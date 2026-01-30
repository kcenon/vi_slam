cmake_minimum_required(VERSION 3.16)
project(vi_slam_pc_client VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Find required packages
find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres QUIET)
find_package(glfw3 QUIET)

# Check if ZMQ is available
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ libzmq QUIET)
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

if(CERES_FOUND)
    include_directories(${CERES_INCLUDE_DIRS})
endif()

if(ZMQ_FOUND)
    include_directories(${ZMQ_INCLUDE_DIRS})
endif()

# Source files
set(PC_CLIENT_SOURCES
    src/main.cpp
    src/webrtc_receiver.cpp
)

# Visualizer sources (optional, only if GLFW is found)
if(glfw3_FOUND)
    set(VISUALIZER_SOURCES
        src/visualizer/renderer.cpp
        src/visualizer/point_cloud_renderer.cpp
        src/visualizer/trajectory_renderer.cpp
        src/visualizer/camera_controller.cpp
        src/visualizer/status_overlay.cpp
    )
endif()

# Executable
add_executable(${PROJECT_NAME}
    ${PC_CLIENT_SOURCES}
    ${VISUALIZER_SOURCES}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

if(CERES_FOUND)
    target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_CERES)
endif()

if(ZMQ_FOUND)
    target_link_libraries(${PROJECT_NAME} ${ZMQ_LINK_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${ZMQ_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_ZMQ)
endif()

if(glfw3_FOUND)
    target_link_libraries(${PROJECT_NAME} glfw)
    find_package(OpenGL REQUIRED)
    target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_VISUALIZER)
endif()

# Installation
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Testing
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_executable(test_webrtc_receiver
            tests/test_webrtc_receiver.cpp
            src/webrtc_receiver.cpp
        )
        target_link_libraries(test_webrtc_receiver
            ${OpenCV_LIBS}
            Eigen3::Eigen
            GTest::GTest
            GTest::Main
        )
        add_test(NAME webrtc_receiver_test COMMAND test_webrtc_receiver)

        # Visualizer tests (only if GLFW is found)
        if(glfw3_FOUND)
            add_executable(test_renderer
                tests/visualizer/test_renderer.cpp
                src/visualizer/renderer.cpp
            )
            target_link_libraries(test_renderer
                ${OpenCV_LIBS}
                Eigen3::Eigen
                glfw
                ${OPENGL_LIBRARIES}
                GTest::GTest
                GTest::Main
            )
            add_test(NAME renderer_test COMMAND test_renderer)

            add_executable(test_point_cloud_renderer
                tests/visualizer/test_point_cloud_renderer.cpp
                src/visualizer/renderer.cpp
                src/visualizer/point_cloud_renderer.cpp
            )
            target_link_libraries(test_point_cloud_renderer
                ${OpenCV_LIBS}
                Eigen3::Eigen
                glfw
                ${OPENGL_LIBRARIES}
                GTest::GTest
                GTest::Main
            )
            add_test(NAME point_cloud_renderer_test COMMAND test_point_cloud_renderer)

            add_executable(test_trajectory_renderer
                tests/visualizer/test_trajectory_renderer.cpp
                src/visualizer/renderer.cpp
                src/visualizer/trajectory_renderer.cpp
            )
            target_link_libraries(test_trajectory_renderer
                ${OpenCV_LIBS}
                Eigen3::Eigen
                glfw
                ${OPENGL_LIBRARIES}
                GTest::GTest
                GTest::Main
            )
            add_test(NAME trajectory_renderer_test COMMAND test_trajectory_renderer)

            add_executable(test_camera_controller
                tests/visualizer/test_camera_controller.cpp
                src/visualizer/camera_controller.cpp
            )
            target_link_libraries(test_camera_controller
                ${OpenCV_LIBS}
                Eigen3::Eigen
                GTest::GTest
                GTest::Main
            )
            add_test(NAME camera_controller_test COMMAND test_camera_controller)

            add_executable(test_status_overlay
                tests/visualizer/test_status_overlay.cpp
                src/visualizer/status_overlay.cpp
            )
            target_link_libraries(test_status_overlay
                ${OpenCV_LIBS}
                Eigen3::Eigen
                glfw
                ${OPENGL_LIBRARIES}
                GTest::GTest
                GTest::Main
            )
            add_test(NAME status_overlay_test COMMAND test_status_overlay)

            message(STATUS "  Visualizer tests: ENABLED")
        endif()

        message(STATUS "  Tests: ENABLED (GTest found)")
    else()
        message(STATUS "  Tests: DISABLED (GTest not found)")
    endif()
else()
    message(STATUS "  Tests: DISABLED")
endif()

# Python bindings (optional)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development)
    if(Python3_FOUND)
        message(STATUS "Python3 found: ${Python3_VERSION}")
        message(STATUS "Python bindings will be built in python/ directory")
    endif()
endif()

# Print configuration
message(STATUS "")
message(STATUS "VI-SLAM PC Client Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  Eigen3 version: ${Eigen3_VERSION}")
if(CERES_FOUND)
    message(STATUS "  Ceres version: ${CERES_VERSION}")
else()
    message(STATUS "  Ceres: NOT FOUND (optional)")
endif()
if(ZMQ_FOUND)
    message(STATUS "  ZMQ found: YES")
else()
    message(STATUS "  ZMQ: NOT FOUND (optional)")
endif()
if(glfw3_FOUND)
    message(STATUS "  GLFW3 found: YES")
    message(STATUS "  Visualizer module: ENABLED")
else()
    message(STATUS "  GLFW3: NOT FOUND (optional)")
    message(STATUS "  Visualizer module: DISABLED")
endif()
message(STATUS "")
