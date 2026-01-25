cmake_minimum_required(VERSION 3.16)
project(vi_slam VERSION 1.0.0 LANGUAGES CXX)

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

# Optional ROS support
option(ENABLE_ROS "Enable ROS integration" OFF)
if(ENABLE_ROS)
    find_package(roscpp REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    add_definitions(-DENABLE_ROS)
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

if(ENABLE_ROS)
    include_directories(
        ${roscpp_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
    )
endif()

# Source files
set(COMMON_SOURCES
    src/slam/slam_engine.cpp
    src/slam/adapters/vins_mono_adapter.cpp
    src/slam/adapters/openvins_adapter.cpp
    src/slam/adapters/orbslam3_adapter.cpp
    src/slam/output/trajectory_exporter.cpp
)

if(ENABLE_ROS)
    list(APPEND COMMON_SOURCES
        src/slam/output/ros_publisher.cpp
    )
endif()

# Library
add_library(${PROJECT_NAME} SHARED
    ${COMMON_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

if(ENABLE_ROS)
    target_link_libraries(${PROJECT_NAME}
        ${roscpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${tf2_LIBRARIES}
        ${tf2_ros_LIBRARIES}
    )
endif()

# Installation
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Testing
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Print configuration
message(STATUS "")
message(STATUS "VI-SLAM Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "  ROS support: ${ENABLE_ROS}")
message(STATUS "")
