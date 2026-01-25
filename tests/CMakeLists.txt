cmake_minimum_required(VERSION 3.16)

# Test sources
add_executable(test_vins_mono_adapter
    test_vins_mono_adapter.cpp
)

target_link_libraries(test_vins_mono_adapter
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_vins_mono_adapter COMMAND test_vins_mono_adapter)

# OpenVINSAdapter test
add_executable(test_openvins_adapter
    test_openvins_adapter.cpp
)

target_link_libraries(test_openvins_adapter
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_openvins_adapter COMMAND test_openvins_adapter)

# ORBSLAM3Adapter test
add_executable(test_orbslam3_adapter
    test_orbslam3_adapter.cpp
)

target_link_libraries(test_orbslam3_adapter
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_orbslam3_adapter COMMAND test_orbslam3_adapter)

# SLAMEngine test
add_executable(test_slam_engine
    test_slam_engine.cpp
)

target_link_libraries(test_slam_engine
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_slam_engine COMMAND test_slam_engine)

# TrajectoryExporter test
add_executable(test_trajectory_exporter
    slam/output/test_trajectory_exporter.cpp
)

target_link_libraries(test_trajectory_exporter
    ${PROJECT_NAME}
)

add_test(NAME test_trajectory_exporter COMMAND test_trajectory_exporter)

# E2E tests
add_subdirectory(e2e)
