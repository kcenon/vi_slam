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

# BasaltAdapter test
add_executable(test_basalt_adapter
    test_basalt_adapter.cpp
)

target_link_libraries(test_basalt_adapter
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_test(NAME test_basalt_adapter COMMAND test_basalt_adapter)

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

# PointCloudExporter test
add_executable(test_pointcloud_exporter
    slam/output/test_pointcloud_exporter.cpp
)

target_link_libraries(test_pointcloud_exporter
    ${PROJECT_NAME}
)

add_test(NAME test_pointcloud_exporter COMMAND test_pointcloud_exporter)

# ROSPublisher test (conditional on ROS support)
if(ENABLE_ROS)
    find_package(GTest REQUIRED)

    add_executable(test_ros_publisher
        slam/output/test_ros_publisher.cpp
    )

    target_link_libraries(test_ros_publisher
        ${PROJECT_NAME}
        ${roscpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        GTest::GTest
        GTest::Main
    )

    add_test(NAME test_ros_publisher COMMAND test_ros_publisher)

    # TFPublisher test
    add_executable(test_tf_publisher
        slam/output/test_tf_publisher.cpp
    )

    target_link_libraries(test_tf_publisher
        ${PROJECT_NAME}
        ${roscpp_LIBRARIES}
        ${tf2_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        GTest::GTest
        GTest::Main
    )

    add_test(NAME test_tf_publisher COMMAND test_tf_publisher)
endif()

# ZMQPublisher test (conditional on ZMQ support)
if(ENABLE_ZMQ)
    find_package(GTest REQUIRED)

    add_executable(test_zmq_publisher
        slam/output/test_zmq_publisher.cpp
    )

    target_link_libraries(test_zmq_publisher
        ${PROJECT_NAME}
        cppzmq
        nlohmann_json::nlohmann_json
        GTest::GTest
        GTest::Main
    )

    add_test(NAME test_zmq_publisher COMMAND test_zmq_publisher)
endif()

# Visualizer test (conditional on Visualizer support)
if(ENABLE_VISUALIZER)
    add_executable(test_visualizer
        visualizer/test_visualizer.cpp
    )

    target_link_libraries(test_visualizer
        ${PROJECT_NAME}
        glfw
        ${OPENGL_LIBRARIES}
    )

    add_test(NAME test_visualizer COMMAND test_visualizer)
endif()

# E2E tests
add_subdirectory(e2e)
