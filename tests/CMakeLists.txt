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
