cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME disable_lidar_workmode)
add_executable(${DEMO_NAME} disable_lidar.cpp)

target_link_libraries(${DEMO_NAME}
        PUBLIC
        livox_lidar_sdk_static)

