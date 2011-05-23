include_directories(${PROJECT_SOURCE_DIR}/src)
rosbuild_add_executable(imu_node imu_node.cpp )
target_link_libraries(imu_node IMUSampler)
