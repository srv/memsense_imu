include_directories(${PROJECT_SOURCE_DIR}/src)
rosbuild_add_executable(imu_node
                          imu_node.cpp
                          imu_base_node.h
                          imu_base_node.cpp
                          imu_filter.h
                          imu_filter.cpp
                          imu_sample.h )
target_link_libraries(imu_node IMUSampler)
