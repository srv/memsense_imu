include_directories(${PROJECT_SOURCE_DIR}/src)

set(BASE_SOURCES
      imu_node_base.cpp
      imu_filter.cpp )

rosbuild_add_executable(imu_node imu_node.cpp ${BASE_SOURCES})
target_link_libraries(imu_node IMUSampler)

rosbuild_add_library(imu_nodelet imu_nodelet.cpp ${BASE_SOURCES})
target_link_libraries(imu_nodelet IMUSampler)
