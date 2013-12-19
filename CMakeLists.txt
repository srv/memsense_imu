cmake_minimum_required(VERSION 2.8.3)
project(memsense_imu)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs message_generation driver_base dynamic_reconfigure nodelet)

#uncomment if you have defined messages
add_message_files(
  DIRECTORY
  msg
  FILES
  ImuMAG.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

# auto-generated dynamic reconfiguration GUI
generate_dynamic_reconfigure_options(cfg/DynParams.cfg)

catkin_package()

include_directories(src cfg/cpp ${catkin_INCLUDE_DIRS})

add_library(IMUDataUtils 
            src/IMUDataUtils/UnknownIMUSample.cpp
            src/IMUDataUtils/IMUCounterUtils.cpp
            src/IMUDataUtils/DeviceParams.cpp
            src/IMUDataUtils/StandardIMUHeader.cpp
            src/IMUDataUtils/IIMUPayload.cpp
            src/IMUDataUtils/SDTDefaultPre.cpp
            src/IMUDataUtils/IMUHiGPayload.cpp
            src/IMUDataUtils/SensorDataUtils.cpp
            src/IMUDataUtils/SerialDeviceParams.cpp
            src/IMUDataUtils/SDTDefault.cpp
            src/IMUDataUtils/CommonUtils.cpp
            src/IMUDataUtils/STDIMUSample.cpp
            src/IMUDataUtils/IMUTempPayload.cpp
            src/IMUDataUtils/SerialComm.cpp
            src/IMUDataUtils/IMUUnknownPayload.cpp
            src/IMUDataUtils/IMUMAGPayload.cpp)

add_library(IMUSampler
            src/IMUSampler/IMUSampler.cpp
            src/IMUSampler/IMUSampler.h)
target_link_libraries(IMUSampler IMUDataUtils ${catkin_LIBRARIES})

add_executable(IDC
               src/IDC/AutoConfig.cpp
               src/IDC/AutoConfig.h
               src/IDC/GlobalVars.cpp
               src/IDC/GlobalVars.h
               src/IDC/GlobalVarsInternal.cpp
               src/IDC/GlobalVarsInternal.h
               src/IDC/IDCConfig.h
               src/IDC/IDCUtils.cpp
               src/IDC/IDCUtils.h
               src/IDC/IMenuHook.h
               src/IDC/IMUMsgsCommon.cpp
               src/IDC/IMUMsgsCommon.h
               src/IDC/IOHelpers.h
               src/IDC/IOHelpersImpl.h
               src/IDC/main.cpp
               src/IDC/MenuHelper.cpp
               src/IDC/MenuHelper.h
               src/IDC/MenuHelperInternal.cpp
               src/IDC/MenuHelperInternal.h
)

target_link_libraries(IDC IMUDataUtils ${catkin_LIBRARIES})

set(BASE_SOURCES
    src/nodes/imu_node_base.cpp
    src/nodes/imu_filter.cpp )

add_executable(imu_node src/nodes/imu_node.cpp ${BASE_SOURCES})
target_link_libraries(imu_node IMUSampler ${catkin_LIBRARIES})

add_library(imu_nodelet src/nodes/imu_nodelet.cpp ${BASE_SOURCES})
target_link_libraries(imu_nodelet IMUSampler ${catkin_LIBRARIES})




