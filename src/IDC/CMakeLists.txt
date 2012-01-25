include_directories(..)
rosbuild_add_executable(IDC
                          AutoConfig.cpp
                          AutoConfig.h
                          CMakeLists.txt
                          GlobalVars.cpp
                          GlobalVars.h
                          GlobalVarsInternal.cpp
                          GlobalVarsInternal.h
                          IDCConfig.h
                          IDCUtils.cpp
                          IDCUtils.h
                          IMenuHook.h
                          IMUMsgsCommon.cpp
                          IMUMsgsCommon.h
                          IOHelpers.h
                          IOHelpersImpl.h
                          main.cpp
                          MenuHelper.cpp
                          MenuHelper.h
                          MenuHelperInternal.cpp
                          MenuHelperInternal.h
)

target_link_libraries(IDC IMUDataUtils)