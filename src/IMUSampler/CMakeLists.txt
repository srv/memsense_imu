include_directories(${PROJECT_SOURCE_DIR}/src)
add_library(IMUSampler
               IMUSampler.cpp
               IMUSampler.h
)
target_link_libraries(IMUSampler IMUDataUtils)
