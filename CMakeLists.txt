cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(ArnoldSequenceRender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Arnold REQUIRED)
add_executable(cyber-supervisor cyber-supervisor.cc)
message("-- ${ARNOLD_INCLUDE_DIR}")
message("-- ${ARNOLD_LIBRARIES}")
target_include_directories(cyber-supervisor PRIVATE ${ARNOLD_INCLUDE_DIR})
target_link_libraries(cyber-supervisor PRIVATE ${ARNOLD_LIBRARIES})
# target_compile_features(cyber-supervisor PRIVATE cxx_std_17)