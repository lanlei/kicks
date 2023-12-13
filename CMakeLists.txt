cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(ArnoldSequenceRender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# debug mode
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -Wall")
set(CMAKE_LINK_LIBRARY_FLAG_DEBUG "-g")

# release mode
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# profile mode
set(CMAKE_CXX_FLAGS_PROFILE "-g -O2")

include(cmake/3rdParty.cmake)

find_package(Arnold REQUIRED)
add_executable(cyber-supervisor cyber-supervisor.cc)
target_include_directories(cyber-supervisor PRIVATE ${ARNOLD_INCLUDE_DIR})
target_link_libraries(cyber-supervisor PRIVATE ${ARNOLD_LIBRARIES})
target_link_libraries(cyber-supervisor PRIVATE tinyobjloader_double)
target_link_libraries(cyber-supervisor PRIVATE Eigen3::Eigen)