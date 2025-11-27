cmake_minimum_required(VERSION 3.10)
project(small_point_lio)

# using c++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable color output
set(CMAKE_COLOR_DIAGNOSTICS ON)

# export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# enable Compile warning
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
elseif (MSVC)
    add_compile_options("/W4")
    add_compile_options("/wd4100")
endif()

# fix issues in MSVC
if (MSVC)
    add_compile_options("/utf-8")
endif ()

# optimize performance
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-march=native")
    add_compile_options("-ffast-math")
    add_compile_options("-fno-math-errno")
endif()

# find Packages
find_package(spdlog REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)
find_package(Pangolin REQUIRED QUIET)
find_package(OpenMP REQUIRED QUIET)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(livox_ros_driver2_DIR livox_ros_driver2/share/livox_ros_driver2/cmake)
find_package(rosbag2_cpp REQUIRED QUIET)
find_package(rosbag2_storage REQUIRED QUIET)
find_package(livox_ros_driver2 REQUIRED QUIET)
find_package(sensor_msgs REQUIRED QUIET)

configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

file(GLOB_RECURSE SOURCE src/*.cpp 3rdparty/*.c 3rdparty/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE})
target_precompile_headers(${PROJECT_NAME} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/small_point_lio/pch.h>")
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    3rdparty
    src
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
    ${rosbag2_cpp_INCLUDE_DIRS}
    ${rosbag2_storage_INCLUDE_DIRS}
    ${livox_ros_driver2_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    ${Pangolin_LIBRARIES} 
    ${rosbag2_cpp_LIBRARIES}
    ${rosbag2_storage_LIBRARIES}
    ${livox_ros_driver2_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)
