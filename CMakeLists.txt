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
set(livox_ros_driver2_DIR livox_ros_driver2/share/livox_ros_driver2/cmake)
find_package(spdlog REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(rosbag2_cpp REQUIRED QUIET)
find_package(rosbag2_storage REQUIRED QUIET)
find_package(sensor_msgs REQUIRED QUIET)
find_package(pcl_conversions REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)
find_package(Pangolin REQUIRED QUIET)
find_package(livox_ros_driver2 REQUIRED QUIET)
find_package(OpenMP REQUIRED QUIET)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

file(GLOB_RECURSE SOURCE src/*.h src/*.hpp src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE})
target_precompile_headers(${PROJECT_NAME} PRIVATE include/small_point_lio/pch.h)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${pcl_conversions_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    Eigen3::Eigen
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
    sensor_msgs::sensor_msgs_library
    yaml-cpp::yaml-cpp
    livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp
    livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp
    ${livox_ros_driver2_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Pangolin_LIBRARIES} 
)
