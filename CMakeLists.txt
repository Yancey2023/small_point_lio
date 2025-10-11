cmake_minimum_required(VERSION 3.10)
project(small_point_lio)

## use c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable color output
set(CMAKE_COLOR_DIAGNOSTICS ON)

## export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# enable compile warning
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
elseif(MSVC)
    add_compile_options("/W4")
    add_compile_options("/wd4100")
endif()

# fix issues in MSVC
if(MSVC)
    add_compile_options("/utf-8")
endif()

# optimize performance
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-march=native")
    add_compile_options("-ffast-math")
    add_compile_options("-fno-math-errno")
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########

configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

file(GLOB_RECURSE SOURCE include/*.h src/*.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE})
target_precompile_headers(${PROJECT_NAME} PRIVATE include/small_point_lio/pch.h)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN small_point_lio::SmallPointLioNode
    EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
    launch
)
