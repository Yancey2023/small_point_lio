#pragma once

#ifndef PCH_H
#define PCH_H

// 一些参数
#include <param_deliver.h>
// STL
#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
// omp
#include <omp.h>
// ankerl
#include <ankerl/unordered_dense.h>
// liblzf
#include <liblzf/lzf.h>
// ros2
#include <rclcpp/rclcpp.hpp>

#endif// PCH_H
