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
// boost
#include <boost/math/tools/precision.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/seq.hpp>
// spdlog
#include <spdlog/spdlog.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
// yaml-cpp
#include <yaml-cpp/yaml.h>
// omp
#include <omp.h>

#endif// PCH_H
