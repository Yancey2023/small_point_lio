/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "../common/common.h"
#include <small_point_lio/pch.h>

/**
 * copy from https://github.com/koide3/small_gicp, modified to fit our need.
 * small_gicp is open source under the MIT license: https://github.com/koide3/small_gicp/blob/master/LICENSE
 */
namespace voxelgrid_sampling {

    class VoxelgridSampling {
    private:
        std::vector<std::pair<std::uint64_t, size_t>> coord_pt;

    public:
        void voxelgrid_sampling(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &downsampled, double leaf_size);

        void voxelgrid_sampling(const std::vector<common::Point> &points, std::vector<common::Point> &downsampled, double leaf_size);

        void voxelgrid_sampling_omp(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &downsampled, double leaf_size, int num_threads = 4);
    };

}// namespace voxelgrid_sampling
