/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "../common/common.h"
#include "../voxelgrid_sampling/voxelgrid_sampling.h"
#include "parameters.h"

namespace small_point_lio {

    class Preprocess {
    public:
        Parameters *parameters = nullptr;
        std::deque<common::Point> point_deque;
        std::deque<common::Point> dense_point_deque;
        std::deque<common::ImuMsg> imu_deque;

    private:
        double last_timestamp_lidar = -1;
        double last_timestamp_dense_point = -1;
        double last_timestamp_imu = -1;
        voxelgrid_sampling::VoxelgridSampling downsampler;
        std::vector<common::Point> filtered_points;
        std::vector<common::Point> dense_points;
        std::vector<common::Point> processed_pointcloud;

    public:
        Preprocess() = default;

        void reset();
        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);
        void on_imu_callback(const common::ImuMsg &imu_msg);
    };

}// namespace small_point_lio
