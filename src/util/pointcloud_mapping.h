/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace util {

    class PointcloudMappingGrid {
    private:
        size_t points_size = 0;
        Eigen::Vector3f points_sum = Eigen::Vector3f::Zero();

    public:
        PointcloudMappingGrid() = default;

        void add_point(const Eigen::Vector3f &point);

        [[nodiscard]] Eigen::Vector3f get_point() const;
    };

    class PointcloudMapping {
    private:
        using GridKeyType = Eigen::Vector3i;

        struct GridKeyTypeHasher {
            uint64_t operator()(const GridKeyType &v) const;
        };

        ankerl::unordered_dense::map<GridKeyType, PointcloudMappingGrid, GridKeyTypeHasher> grids_map;
        float inv_resolution;

    public:
        explicit PointcloudMapping(float resolution);

    private:
        [[nodiscard]] GridKeyType get_position_index(const Eigen::Vector3f &point) const;

    public:
        void add_point(const Eigen::Vector3f &point);

        void add_pointcloud(const std::vector<Eigen::Vector3f> &point);

        [[nodiscard]] std::vector<Eigen::Vector3f> get_points() const;
    };

}// namespace util
