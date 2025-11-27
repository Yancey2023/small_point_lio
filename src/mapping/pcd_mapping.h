/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace mapping {

    class PCDMappingGrid {
    private:
        size_t points_size = 0;
        Eigen::Vector3f points_sum = Eigen::Vector3f::Zero();

    public:
        PCDMappingGrid() = default;

        void add_point(const Eigen::Vector3f &point);

        [[nodiscard]] Eigen::Vector3f get_point() const;
    };

    class PCDMapping {
    private:
        using GridKeyType = Eigen::Vector3i;

        struct GridKeyTypeHasher {
            uint64_t operator()(const GridKeyType &v) const;
        };

        ankerl::unordered_dense::map<GridKeyType, PCDMappingGrid, GridKeyTypeHasher> grids_map;
        float inv_resolution;

    public:
        explicit PCDMapping(float resolution);

    private:
        [[nodiscard]] GridKeyType get_position_index(const Eigen::Vector3f &pt) const;

    public:
        void add_point(const Eigen::Vector3f &point);

        [[nodiscard]] std::vector<Eigen::Vector3f> get_points() const;
    };

}// namespace mapping
