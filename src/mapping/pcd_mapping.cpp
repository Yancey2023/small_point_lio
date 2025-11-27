/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "pcd_mapping.h"

namespace mapping {

    void PCDMappingGrid::add_point(const Eigen::Vector3f &point) {
        ++points_size;
        points_sum += point;
    }

    [[nodiscard]] Eigen::Vector3f PCDMappingGrid::get_point() const {
        return points_sum / points_size;
    }

    uint64_t PCDMapping::GridKeyTypeHasher::operator()(const GridKeyType &v) const {
        return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
    }

    PCDMapping::PCDMapping(float resolution)
        : inv_resolution(1 / resolution) {}

    [[nodiscard]] PCDMapping::GridKeyType PCDMapping::get_position_index(const Eigen::Vector3f &pt) const {
        return (pt * inv_resolution).array().floor().cast<int>();
    }

    void PCDMapping::add_point(const Eigen::Vector3f &point) {
        PCDMapping::GridKeyType key = get_position_index(point);
        auto iter = grids_map.find(key);
        if (iter == grids_map.end()) {
            iter = grids_map.emplace(key, PCDMappingGrid()).first;
        }
        iter->second.add_point(point);
    }

    [[nodiscard]] std::vector<Eigen::Vector3f> PCDMapping::get_points() const {
        std::vector<Eigen::Vector3f> result;
        result.reserve(grids_map.size());
        for (const auto &item: grids_map) {
            result.push_back(item.second.get_point());
        }
        return result;
    }

}// namespace mapping
