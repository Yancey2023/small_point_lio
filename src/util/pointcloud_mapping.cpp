/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "pointcloud_mapping.h"

namespace util {

    void PointcloudMappingGrid::add_point(const Eigen::Vector3f &point) {
        ++points_size;
        points_sum += point;
    }

    [[nodiscard]] Eigen::Vector3f PointcloudMappingGrid::get_point() const {
        return points_sum / points_size;
    }

    uint64_t PointcloudMapping::GridKeyTypeHasher::operator()(const GridKeyType &v) const {
        return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
    }

    PointcloudMapping::PointcloudMapping(float resolution)
        : inv_resolution(1 / resolution) {}

    [[nodiscard]] PointcloudMapping::GridKeyType PointcloudMapping::get_position_index(const Eigen::Vector3f &point) const {
        return (point * inv_resolution).array().floor().cast<int>();
    }

    void PointcloudMapping::add_point(const Eigen::Vector3f &point) {
        grids_map.try_emplace(get_position_index(point)).first->second.add_point(point);
    }

    void PointcloudMapping::add_pointcloud(const std::vector<Eigen::Vector3f> &pointcloud) {
        for (const auto &point: pointcloud) {
            add_point(point);
        }
    }

    [[nodiscard]] std::vector<Eigen::Vector3f> PointcloudMapping::get_points() const {
        std::vector<Eigen::Vector3f> result;
        result.reserve(grids_map.size());
        for (const auto &item: grids_map) {
            result.push_back(item.second.get_point());
        }
        return result;
    }

}// namespace util
