/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace pointcloud_cache {

    class PointcloudCache {
    public:
        explicit PointcloudCache(const YAML::Node &node);

        void set_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &callback);

        void add_pointcloud(const std::vector<Eigen::Vector3f> &pointcloud, double timestamp);

    private:
        double update_interval;
        std::vector<Eigen::Vector3f> cache;
        double last_callback_time;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> callback;
    };

}// namespace pointcloud_cache
