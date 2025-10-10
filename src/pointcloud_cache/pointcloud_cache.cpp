/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "pointcloud_cache.hpp"

namespace pointcloud_cache {

    PointcloudCache::PointcloudCache(const YAML::Node &node) {
        update_interval = node["update_interval"].as<double>();
        last_callback_time = 0;
    }

    void PointcloudCache::set_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &callback) {
        this->callback = callback;
    }

    void PointcloudCache::add_pointcloud(const std::vector<Eigen::Vector3f> &pointcloud, double timestamp) {
        cache.insert(cache.end(), pointcloud.begin(), pointcloud.end());
        if (timestamp - last_callback_time > update_interval) {
            callback(cache);
            cache.clear();
            last_callback_time = timestamp;
        }
    }

}// namespace pointcloud_cache
