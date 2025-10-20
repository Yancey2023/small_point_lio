/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#ifdef HAVE_LIVOX_DRIVER

#include "base_lidar.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace small_point_lio {

    class LivoxLidarAdapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback](const livox_ros_driver2::msg::CustomMsg &msg) {
                        std::vector<common::Point> cloud;
                        cloud.reserve(msg.points.size());
                        common::Point p;
                        for (const auto &pt: msg.points) {
                            if ((pt.tag & 0b00110000) || (pt.tag & 0b00001100) || (pt.tag & 0b00000011)) continue;
                            p.position << pt.x, pt.y, pt.z;
                            p.timestamp = static_cast<double>(msg.timebase + pt.offset_time) * 1e-9;
                            cloud.push_back(p);
                        }
                        callback(cloud);
                    });
        }
    };

}// namespace small_point_lio

#endif
