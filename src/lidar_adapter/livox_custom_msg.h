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

    class LivoxCustomMsgAdapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback](const livox_ros_driver2::msg::CustomMsg &msg) {
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(msg.points.size());
                        common::Point new_point;
                        for (const auto &point: msg.points) {
                            if ((point.tag & 0b00111111) == 0b00000000) {
                                common::Point new_point;
                                new_point.position << point.x, point.y, point.z;
                                new_point.timestamp = static_cast<double>(msg.timebase + point.offset_time) / 1e9;
                                pointcloud.push_back(new_point);
                            }
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio

#endif
