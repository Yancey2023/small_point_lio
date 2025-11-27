/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "base_lidar.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace small_point_lio {

    class LivoxPointCloud2Adapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback](const sensor_msgs::msg::PointCloud2 &msg) {
                        sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
                        sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
                        sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");
                        sensor_msgs::PointCloud2ConstIterator<uint8_t> out_tag(msg, "tag");
                        sensor_msgs::PointCloud2ConstIterator<double> out_timestamp(msg, "timestamp");
                        size_t size = msg.width * msg.height;
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(size);
                        for (size_t i = 0; i < size; ++i) {
                            if ((*out_tag & 0b00111111) == 0b00000000) {
                                common::Point new_point;
                                new_point.position << *out_x, *out_y, *out_z;
                                new_point.timestamp = *out_timestamp * 1e-9;
                                pointcloud.push_back(new_point);
                            }
                            ++out_x;
                            ++out_y;
                            ++out_z;
                            ++out_tag;
                            ++out_timestamp;
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio
