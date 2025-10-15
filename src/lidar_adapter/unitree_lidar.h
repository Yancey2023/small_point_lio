/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "base_lidar.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <small_point_lio/pch.h>

namespace unilidar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        std::uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}// namespace unilidar_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    unilidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
)
// clang-format on

namespace small_point_lio {

    class UnilidarAdapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                        pcl::PointCloud<unilidar_ros::Point> pl_orig;
                        pcl::fromROSMsg(*msg, pl_orig);
                        if (pl_orig.empty()) {
                            return;
                        }
                        double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(pl_orig.size());
                        for (const auto &src: pl_orig.points) {
                            common::Point p;
                            p.position << src.x, src.y, src.z;
                            p.timestamp = msg_time + src.time;
                            pointcloud.push_back(p);
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio
