/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "base_lidar.h"
#include "common/common.h"
#include "small_point_lio/small_point_lio.h"
#ifdef USE_LIVOX_LIDAR
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_point_lio/pch.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>
namespace unilidar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        std::uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace unilidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(unilidar_ros::Point,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (std::uint16_t, ring, ring)
                                    (float, time, time)
                                    )
namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::vector<Eigen::Vector3f> pointcloud_to_save;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        common::Odometry last_odometry;

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);

    private:
        std::unique_ptr<LidarDriverBase> lidar_driver;
        void imu_callback(const sensor_msgs::msg::Imu &msg);
    };

}// namespace small_point_lio
