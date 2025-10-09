/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "small_point_lio/small_point_lio.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_point_lio/pch.h>
#include <spdlog/sinks/base_sink.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

namespace small_point_lio {

    template<typename Mutex>
    class ROS2Sink : public spdlog::sinks::base_sink<Mutex> {
    private:
        rclcpp::Node *node;

    public:
        explicit ROS2Sink(rclcpp::Node *node)
            : node(node) {}

    protected:
        void sink_it_(const spdlog::details::log_msg &msg) override {
            spdlog::memory_buf_t formatted;
            spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
            std::string log_str = fmt::to_string(formatted);
            if (!log_str.empty() && log_str.back() == '\n') {
                log_str.pop_back();
            }
            rclcpp::Logger logger = node->get_logger();
            switch (msg.level) {
                case spdlog::level::trace:
                case spdlog::level::debug:
                    RCLCPP_DEBUG(logger, "%s", log_str.c_str());
                    break;
                case spdlog::level::info:
                    RCLCPP_INFO(logger, "%s", log_str.c_str());
                    break;
                case spdlog::level::warn:
                    RCLCPP_WARN(logger, "%s", log_str.c_str());
                    break;
                case spdlog::level::err:
                case spdlog::level::critical:
                    RCLCPP_ERROR(logger, "%s", log_str.c_str());
                    break;
                default:
                    RCLCPP_INFO(logger, "%s", log_str.c_str());
                    break;
            }
        }

        void flush_() override {
        }
    };

    using ROS2Sink_mt = ROS2Sink<std::mutex>;

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::vector<Eigen::Vector3f> pointcloud_to_save;
        std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> pointcloud_subsciber;
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

        ~SmallPointLioNode() override;

    private:
        void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg &msg);

        void imu_callback(const sensor_msgs::msg::Imu &msg);
    };

}// namespace small_point_lio
