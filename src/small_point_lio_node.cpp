/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "common/common.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        spdlog::default_logger()->sinks().push_back(std::make_shared<ROS2Sink_mt>(this));
        std::string config_file = declare_parameter<std::string>("config_file", "");
        if (config_file.empty()) {
            SPDLOG_ERROR("error config file");
            spdlog::shutdown();
            return;
        }
        YAML::Node config = YAML::LoadFile(config_file)["small_point_lio"];
        bool save_pcd = config["save_pcd"].as<bool>();
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(config);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        map_save_trigger = this->create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        SPDLOG_ERROR("pcd save is disabled");
                        return;
                    }
                    voxelgrid_sampling::VoxelgridSampling downsampler;
                    std::vector<Eigen::Vector3f> downsampled;
                    downsampler.voxelgrid_sampling(pointcloud_to_save, downsampled, 0.02);
                    pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
                    pcl_pointcloud.reserve(downsampled.size());
                    for (const auto &point: downsampled) {
                        pcl::PointXYZI new_point;
                        new_point.x = point.x();
                        new_point.y = point.y();
                        new_point.z = point.z();
                        pcl_pointcloud.push_back(new_point);
                    }
                    pcl::PCDWriter writer;
                    writer.writeBinary(ROOT_DIR + "/pcd/scan.pcd", pcl_pointcloud);
                    SPDLOG_INFO("save pcd success");
                });
        small_point_lio->set_odometry_callback([this](const common::Odometry &odometry) {
            last_odometry = odometry;

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp.sec = std::floor(odometry.timestamp);
            odometry_msg.header.stamp.nanosec = static_cast<uint32_t>((odometry.timestamp - odometry_msg.header.stamp.sec) * 1e9);
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";
            odometry_msg.pose.pose.position.x = odometry.position.x();
            odometry_msg.pose.pose.position.y = odometry.position.y();
            odometry_msg.pose.pose.position.z = odometry.position.z();
            odometry_msg.pose.pose.orientation.x = odometry.orientation.x();
            odometry_msg.pose.pose.orientation.y = odometry.orientation.y();
            odometry_msg.pose.pose.orientation.z = odometry.orientation.z();
            odometry_msg.pose.pose.orientation.w = odometry.orientation.w();
            odometry_msg.twist.twist.linear.x = odometry.velocity.x();
            odometry_msg.twist.twist.linear.y = odometry.velocity.y();
            odometry_msg.twist.twist.linear.z = odometry.velocity.z();
            odometry_msg.twist.twist.angular.x = odometry.angular_velocity.x();
            odometry_msg.twist.twist.angular.y = odometry.angular_velocity.y();
            odometry_msg.twist.twist.angular.z = odometry.angular_velocity.z();

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = odometry_msg.header.stamp;
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = "base_link";
            geometry_msgs::msg::TransformStamped base_link_to_livox_frame_transform;
            try {
                base_link_to_livox_frame_transform =
        tf_buffer->lookupTransform("unilidar_lidar", "base_link",odometry_msg.header.stamp);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(get_logger(), "Failed to lookup transform from base_link to unilidar_lidar: %s", ex.what());
                return;
            }
            tf2::Transform tf_lidar_odom_to_livox_frame;
            tf_lidar_odom_to_livox_frame.setOrigin(tf2::Vector3(odometry.position.x(), odometry.position.y(), odometry.position.z()));
            tf_lidar_odom_to_livox_frame.setRotation(tf2::Quaternion(odometry.orientation.x(), odometry.orientation.y(), odometry.orientation.z(), odometry.orientation.w()));
            tf2::Transform tf_base_link_to_livox_frame;
            tf2::fromMsg(base_link_to_livox_frame_transform.transform, tf_base_link_to_livox_frame);
            tf2::Transform tf_odom_to_base_link = tf_base_link_to_livox_frame.inverse() * tf_lidar_odom_to_livox_frame * tf_base_link_to_livox_frame;
            transform_stamped.transform = tf2::toMsg(tf_odom_to_base_link);

            tf_broadcaster->sendTransform(transform_stamped);
            odometry_publisher->publish(odometry_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_publisher->get_subscription_count() > 0) {
                pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
                pcl_pointcloud.reserve(pointcloud.size());
                for (const auto &point: pointcloud) {
                    pcl::PointXYZI new_point;
                    new_point.x = point.x();
                    new_point.y = point.y();
                    new_point.z = point.z();
                    pcl_pointcloud.push_back(new_point);
                }
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(pcl_pointcloud, msg);
                msg.header.stamp.sec = std::floor(last_odometry.timestamp);
                msg.header.stamp.nanosec = static_cast<uint32_t>((last_odometry.timestamp - msg.header.stamp.sec) * 1e9);
                msg.header.frame_id = "odom";
                pointcloud_publisher->publish(msg);
            }
            if (save_pcd) {
                pointcloud_to_save.insert(pointcloud_to_save.end(), pointcloud.begin(), pointcloud.end());
            }
        });
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
        pointcloud_subsciber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config["lidar_topic"].as<std::string>(),
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // Step 1: ROS -> PCL
                pcl::PointCloud<unilidar_ros::Point> pl_orig;
                pcl::fromROSMsg(*msg, pl_orig);
                if (pl_orig.empty()) return;

                // ✅ 获取这一帧的 ROS 时间（全局时间戳）
                double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

                float blind = 0.1f;          // 盲区距离
                float time_unit_scale = 1.0f; // 雷达 time 字段单位是秒

                // Step 2: 过滤 + 转换为 common::Point
                std::vector<common::Point> pointcloud;
                pointcloud.reserve(pl_orig.size());

                int count_filtered = 0;
                for (const auto &src : pl_orig.points) {
                    // 距离过滤
                    float dist2 = src.x * src.x + src.y * src.y + src.z * src.z;
                    if (dist2 <= blind * blind) {
                        count_filtered++;
                        continue;
                    }

                    common::Point p;
                    p.position << src.x, src.y, src.z;

                    // ✅ 关键修正：把相对时间偏移叠加到全局时间上
                    p.timestamp = msg_time + src.time * time_unit_scale;

                    pointcloud.push_back(p);
                }

                // 打印调试信息，确认时间是否合理
                if (!pointcloud.empty()) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "msg_time=%.6f, first_point=%.6f, last_point=%.6f, input=%zu, filtered=%d, output=%zu",
                        msg_time,
                        pointcloud.front().timestamp,
                        pointcloud.back().timestamp,
                        pl_orig.size(),
                        count_filtered,
                        pointcloud.size());
                } else {
                    RCLCPP_WARN(this->get_logger(), "All points filtered out!");
                }

                // Step 3: 送入 LIO 算法
                small_point_lio->on_point_cloud_callback(pointcloud);
                small_point_lio->handle_once();
            });



        imu_subsciber = this->create_subscription<sensor_msgs::msg::Imu>(
                config["imu_topic"].as<std::string>(),
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    double an = std::sqrt(
  msg.linear_acceleration.x*msg.linear_acceleration.x +
  msg.linear_acceleration.y*msg.linear_acceleration.y +
  msg.linear_acceleration.z*msg.linear_acceleration.z
);
                    RCLCPP_INFO(this->get_logger(), "IMU acc norm: %.3f", an);
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

    SmallPointLioNode::~SmallPointLioNode() {
        spdlog::default_logger()->sinks().clear();
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
