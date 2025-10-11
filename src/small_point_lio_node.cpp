/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        bool save_pcd = declare_parameter<bool>("save_pcd");
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
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
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
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
                base_link_to_livox_frame_transform = tf_buffer->lookupTransform("livox_frame", "base_link", odometry_msg.header.stamp);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform from base_link to livox_frame: %s", ex.what());
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
        pointcloud_subsciber = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lidar_topic,
                rclcpp::SensorDataQoS(),
                [this](const livox_ros_driver2::msg::CustomMsg &msg) {
                    pointcloud.clear();
                    pointcloud.reserve(msg.points.size());
                    common::Point new_point;
                    for (const auto &point: msg.points) {
                        if ((point.tag & 0b010000) != 0b00000000 || (point.tag & 0b00001100) != 0b00000000 || (point.tag & 0b00000011) != 0b00000000) {
                            continue;
                        }
                        new_point.position << point.x, point.y, point.z;
                        new_point.timestamp = static_cast<double>(msg.timebase + point.offset_time) * 1e-9;
                        pointcloud.push_back(new_point);
                    }
                    small_point_lio->on_point_cloud_callback(pointcloud);
                    small_point_lio->handle_once();
                });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
