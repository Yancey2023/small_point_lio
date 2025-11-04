#include "common/common.h"
#include "small_point_lio/small_point_lio.h"
#include "pointcloud_cache/pointcloud_cache.hpp"
#include "visualize/visualize.h"
#include "voxelgrid_sampling/voxelgrid_sampling.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <small_point_lio/pch.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

int main() {
    YAML::Node config = YAML::LoadFile(ROOT_DIR + "/config/config.yaml");

    common::Odometry current_odometry;

    small_point_lio::SmallPointLio small_point_lio(config["small_point_lio"]);
    std::vector<Eigen::Vector3f> path_cache;

    visualize::Visualize visualize;

    std::thread visualize_thread([&visualize]() {
        visualize.loop();
    });

    std::vector<Eigen::Vector3f> pointcloud_map;
    size_t downsampled_points_size = 1;
    voxelgrid_sampling::VoxelgridSampling downsampler;
    std::vector<Eigen::Vector3f> downsampled;

    pointcloud_cache::PointcloudCache pointcloud_cache(config["pointcloud_cache"]);
    pointcloud_cache.set_callback([&](const std::vector<Eigen::Vector3f> &pointcloud) {
        pointcloud_map.insert(pointcloud_map.end(), pointcloud.begin(), pointcloud.end());
        if (pointcloud_map.size() > 2000000 * downsampled_points_size) {// 200000000
            downsampler.voxelgrid_sampling_omp(pointcloud_map, downsampled, 0.02, 12);
            pointcloud_map = downsampled;
            ++downsampled_points_size;
        }
        visualize.mutex.lock();
        visualize.pointcloud_realtime = pointcloud;
        if (!downsampled.empty()) {
            std::swap(visualize.pointcloud_map, downsampled);
            downsampled.clear();
        }
        visualize.path.insert(visualize.path.end(), path_cache.begin(), path_cache.end());
        visualize.mutex.unlock();
        path_cache.clear();
    });

    small_point_lio.set_pointcloud_callback([&pointcloud_cache, &current_odometry](const std::vector<Eigen::Vector3f> &pointcloud) {
        pointcloud_cache.add_pointcloud(pointcloud, current_odometry.timestamp);
    });
    small_point_lio.set_odometry_callback([&current_odometry, &path_cache](const common::Odometry &odometry) {
        path_cache.emplace_back(odometry.position.cast<float>());
        current_odometry = odometry;
        // SPDLOG_INFO("odometry: ({:.2f}, {:.2f}, {:.2f})", odometry.position.x(), odometry.position.y(), odometry.position.z());
    });
    auto rosbag_config = config["rosbag"];
    auto path = rosbag_config["path"].as<std::string>();
    auto lidar_topic = rosbag_config["lidar_topic"].as<std::string>();
    auto lidar_topic_type = rosbag_config["lidar_topic_type"].as<std::string>();
    auto imu_topic = rosbag_config["imu_topic"].as<std::string>();
    rosbag2_cpp::Reader reader = rosbag2_cpp::Reader();
    std::vector<common::Point> pointcloud;
    reader.open(path);
    while (reader.has_next()) {
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message = reader.read_next();
        if (bag_message->topic_name == lidar_topic) {
            if (lidar_topic_type == "livox_custom_msg") {
                livox_ros_driver2::msg::CustomMsg msg;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization;
                serialization.deserialize_message(&extracted_serialized_msg, &msg);
                pointcloud.clear();
                pointcloud.reserve(msg.points.size());
                for (size_t i = 0; i < msg.points.size(); ++i) {
                    livox_ros_driver2::msg::CustomPoint &point = msg.points[i];
                    if ((point.tag & 0b00110000) != 0b00000000 || (point.tag & 0b00001100) != 0b00000000 || (point.tag & 0b00000011) != 0b00000000) {
                        continue;
                    }
                    common::Point new_point;
                    new_point.position << point.x, point.y, point.z;
                    new_point.timestamp = static_cast<double>(msg.timebase + point.offset_time) / 1e9;
                    pointcloud.push_back(new_point);
                }
            } else {
                sensor_msgs::msg::PointCloud2 msg;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                serialization.deserialize_message(&extracted_serialized_msg, &msg);
                pointcloud.clear();
                size_t size = msg.width * msg.height;
                sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");
                sensor_msgs::PointCloud2ConstIterator<double> out_timestamp(msg, "timestamp");
                pointcloud.reserve(size);
                for (size_t i = 0; i < size; ++i) {
                    common::Point new_point;
                    new_point.position << *out_x, *out_y, *out_z;
                    new_point.timestamp = *out_timestamp;
                    pointcloud.push_back(new_point);
                    ++out_x;
                    ++out_y;
                    ++out_z;
                    ++out_timestamp;
                }
            }
            small_point_lio.on_point_cloud_callback(pointcloud);
        } else if (bag_message->topic_name == imu_topic) {
            sensor_msgs::msg::Imu msg;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
            serialization.deserialize_message(&extracted_serialized_msg, &msg);
            common::ImuMsg imu_msg;
            imu_msg.timestamp = rclcpp::Time(msg.header.stamp).seconds();
            imu_msg.linear_acceleration << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
            imu_msg.angular_velocity << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
            small_point_lio.on_imu_callback(imu_msg);
        } else {
            continue;
        }
        small_point_lio.handle_once();
    }
    visualize.mutex.lock();
    downsampler.voxelgrid_sampling_omp(visualize.pointcloud_map, downsampled, 0.02, 12);
    visualize.pointcloud_map = std::move(downsampled);
    visualize.pointcloud_realtime.clear();
    visualize.is_running = false;
    visualize.mutex.unlock();
    visualize_thread.join();
    return 0;
}