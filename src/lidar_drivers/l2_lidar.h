//
// Created by sw on 2025/10/15.
//

#ifndef SMALL_POINT_LIO_L2_LIDAR_H
#define SMALL_POINT_LIO_L2_LIDAR_H
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "base_lidar.h"

class L2Driver : public LidarDriverBase {
public:
    void setup_subscription(rclcpp::Node* node, const std::string& topic,
                            std::function<void(const std::vector<common::Point>&)> callback) override {
        auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [callback](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

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

                callback(pointcloud);
            });
        subscription_ = sub;
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

#endif //SMALL_POINT_LIO_L2_LIDAR_H