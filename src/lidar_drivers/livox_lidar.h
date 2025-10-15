//
// Created by sw on 2025/10/15.
//

#ifndef SMALL_POINT_LIO_LIVOX_LIDAR_H
#define SMALL_POINT_LIO_LIVOX_LIDAR_H
#ifdef HAVE_LIVOX_DRIVER
#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxDriver : public LidarDriverBase {
public:
    void setup_subscription(rclcpp::Node* node, const std::string& topic,
                            std::function<void(const std::vector<common::Point>&)> callback) override {
        auto sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            topic, rclcpp::SensorDataQoS(),
            [callback](const livox_ros_driver2::msg::CustomMsg &msg) {
                std::vector<common::Point> cloud;
                cloud.reserve(msg.points.size());
                common::Point p;
                for (const auto &pt : msg.points) {
                    if ((pt.tag & 0b010000) || (pt.tag & 0b00001100) || (pt.tag & 0b00000011)) continue;
                    p.position << pt.x, pt.y, pt.z;
                    p.timestamp = static_cast<double>(msg.timebase + pt.offset_time) * 1e-9;
                    cloud.push_back(p);
                }
                callback(cloud);
            });
        subscription_ = sub;
    }
private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};
#endif

#endif //SMALL_POINT_LIO_LIVOX_LIDAR_H