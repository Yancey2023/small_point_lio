//
// Created by sw on 2025/10/15.
//

#ifndef SMALL_POINT_LIO_BASE_LIDAR_HPP_H
#define SMALL_POINT_LIO_BASE_LIDAR_HPP_H
#include <functional>
#include <string>
#include <rclcpp/node.hpp>
#include <common.h>
class LidarDriverBase {
public:
    virtual ~LidarDriverBase() = default;
    virtual void setup_subscription(rclcpp::Node* node, const std::string& topic,
                                    std::function<void(const std::vector<common::Point>&)> callback) = 0;
};

#endif //SMALL_POINT_LIO_BASE_LIDAR_HPP_H