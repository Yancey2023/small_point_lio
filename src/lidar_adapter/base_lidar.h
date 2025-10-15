/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "../common/common.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class LidarAdapterBase {
    public:
        virtual ~LidarAdapterBase() = default;
        virtual void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) = 0;
    };

}// namespace small_point_lio
