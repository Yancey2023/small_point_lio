/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace common {

    struct Odometry {
        double timestamp;                // Unit: s
        Eigen::Vector3d position;        // Unit: m
        Eigen::Vector3d velocity;        // Unit: m/s
        Eigen::Quaterniond orientation;  // Unit: rad
        Eigen::Vector3d angular_velocity;// Unit: rad/s
    };

    struct ImuMsg {
        double timestamp;                   // Unit: s
        Eigen::Vector3d linear_acceleration;// Unit: g
        Eigen::Vector3d angular_velocity;   // Unit: rad/s
    };

    struct Point {
        double timestamp;        // Unit: s
        Eigen::Vector3f position;// Unit: m
    };

}// namespace common
