/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class Estimator {
    public:
        // for common
        Parameters *parameters = nullptr;
        eskf kf;
        // for h_point
        std::shared_ptr<SmallIVox> ivox;
        Eigen::Matrix<state::value_type, 3, 1> Lidar_T_wrt_IMU;
        Eigen::Matrix<state::value_type, 3, 3> Lidar_R_wrt_IMU;
        Eigen::Vector3f point_lidar_frame;
        Eigen::Vector3f point_odom_frame;
        std::vector<Eigen::Vector3f> nearest_points;
        // for h_imu
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;

        Estimator();

        void reset();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        void h_point(const state &s, point_measurement_result &measurement_result);

        void h_imu(const state &s, imu_measurement_result &measurement_result);
    };

}// namespace small_point_lio
