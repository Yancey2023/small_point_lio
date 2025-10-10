/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "esekf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class Estimator {
    public:
        Parameters *parameters = nullptr;
        Eigen::Vector3f point_lidar_frame;
        Eigen::Vector3f point_odom_frame;
        std::vector<Eigen::Vector3f> nearest_points;
        std::shared_ptr<SmallIVox> ivox = nullptr;
        esekf kf;
        Eigen::Vector3d angular_velocity, linear_acceleration;
        Eigen::Vector3d Lidar_T_wrt_IMU;
        Eigen::Matrix3d Lidar_R_wrt_IMU;
        double G_m_s2 = 9.81;

        Estimator();

        void reset();

        Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov();

        Eigen::Matrix<state::value_type, state::DIM, 1> f_x(const state &s);

        Eigen::Matrix<state::value_type, state::DIM, state::DIM> df_dx(const state &s);

        void h_point(const state &s, const Eigen::Matrix<state::value_type, 3, 3> &cov_p, const Eigen::Matrix<state::value_type, 3, 3> &cov_R, point_measurement_result &measurement_resulta);

        void h_imu(const state &s, imu_measurement_result &ekfom_data);
    };

}// namespace small_point_lio
