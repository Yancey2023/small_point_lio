/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

namespace small_point_lio {

    void Parameters::read_parameters(rclcpp::Node &node) {
        // 点云过滤
        point_filter_num = static_cast<int>(node.declare_parameter<long>("point_filter_num"));
        auto min_distance = node.declare_parameter<float>("min_distance");
        auto max_distance = node.declare_parameter<float>("max_distance");
        min_distance_squared = min_distance * min_distance;
        max_distance_squared = max_distance * max_distance;
        space_downsample = node.declare_parameter<bool>("space_downsample");
        space_downsample_leaf_size = node.declare_parameter<float>("space_downsample_leaf_size");

        // IMU处理
        std::vector<double> gravity_temp = node.declare_parameter<std::vector<double>>("gravity");
        gravity << gravity_temp[0], gravity_temp[1], gravity_temp[2];
        check_satu = node.declare_parameter<bool>("check_satu");
        fix_gravity_direction = node.declare_parameter<bool>("fix_gravity_direction");
        satu_acc = node.declare_parameter<double>("satu_acc") * 0.99;
        satu_gyro = node.declare_parameter<double>("satu_gyro") * 0.99;
        acc_norm = node.declare_parameter<double>("acc_norm");

        // 地图
        map_resolution = node.declare_parameter<float>("map_resolution");
        init_map_size = static_cast<size_t>(node.declare_parameter<long>("init_map_size"));

        // 雷达与IMU相对位姿
        extrinsic_est_en = node.declare_parameter<bool>("extrinsic_est_en");
        std::vector<double> extrinsic_T_temp = node.declare_parameter<std::vector<double>>("extrinsic_T");
        extrinsic_T << extrinsic_T_temp[0], extrinsic_T_temp[1], extrinsic_T_temp[2];
        std::vector<double> extrinsic_R_temp = node.declare_parameter<std::vector<double>>("extrinsic_R");
        extrinsic_R << extrinsic_R_temp[0], extrinsic_R_temp[1], extrinsic_R_temp[2],
                extrinsic_R_temp[3], extrinsic_R_temp[4], extrinsic_R_temp[5],
                extrinsic_R_temp[6], extrinsic_R_temp[7], extrinsic_R_temp[8];

        // 滤波器参数
        laser_point_cov = node.declare_parameter<double>("laser_point_cov");
        imu_meas_acc_cov = node.declare_parameter<double>("imu_meas_acc_cov");
        imu_meas_omg_cov = node.declare_parameter<double>("imu_meas_omg_cov");
        velocity_cov = node.declare_parameter<double>("velocity_cov");
        acceleration_cov = node.declare_parameter<double>("acceleration_cov");
        omg_cov = node.declare_parameter<double>("omg_cov");
        ba_cov = node.declare_parameter<double>("ba_cov");
        bg_cov = node.declare_parameter<double>("bg_cov");
        plane_threshold = node.declare_parameter<double>("plane_threshold");
        match_sqaured = node.declare_parameter<double>("match_sqaured");

        // 数据发布
        publish_odometry_without_downsample = node.declare_parameter<bool>("publish_odometry_without_downsample");
    }

}// namespace small_point_lio