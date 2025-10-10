/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

namespace small_point_lio {

    void Parameters::read_parameters(const YAML::Node &node) {
        // 点云过滤
        point_filter_num = node["point_filter_num"].as<int>();
        auto min_distance = node["min_distance"].as<float>();
        auto max_distance = node["max_distance"].as<float>();
        min_distance_squared = min_distance * min_distance;
        max_distance_squared = max_distance * max_distance;
        space_downsample = node["space_downsample"].as<bool>();
        space_downsample_leaf_size = node["space_downsample_leaf_size"].as<float>();

        // IMU处理
        gravity << node["gravity"][0].as<double>(),
                node["gravity"][1].as<double>(),
                node["gravity"][2].as<double>();
        check_satu = node["check_satu"].as<bool>();
        fix_gravity_direction = node["fix_gravity_direction"].as<bool>();
        satu_acc = node["satu_acc"].as<double>() * 0.99;
        satu_gyro = node["satu_gyro"].as<double>() * 0.99;
        acc_norm = node["acc_norm"].as<double>();

        // 地图
        map_resolution = node["map_resolution"].as<float>();
        init_map_size = node["init_map_size"].as<size_t>();

        // 雷达与IMU相对位姿
        extrinsic_est_en = node["extrinsic_est_en"].as<bool>();
        extrinsic_T << node["extrinsic_T"][0].as<double>(),
                node["extrinsic_T"][1].as<double>(),
                node["extrinsic_T"][2].as<double>();
        extrinsic_R << node["extrinsic_R"][0].as<double>(),
                node["extrinsic_R"][1].as<double>(),
                node["extrinsic_R"][2].as<double>(),
                node["extrinsic_R"][3].as<double>(),
                node["extrinsic_R"][4].as<double>(),
                node["extrinsic_R"][5].as<double>(),
                node["extrinsic_R"][6].as<double>(),
                node["extrinsic_R"][7].as<double>(),
                node["extrinsic_R"][8].as<double>();

        // 滤波器参数
        laser_point_cov = node["laser_point_cov"].as<double>();
        imu_meas_acc_cov = node["imu_meas_acc_cov"].as<double>();
        imu_meas_omg_cov = node["imu_meas_omg_cov"].as<double>();
        velocity_cov = node["velocity_cov"].as<double>();
        acceleration_cov = node["acceleration_cov"].as<double>();
        omg_cov = node["omg_cov"].as<double>();
        ba_cov = node["ba_cov"].as<double>();
        bg_cov = node["bg_cov"].as<double>();
        plane_thr = node["plane_thr"].as<double>();
        match_s = node["match_s"].as<double>();

        // 数据发布
        publish_odometry_without_downsample = node["publish_odometry_without_downsample"].as<bool>();
    }

}// namespace small_point_lio