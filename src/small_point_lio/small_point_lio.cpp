/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        is_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                estimator.kf.predict_state(time_current);

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                estimator.kf.update_point();

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                estimator.ivox->add_point(estimator.point_odom_frame);

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();

                preprocess.imu_deque.pop_front();
            }
        }

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry_callback(odometry);
        }
    }

}// namespace small_point_lio
