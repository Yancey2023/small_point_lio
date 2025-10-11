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
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T;
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R;
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T;
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R;
        }

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        Q = estimator.process_noise_cov();
        estimator.kf.x.gravity = parameters.gravity;
        estimator.G_m_s2 = parameters.gravity.norm();
        is_init = false;
        is_time_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need init map and fix gravity direction
        if (!is_init) {
            if (preprocess.point_deque.size() >= parameters.init_map_size && (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Vector3d::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration;
                    }
                    double scale = -parameters.gravity.norm() / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                }
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool has_update = false;
        pointcloud_odom_frame.clear();
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // make sure time init
                if (!is_time_init) {
                    preprocess.dense_point_deque.pop_front();
                    continue;
                }

                // collect odom frame pointcloud
                Eigen::Vector3d dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<double>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<double>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                time_current = point_lidar_frame.timestamp;

                // make sure time init
                if (!is_time_init) {
                    time_predict_last = time_current;
                    time_update_last = time_current;
                    preprocess.point_deque.pop_front();
                    is_time_init = true;
                    continue;
                }

                // predict
                double dt = time_current - time_predict_last;
                if (dt > 0) {
                    // if dt equal 0, don't predict
                    estimator.kf.predict_state(dt);
                    time_predict_last = time_current;
                }

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                estimator.kf.update_iterated_point();

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                estimator.ivox->add_point(estimator.point_odom_frame);

                has_update = true;
                preprocess.point_deque.pop_front();
            } else {
                // imu update
                time_current = imu_msg.timestamp;

                // make sure time init
                if (!is_time_init) {
                    time_predict_last = time_current;
                    time_update_last = time_current;
                    preprocess.imu_deque.pop_front();
                    is_time_init = true;
                    continue;
                }

                // predict
                double dt = time_current - time_predict_last;
                if (dt > 0) {
                    // if dt equal 0, don't predict
                    estimator.kf.predict_state(dt);
                    time_predict_last = time_current;
                }

                // update
                estimator.angular_velocity = imu_msg.angular_velocity;
                estimator.linear_acceleration = imu_msg.linear_acceleration;
                double dt_cov = time_current - time_update_last;
                time_update_last = time_current;
                estimator.kf.predict_prop_cov(dt_cov, Q);
                estimator.kf.update_iterated_imu();

                // we want odometry frequency same as lidar frequency, so we don't mark odometry updated here
                // has_update = true;
                preprocess.imu_deque.pop_front();
            }
        }

        if (!has_update) {
            return;
        }

        // publish odometry and pointcloud
        if (!parameters.publish_odometry_without_downsample) {
            publish_odometry(time_current);
        }
        if (pointcloud_callback && !pointcloud_odom_frame.empty()) {
            pointcloud_callback(pointcloud_odom_frame);
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
            odometry.position = estimator.kf.x.position;
            odometry.velocity = estimator.kf.x.velocity;
            odometry.orientation = estimator.kf.x.rotation;
            odometry.angular_velocity = estimator.kf.x.omg;
            odometry_callback(odometry);
        }
    }

}// namespace small_point_lio
