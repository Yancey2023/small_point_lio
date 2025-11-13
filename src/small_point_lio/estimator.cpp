/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "estimator.h"

namespace small_point_lio {

    constexpr int NUM_MATCH_POINTS = 5;

    Estimator::Estimator() {// NOLINT(cppcoreguidelines-pro-type-member-init)
        kf.init(
                [this](auto &&s, auto &&measurement_result) {
                    return h_point(s, measurement_result);
                },
                [this](auto &&s, auto &&measurement_result) {
                    return h_imu(s, measurement_result);
                });
    }

    void Estimator::reset() {
        ivox = std::make_shared<SmallIVox>(parameters->map_resolution, 1000000);
        kf.P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity() * 0.01;
        kf.P.block<3, 3>(state::gravity_index, state::gravity_index).diagonal().fill(0.0001);
        kf.P.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(0.001);
        kf.P.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(0.001);
    }

    [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::process_noise_cov() const {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Zero();
        cov.block<3, 3>(state::velocity_index, state::velocity_index).diagonal().fill(static_cast<state::value_type>(parameters->velocity_cov));
        cov.block<3, 3>(state::omg_index, state::omg_index).diagonal().fill(static_cast<state::value_type>(parameters->omg_cov));
        cov.block<3, 3>(state::acceleration_index, state::acceleration_index).diagonal().fill(static_cast<state::value_type>(parameters->acceleration_cov));
        cov.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(static_cast<state::value_type>(parameters->bg_cov));
        cov.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(static_cast<state::value_type>(parameters->ba_cov));
        return cov;
    }

    void Estimator::h_point(const state &s, point_measurement_result &measurement_result) {
        measurement_result.valid = false;
        // get closest point
        Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
        if (parameters->extrinsic_est_en) {
            point_imu_frame = kf.x.offset_R_L_I * point_lidar_frame.cast<state::value_type>() + kf.x.offset_T_L_I;
        } else {
            point_imu_frame = Lidar_R_wrt_IMU * point_lidar_frame.cast<state::value_type>() + Lidar_T_wrt_IMU;
        }
        point_odom_frame = (kf.x.rotation * point_imu_frame + kf.x.position).cast<float>();
        ivox->get_closest_point(point_odom_frame, nearest_points, NUM_MATCH_POINTS);
        if (nearest_points.size() != NUM_MATCH_POINTS) {
            return;
        }
        // estimate plane
#if 0
        Eigen::Matrix<float, NUM_MATCH_POINTS, 3> A;
        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
            A.row(j) = nearest_points[j];
        }
        Eigen::Matrix<float, NUM_MATCH_POINTS, 1> b;
        b.setConstant(-1);
        Eigen::Vector3f normal = A.colPivHouseholderQr().solve(b);
        float d = 1.0f / normal.norm();
#else
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto &p: nearest_points) {
            centroid.noalias() += p;
        }
        centroid /= static_cast<float>(nearest_points.size());
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto &p: nearest_points) {
            Eigen::Vector3f centered = p - centroid;
            covariance.noalias() += centered * centered.transpose();
        }
        covariance /= static_cast<float>(nearest_points.size() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);
        float d = -normal.dot(centroid);
#endif
        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
            float point_distanace = std::abs(normal.dot(nearest_points[j]) + d);
            if (point_distanace > parameters->plane_threshold) {
                return;
            }
        }
        float point_distanace = normal.dot(point_odom_frame) + d;
        if (point_lidar_frame.norm() <= parameters->match_sqaured * point_distanace * point_distanace) {
            return;
        }
        // calculate residual and jacobian matrix
        measurement_result.laser_point_cov = static_cast<state::value_type>(parameters->laser_point_cov);
        if (parameters->extrinsic_est_en) {
            Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
            Eigen::Matrix<state::value_type, 3, 1> C = s.rotation.transpose() * normal0;
            Eigen::Matrix<state::value_type, 3, 1> A, B;
            A.noalias() = point_imu_frame.cross(C);
            B.noalias() = point_lidar_frame.cast<state::value_type>().cross(s.offset_R_L_I.transpose() * C);
            measurement_result.H << normal0.transpose(), A.transpose(), B.transpose(), C.transpose();
        } else {
            Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
            Eigen::Matrix<state::value_type, 3, 1> A;
            A.noalias() = point_imu_frame.cross(s.rotation.transpose() * normal0);
            measurement_result.H << normal0.transpose(), A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        measurement_result.z = -point_distanace;
        measurement_result.valid = true;
    }

    void Estimator::h_imu(const state &s, imu_measurement_result &measurement_result) {
        std::memset(measurement_result.satu_check, false, 6);
        measurement_result.z.segment<3>(0) = angular_velocity - s.omg - s.bg;
        measurement_result.z.segment<3>(3) = linear_acceleration * imu_acceleration_scale - s.acceleration - s.ba;
        measurement_result.imu_meas_omg_cov = static_cast<state::value_type>(parameters->imu_meas_omg_cov);
        measurement_result.imu_meas_acc_cov = static_cast<state::value_type>(parameters->imu_meas_acc_cov);
        if (parameters->check_satu) {
            for (int i = 0; i < 3; i++) {
                if (std::abs(angular_velocity(i)) >= parameters->satu_gyro) {
                    measurement_result.satu_check[i] = true;
                    measurement_result.z(i) = 0.0;
                }
                if (std::abs(linear_acceleration(i)) >= parameters->satu_acc) {
                    measurement_result.satu_check[i + 3] = true;
                    measurement_result.z(i + 3) = 0.0;
                }
            }
        }
    }

}// namespace small_point_lio
