/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "estimator.h"

namespace small_point_lio {

    constexpr int NUM_MATCH_POINTS = 5;

    Estimator::Estimator() {
        kf.init(
                [this](auto &&s) {
                    return f_x(s);
                },
                [this](auto &&s) {
                    return df_dx(s);
                },
                [this](auto &&s, auto &&cov_p, auto &&cov_R, auto &&measurement_result) {
                    return h_point(s, cov_p, cov_R, measurement_result);
                },
                [this](auto &&s, auto &&measurement_result) {
                    return h_imu(s, measurement_result);
                });
    }

    void Estimator::reset() {
        ivox = std::make_shared<SmallIVox>(parameters->map_resolution, 1000000);
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> P_init;
        kf.P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity() * 0.01;
        kf.P.block<3, 3>(state::gravity_index, state::gravity_index).diagonal().fill(0.0001);
        kf.P.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(0.001);
        kf.P.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(0.001);
    }

    Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::process_noise_cov() {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov;
        cov.setZero();
        cov.block<3, 3>(state::velocity_index, state::velocity_index).diagonal() << parameters->velocity_cov, parameters->velocity_cov, parameters->velocity_cov;
        cov.block<3, 3>(state::omg_index, state::omg_index).diagonal() << parameters->omg_cov, parameters->omg_cov, parameters->omg_cov;
        cov.block<3, 3>(state::acceleration_index, state::acceleration_index).diagonal() << parameters->acceleration_cov, parameters->acceleration_cov, parameters->acceleration_cov;
        cov.block<3, 3>(state::bg_index, state::bg_index).diagonal() << parameters->bg_cov, parameters->bg_cov, parameters->bg_cov;
        cov.block<3, 3>(state::ba_index, state::ba_index).diagonal() << parameters->ba_cov, parameters->ba_cov, parameters->ba_cov;
        return cov;
    }

    Eigen::Matrix<state::value_type, state::DIM, 1> Estimator::f_x(const state &s) {
        Eigen::Matrix<state::value_type, state::DIM, 1> res = Eigen::Matrix<state::value_type, state::DIM, 1>::Zero();
        res.block<3, 1>(state::position_index, 0) = s.velocity;
        res.block<3, 1>(state::rotation_index, 0) = s.omg;
        res.block<3, 1>(state::velocity_index, 0) = s.rotation * s.acceleration + s.gravity;
        return res;
    }

    Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::df_dx(const state &s) {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Zero();
        cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
        Eigen::Matrix<state::value_type, 3, 3> acc_hat = hat<state::value_type>(s.acceleration);
        cov.block<3, 3>(12, 3) = -s.rotation * acc_hat;
        cov.block<3, 3>(12, 18) = s.rotation;
        cov.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
        cov.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
        return cov;
    }

    void Estimator::h_point(const state &s, const Eigen::Matrix3d &cov_p, const Eigen::Matrix3d &cov_R, point_measurement_result &measurement_result) {
        measurement_result.valid = false;
        Eigen::Vector4f pabcd;
        pabcd.setZero();
        Eigen::Vector4f normvec;
        // get closest point
        Eigen::Vector3d point_imu_frame;
        if (parameters->extrinsic_est_en) {
            point_imu_frame = kf.x.offset_R_L_I * point_lidar_frame.cast<double>() + kf.x.offset_T_L_I;
        } else {
            point_imu_frame = Lidar_R_wrt_IMU * point_lidar_frame.cast<double>() + Lidar_T_wrt_IMU;
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
        Eigen::Vector3f norm_vec = A.colPivHouseholderQr().solve(b);
        float n = norm_vec.norm();
        pabcd << norm_vec / n, 1.0f / n;
#else
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto &p: nearest_points) {
            centroid += p;
        }
        centroid /= nearest_points.size();
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto &p: nearest_points) {
            Eigen::Vector3f centered = p - centroid;
            covariance.noalias() += centered * centered.transpose();
        }
        covariance /= (nearest_points.size() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);
        pabcd << normal, -normal.dot(centroid);
#endif
        Eigen::Vector4f point_homo;
        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
            point_homo << nearest_points[j], 1.0f;
            float pd2 = fabs(pabcd.dot(point_homo));
            if (pd2 > parameters->plane_thr) {
                return;
            }
        }
        point_homo << point_odom_frame, 1.0f;
        float pd2 = pabcd.dot(point_homo);
        if (point_lidar_frame.norm() <= parameters->match_s * pd2 * pd2) {
            return;
        }
        measurement_result.laser_point_cov = parameters->laser_point_cov;
        measurement_result.h_x = Eigen::Matrix<double, 1, 12>::Zero();
        if (parameters->extrinsic_est_en) {
            Eigen::Vector3d pabc;
            pabc.noalias() = pabcd.head<3>().cast<double>();
            Eigen::Matrix3d point_lidar_frame_crossmat  = hat<state::value_type>(point_lidar_frame.cast<state::value_type>());
            Eigen::Matrix3d p_imu_frame_crossmat = hat<state::value_type>(point_imu_frame.cast<state::value_type>());
            Eigen::Vector3d C, A, B;
            C.noalias() = s.rotation.transpose() * pabc;
            A.noalias() = p_imu_frame_crossmat * C;
            B.noalias() = point_lidar_frame_crossmat * s.offset_R_L_I.transpose() * C;
            measurement_result.h_x << pabc.transpose(), A.transpose(), B.transpose(), C.transpose();
        } else {
            Eigen::Vector3d pabc;
            pabc.noalias() = pabcd.head<3>().cast<double>();
            Eigen::Matrix3d point_crossmat = hat<state::value_type>(point_imu_frame.cast<state::value_type>());
            Eigen::Vector3d C, A;
            C.noalias() = s.rotation.transpose() * pabc;
            A.noalias() = point_crossmat * C;
            measurement_result.h_x << pabc.transpose(), A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        measurement_result.z = -pd2;
        measurement_result.valid = true;
    }

    void Estimator::h_imu(const state &s, imu_measurement_result &measurement_result) {
        std::memset(measurement_result.satu_check, false, 6);
        measurement_result.z.block<3, 1>(0, 0) = angular_velocity - s.omg - s.bg;
        measurement_result.z.block<3, 1>(3, 0) = linear_acceleration * G_m_s2 / parameters->acc_norm - s.acceleration - s.ba;
        measurement_result.R << parameters->imu_meas_omg_cov, parameters->imu_meas_omg_cov, parameters->imu_meas_omg_cov, parameters->imu_meas_acc_cov, parameters->imu_meas_acc_cov, parameters->imu_meas_acc_cov;
        if (parameters->check_satu) {
            if (fabs(angular_velocity(0)) >= parameters->satu_gyro) {
                measurement_result.satu_check[0] = true;
                measurement_result.z(0) = 0.0;
            }

            if (fabs(angular_velocity(1)) >= parameters->satu_gyro) {
                measurement_result.satu_check[1] = true;
                measurement_result.z(1) = 0.0;
            }

            if (fabs(angular_velocity(2)) >= parameters->satu_gyro) {
                measurement_result.satu_check[2] = true;
                measurement_result.z(2) = 0.0;
            }

            if (fabs(linear_acceleration(0)) >= parameters->satu_acc) {
                measurement_result.satu_check[3] = true;
                measurement_result.z(3) = 0.0;
            }

            if (fabs(linear_acceleration(1)) >= parameters->satu_acc) {
                measurement_result.satu_check[4] = true;
                measurement_result.z(4) = 0.0;
            }

            if (fabs(linear_acceleration(2)) >= parameters->satu_acc) {
                measurement_result.satu_check[5] = true;
                measurement_result.z(5) = 0.0;
            }
        }
    }

}// namespace small_point_lio