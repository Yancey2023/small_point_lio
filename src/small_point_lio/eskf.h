/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "so3_math.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    struct state {
        using value_type = double;

        constexpr static int DIM = 30;
        constexpr static int position_index = 0;
        constexpr static int rotation_index = 3;
        constexpr static int offset_R_L_I_index = 6;
        constexpr static int offset_T_L_I_index = 9;
        constexpr static int velocity_index = 12;
        constexpr static int omg_index = 15;
        constexpr static int acceleration_index = 18;
        constexpr static int gravity_index = 21;
        constexpr static int bg_index = 24;
        constexpr static int ba_index = 27;

        Eigen::Matrix<value_type, 3, 1> position = Eigen::Matrix<value_type, 3, 1>::Zero();        // 位置
        Eigen::Matrix<value_type, 3, 3> rotation = Eigen::Matrix<value_type, 3, 3>::Identity();    // 旋转
        Eigen::Matrix<value_type, 3, 3> offset_R_L_I = Eigen::Matrix<value_type, 3, 3>::Identity();// 雷达到 IMU 的 R
        Eigen::Matrix<value_type, 3, 1> offset_T_L_I = Eigen::Matrix<value_type, 3, 1>::Zero();    // 雷达到 IMU 的 T
        Eigen::Matrix<value_type, 3, 1> velocity = Eigen::Matrix<value_type, 3, 1>::Zero();        // 速度
        Eigen::Matrix<value_type, 3, 1> omg = Eigen::Matrix<value_type, 3, 1>::Zero();             // 角速度
        Eigen::Matrix<value_type, 3, 1> acceleration = Eigen::Matrix<value_type, 3, 1>::Zero();    // 加速度
        Eigen::Matrix<value_type, 3, 1> gravity = Eigen::Matrix<value_type, 3, 1>::Zero();         // 重力
        Eigen::Matrix<value_type, 3, 1> bg = Eigen::Matrix<value_type, 3, 1>::Zero();              // 陀螺仪零偏
        Eigen::Matrix<value_type, 3, 1> ba = Eigen::Matrix<value_type, 3, 1>::Zero();              // 加速度零偏

        state() = default;

        inline void plus(const Eigen::Matrix<value_type, DIM, 1> &vec) {
            position += vec.segment<3>(position_index);
            rotation *= exp<value_type>(vec.segment<3>(rotation_index));
            offset_R_L_I *= exp<value_type>(vec.segment<3>(offset_R_L_I_index));
            offset_T_L_I += vec.segment<3>(offset_T_L_I_index);
            velocity += vec.segment<3>(velocity_index);
            omg += vec.segment<3>(omg_index);
            acceleration += vec.segment<3>(acceleration_index);
            gravity += vec.segment<3>(gravity_index);
            bg += vec.segment<3>(bg_index);
            ba += vec.segment<3>(ba_index);
        }
    };

    struct point_measurement_result {// NOLINT(cppcoreguidelines-pro-type-member-init)
        bool valid;
        state::value_type z;
        Eigen::Matrix<state::value_type, 1, 12> H;
        state::value_type laser_point_cov;
    };

    struct imu_measurement_result {// NOLINT(cppcoreguidelines-pro-type-member-init)
        bool satu_check[6];
        Eigen::Matrix<state::value_type, 6, 1> z;
        state::value_type imu_meas_omg_cov;
        state::value_type imu_meas_acc_cov;
    };

    class eskf {
    public:
        using measurement_model_point = std::function<void(const state &, point_measurement_result &)>;
        using measurement_model_imu = std::function<void(const state &, imu_measurement_result &)>;

        state x;
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> P;

    private:
        double time_predict_state_last = 0.0;
        double time_predict_cov_last = 0.0;
        measurement_model_point h_point;
        measurement_model_imu h_imu;

    public:
        eskf() = default;

        inline void init(const measurement_model_point &h_point, const measurement_model_imu &h_imu) {
            this->h_point = h_point;
            this->h_imu = h_imu;
        }

        inline void init_timestamp(double timestamp) {
            time_predict_state_last = timestamp;
            time_predict_cov_last = timestamp;
        }

        inline void predict_state(double timestamp) {
            auto dt_state = static_cast<state::value_type>(timestamp - time_predict_state_last);
            if (dt_state > 0) [[likely]] {
                time_predict_state_last = timestamp;
                x.position += x.velocity * dt_state;
                x.rotation *= exp<state::value_type>(x.omg * dt_state);
                x.velocity += (x.rotation * x.acceleration + x.gravity) * dt_state;
            }
        }

        inline void predict_cov(double timestamp, Eigen::Matrix<state::value_type, state::DIM, state::DIM> &Q) {
            auto dt_cov = static_cast<state::value_type>(timestamp - time_predict_cov_last);
            if (dt_cov > 0) [[likely]] {
                time_predict_cov_last = timestamp;
                Eigen::Matrix<state::value_type, 3, 1> seg_SO3 = -x.omg * dt_cov;
                Eigen::Matrix<state::value_type, state::DIM, state::DIM> F = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity();
                F.block<3, 3>(state::position_index, state::velocity_index).diagonal().fill(dt_cov);
                F.block<3, 3>(state::rotation_index, state::rotation_index) = exp<state::value_type>(seg_SO3);
                F.block<3, 3>(state::rotation_index, state::omg_index) = A_matrix<state::value_type>(seg_SO3) * dt_cov;
                F.block<3, 3>(state::velocity_index, state::rotation_index) = -x.rotation * hat<state::value_type>(x.acceleration);
                F.block<3, 3>(state::velocity_index, state::acceleration_index) = x.rotation * dt_cov;
                F.block<3, 3>(state::velocity_index, state::gravity_index).diagonal().fill(dt_cov);
                P = F * P * F.transpose() + Q * (dt_cov * dt_cov);
            }
        }

        inline bool update_point() {
            point_measurement_result measurement_result;
            h_point(x, measurement_result);
            if (!measurement_result.valid) {
                return false;
            }
            Eigen::Matrix<state::value_type, state::DIM, 1> PHT = P.template block<state::DIM, 12>(0, 0) * measurement_result.H.transpose();
            state::value_type temp = measurement_result.H * PHT.topRows(12) + measurement_result.laser_point_cov;
            if (temp == 0) [[unlikely]] {
                temp = 1e-6;
            }
            Eigen::Matrix<state::value_type, state::DIM, 1> K = PHT / temp;
            x.plus(K * measurement_result.z);
            P = P - K * measurement_result.H * P.template block<12, state::DIM>(0, 0);
            return true;
        }

        inline bool update_imu() {
            imu_measurement_result measurement_result;
            h_imu(x, measurement_result);
            Eigen::Matrix<state::value_type, 6, 1> z = measurement_result.z;
            Eigen::Matrix<state::value_type, state::DIM, 6> PHT = Eigen::Matrix<state::value_type, state::DIM, 6>::Zero();
            Eigen::Matrix<state::value_type, 6, state::DIM> HP = Eigen::Matrix<state::value_type, 6, state::DIM>::Zero();
            Eigen::Matrix<state::value_type, 6, 6> HPHT = Eigen::Matrix<state::value_type, 6, 6>::Zero();
            for (int i = 0; i < 3; i++) {
                if (!measurement_result.satu_check[i]) {
                    PHT.col(i) = P.col(state::omg_index + i) + P.col(state::bg_index + i);
                    HP.row(i) = P.row(state::omg_index + i) + P.row(state::bg_index + i);
                }
                if (!measurement_result.satu_check[i]) {
                    PHT.col(i + 3) = P.col(state::acceleration_index + i) + P.col(state::ba_index + i);
                    HP.row(i + 3) = P.row(state::acceleration_index + i) + P.row(state::ba_index + i);
                }
            }
            for (int i = 0; i < 3; i++) {
                if (!measurement_result.satu_check[i]) {
                    HPHT.col(i) = HP.col(state::omg_index + i) + HP.col(state::bg_index + i);
                }
                if (!measurement_result.satu_check[i]) {
                    HPHT.col(i + 3) = HP.col(state::acceleration_index + i) + HP.col(state::ba_index + i);
                }
                HPHT(i, i) += measurement_result.imu_meas_omg_cov;
                HPHT(i + 3, i + 3) += measurement_result.imu_meas_acc_cov;
            }
            Eigen::LDLT<Eigen::Matrix<state::value_type, 6, 6>> ldlt(HPHT);
            if (ldlt.info() != Eigen::Success) [[unlikely]] {
                return false;
            }
            Eigen::Matrix<state::value_type, state::DIM, 6> K = PHT * ldlt.solve(Eigen::Matrix<state::value_type, 6, 6>::Identity());
            x.plus(K * z);
            P -= K * HP;
            return true;
        }
    };

}// namespace small_point_lio
