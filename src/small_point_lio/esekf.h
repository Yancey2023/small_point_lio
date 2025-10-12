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
            position += vec.block<3, 1>(position_index, 0);
            rotation *= exp<value_type>(vec.block<3, 1>(rotation_index, 0));
            offset_R_L_I *= exp<value_type>(vec.block<3, 1>(offset_R_L_I_index, 0));
            offset_T_L_I += vec.block<3, 1>(offset_T_L_I_index, 0);
            velocity += vec.block<3, 1>(velocity_index, 0);
            omg += vec.block<3, 1>(omg_index, 0);
            acceleration += vec.block<3, 1>(acceleration_index, 0);
            gravity += vec.block<3, 1>(gravity_index, 0);
            bg += vec.block<3, 1>(bg_index, 0);
            ba += vec.block<3, 1>(ba_index, 0);
        }
    };

    struct point_measurement_result {
        bool valid;                                 // 是否有效
        state::value_type z;                        // 观测结果
        Eigen::Matrix<state::value_type, 1, 12> h_x;// 观测结果的雅可比矩阵
        state::value_type laser_point_cov;          // 观测协方差
    };

    struct imu_measurement_result {
        Eigen::Matrix<state::value_type, 6, 1> z;// 观测结果
        Eigen::Matrix<state::value_type, 6, 1> R;// 观测协方差
        bool satu_check[6];                      // 是否有效
    };

    class esekf {
    public:
        using cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>;
        using process_model = std::function<Eigen::Matrix<state::value_type, state::DIM, 1>(const state &)>;
        using process_matrix = std::function<Eigen::Matrix<state::value_type, state::DIM, state::DIM>(const state &)>;
        using process_noise_covariance = Eigen::Matrix<state::value_type, state::DIM, state::DIM>;
        using measurement_model_point = std::function<void(const state &, const Eigen::Matrix<state::value_type, 3, 3> &, const Eigen::Matrix<state::value_type, 3, 3> &, point_measurement_result &)>;
        using measurement_model_imu = std::function<void(const state &, imu_measurement_result &)>;

        state x;
        cov P;

    private:
        process_model f_x;
        process_matrix df_dx;
        measurement_model_point h_point;
        measurement_model_imu h_imu;

    public:
        esekf() = default;

        inline void init(const process_model &f_x, const process_matrix &df_dx, const measurement_model_point &h_point, const measurement_model_imu &h_imu) {
            this->f_x = f_x;
            this->df_dx = df_dx;
            this->h_point = h_point;
            this->h_imu = h_imu;
        }

        inline void predict_state(state::value_type &dt) {
            auto f_ = f_x(x);
            x.plus(f_ * dt);
        }

        inline void predict_prop_cov(state::value_type &dt, process_noise_covariance &Q) {
            auto f_ = f_x(x);
            auto df_dx_ = df_dx(x);

            cov f_x1 = cov::Identity();
            Eigen::Matrix<state::value_type, 3, 3> res_temp_SO3;
            Eigen::Matrix<state::value_type, 3, 1> seg_SO3;

            seg_SO3 = -1 * f_.template block<3, 1>(state::rotation_index, 0) * dt;
            f_x1.template block<3, 3>(state::rotation_index, state::rotation_index) = exp<state::value_type>(seg_SO3);
            res_temp_SO3 = A_matrix<state::value_type>(seg_SO3);
            df_dx_.template block<3, 3>(state::rotation_index, 0) = res_temp_SO3 * (df_dx_.template block<3, 3>(state::rotation_index, 0));

            seg_SO3 = -1 * f_.template block<3, 1>(state::offset_R_L_I_index, 0) * dt;
            f_x1.template block<3, 3>(state::offset_R_L_I_index, state::offset_R_L_I_index) = exp<state::value_type>(seg_SO3);
            res_temp_SO3 = A_matrix<state::value_type>(seg_SO3);
            df_dx_.template block<3, 3>(state::offset_R_L_I_index, 0) = res_temp_SO3 * (df_dx_.template block<3, 3>(state::offset_R_L_I_index, 0));

            f_x1 += df_dx_ * dt;
            P = f_x1 * P * f_x1.transpose() + Q * (dt * dt);
        }

        inline bool update_iterated_point() {
            point_measurement_result measurement_result;
            h_point(x, P.template block<3, 3>(0, 0), P.template block<3, 3>(3, 3), measurement_result);
            if (!measurement_result.valid) {
                return false;
            }
            Eigen::Matrix<state::value_type, state::DIM, 1> PHT;
            Eigen::Matrix<state::value_type, state::DIM, 1> K;
            Eigen::Matrix<state::value_type, state::DIM, 1> dx;
            PHT.noalias() = P.template block<state::DIM, 12>(0, 0) * measurement_result.h_x.transpose();
            K.noalias() = PHT / (measurement_result.h_x * PHT.topRows(12) + measurement_result.laser_point_cov);
            dx.noalias() = K * measurement_result.z;
            x.plus(dx);
            P = P - K * measurement_result.h_x * P.template block<12, state::DIM>(0, 0);
            return true;
        }

        inline void update_iterated_imu() {
            imu_measurement_result measurement_result;
            h_imu(x, measurement_result);
            Eigen::Matrix<state::value_type, 6, 1> z = measurement_result.z;
            Eigen::Matrix<state::value_type, 30, 6> PHT = Eigen::Matrix<state::value_type, 30, 6>::Zero();
            Eigen::Matrix<state::value_type, 6, 30> HP = Eigen::Matrix<state::value_type, 6, 30>::Zero();
            Eigen::Matrix<state::value_type, 6, 6> HPHT = Eigen::Matrix<state::value_type, 6, 6>::Zero();
            for (int i = 0; i < 6; i++) {
                if (!measurement_result.satu_check[i]) {
                    PHT.col(i) = P.col(15 + i) + P.col(24 + i);
                    HP.row(i) = P.row(15 + i) + P.row(24 + i);
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!measurement_result.satu_check[i]) {
                    HPHT.col(i) = HP.col(15 + i) + HP.col(24 + i);
                }
                HPHT(i, i) += measurement_result.R(i);
            }
            Eigen::Matrix<state::value_type, 30, 6> K = PHT * HPHT.inverse();
            Eigen::Matrix<state::value_type, state::DIM, 1> dx = K * z;
            P -= K * HP;
            x.plus(dx);
        }
    };

}// namespace small_point_lio
