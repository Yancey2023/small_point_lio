/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace small_point_lio {

    template<class T>
    Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> &v) {
        Eigen::Matrix<T, 3, 3> res;
        res << 0, -v[2], v[1],
                v[2], 0, -v[0],
                -v[1], v[0], 0;
        return res;
    }

    template<class T>
    static inline Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1> &ang) {
        T ang_norm = ang.norm();
        constexpr T tolerance = 0.0000001;
        if (ang_norm > tolerance) {
            Eigen::Matrix<T, 3, 3> K = hat<T>(ang / ang_norm);
            return Eigen::Matrix<T, 3, 3>::Identity() + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
        } else {
            return Eigen::Matrix<T, 3, 3>::Identity();
        }
    }

    template<class T>
    Eigen::Matrix<T, 3, 3> A_matrix(const Eigen::Matrix<T, 3, 1> &v) {
        static_assert(!std::numeric_limits<T>::is_integer);
        Eigen::Matrix<T, 3, 3> res;
        T squaredNorm = v.squaredNorm();
        constexpr T tolerance = std::is_same_v<T, float> ? 1e-5f : 1e-11;
        constexpr T sqaured_tolerance = tolerance * tolerance;
        if (squaredNorm < sqaured_tolerance) {
            res = Eigen::Matrix<T, 3, 3>::Identity();
        } else {
            T norm = std::sqrt(squaredNorm);
            Eigen::Matrix<T, 3, 3> hat_v;
            hat_v.noalias() = hat(v);
            res = Eigen::Matrix<T, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * hat_v + (1 - std::sin(norm) / norm) / squaredNorm * hat_v * hat_v;
        }
        return res;
    }

}// namespace small_point_lio
