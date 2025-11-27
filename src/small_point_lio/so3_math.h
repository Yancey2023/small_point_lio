/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

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
        if (ang_norm < std::numeric_limits<T>::epsilon()) {
            return Eigen::Matrix<T, 3, 3>::Identity();
        } else {
            Eigen::Matrix<T, 3, 3> K = hat<T>(ang / ang_norm);
            return Eigen::Matrix<T, 3, 3>::Identity() + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
        }
    }

    template<class T>
    Eigen::Matrix<T, 3, 3> A_matrix(const Eigen::Matrix<T, 3, 1> &v) {
        static_assert(!std::numeric_limits<T>::is_integer);
        Eigen::Matrix<T, 3, 3> res;
        T squaredNorm = v.squaredNorm();
        if (squaredNorm < std::numeric_limits<T>::epsilon()) {
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
