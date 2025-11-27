/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

/**
 * copy from https://github.com/isl-org/Open3D, modified to fit our need.
 * Open3D is open source under the MIT license: https://github.com/isl-org/Open3D/blob/main/LICENSE
 */
namespace io::pcd {

    /// \struct WritePointCloudOption
    /// \brief Optional parameters to WritePointCloud
    struct WritePointCloudOption {

        enum class IsAscii : bool {
            Binary = false,
            Ascii = true
        };

        enum class Compressed : bool {
            Uncompressed = false,
            Compressed = true
        };

        explicit WritePointCloudOption(
                // Attention: when you update the defaults, update the docstrings in
                // pybind/io/class_io.cpp
                IsAscii write_ascii = IsAscii::Binary,
                Compressed compressed = Compressed::Uncompressed)
            : write_ascii(write_ascii),
              compressed(compressed){};

        /// Whether to save in Ascii or Binary.  Some savers are capable of doing
        /// either, other ignore this.
        IsAscii write_ascii;
        /// Whether to save Compressed or Uncompressed.  Currently, only PCD is
        /// capable of compressing, and only if using IsAscii::Binary, all other
        /// formats ignore this.
        Compressed compressed;
    };

    bool read_pcd(const std::string &filename, std::vector<Eigen::Vector3f> &pointcloud);

    bool write_pcd(const std::string &filename, const std::vector<Eigen::Vector3f> &pointcloud, const WritePointCloudOption &params = WritePointCloudOption());

}// namespace io::pcd
