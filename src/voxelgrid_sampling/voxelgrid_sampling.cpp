/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "voxelgrid_sampling.h"

/**
 * copy from https://github.com/koide3/small_gicp, modified to fit our need.
 * small_gicp is open source under the MIT license: https://github.com/koide3/small_gicp/blob/master/LICENSE
 */
namespace voxelgrid_sampling {

    /// @brief Fast floor (https://stackoverflow.com/questions/824118/why-is-floor-so-slow).
    /// @param pt  Float vector
    /// @return    Floored int vector
    inline Eigen::Array3i fast_floor(const Eigen::Array3f &pt) {
        const Eigen::Array3i ncoord = pt.cast<int>();
        return ncoord - (pt < ncoord.cast<float>()).cast<int>();
    };

    /// @brief Implementation of quick sort with OpenMP parallelism. Do not call this directly. Use quick_sort_omp instead.
    /// @param first  First iterator
    /// @param last   Last iterator
    /// @param comp   Comparison function
    template<typename RandomAccessIterator, typename Compare>
    void quick_sort_omp_impl(RandomAccessIterator first, RandomAccessIterator last, const Compare &comp) {
        const std::ptrdiff_t n = std::distance(first, last);
        if (n < 1024) {
            std::sort(first, last, comp);
            return;
        }

        const auto median3 = [&](const auto &a, const auto &b, const auto &c, const Compare &comp) {
            return comp(a, b) ? (comp(b, c) ? b : (comp(a, c) ? c : a)) : (comp(a, c) ? a : (comp(b, c) ? c : b));
        };

        const int offset = n / 8;
        const auto m1 = median3(*first, *(first + offset), *(first + offset * 2), comp);
        const auto m2 = median3(*(first + offset * 3), *(first + offset * 4), *(first + offset * 5), comp);
        const auto m3 = median3(*(first + offset * 6), *(first + offset * 7), *(last - 1), comp);

        auto pivot = median3(m1, m2, m3, comp);
        auto middle1 = std::partition(first, last, [&](const auto &val) { return comp(val, pivot); });
        auto middle2 = std::partition(middle1, last, [&](const auto &val) { return !comp(pivot, val); });

#pragma omp task
        quick_sort_omp_impl(first, middle1, comp);

#pragma omp task
        quick_sort_omp_impl(middle2, last, comp);
    }

    /// @brief Quick sort with OpenMP parallelism.
    /// @param first        First iterator
    /// @param last         Last iterator
    /// @param comp         Comparison function
    /// @param num_threads  Number of threads
    template<typename RandomAccessIterator, typename Compare>
    void quick_sort_omp(RandomAccessIterator first, RandomAccessIterator last, const Compare &comp, int num_threads) {
#ifndef _MSC_VER
#pragma omp parallel num_threads(num_threads)
        {
#pragma omp single nowait
            { quick_sort_omp_impl(first, last, comp); }
        }
#else
        std::sort(first, last, comp);
#endif
    }

    /// @brief Voxelgrid downsampling. This function computes exact average of points in each voxel, and each voxel can contain arbitrary number of points.
    /// @note  Discretized voxel coords must be in 21bit range [-1048576, 1048575].
    ///        For example, if the downsampling resolution is 0.01 m, point coordinates must be in [-10485.76, 10485.75] m.
    ///        Points outside the valid range will be ignored.
    /// @param points      Input points
    /// @param downsampled Downsampled points
    /// @param leaf_size   Downsampling resolution
    void VoxelgridSampling::voxelgrid_sampling(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &downsampled, double leaf_size) {
        if (points.empty()) {
            downsampled = points;
        }

        const double inv_leaf_size = 1.0 / leaf_size;

        constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
        constexpr int coord_bit_size = 21;                     // Bits to represent each voxel coordinate (pack 21x3=63bits in 64bit int)
        constexpr size_t coord_bit_mask = (1 << 21) - 1;       // Bit mask
        constexpr int coord_offset = 1 << (coord_bit_size - 1);// Coordinate offset to make values positive

        coord_pt.resize(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            const Eigen::Array3i coord = fast_floor(points[i] * inv_leaf_size) + coord_offset;
            if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
                // RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "voxel coord is out of range!!");
                coord_pt[i] = {invalid_coord, i};
                continue;
            }

            // Compute voxel coord bits (0|1bit, z|21bit, y|21bit, x|21bit)
            const std::uint64_t bits =                                                               //
                    (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |//
                    (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |//
                    (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
            coord_pt[i] = {bits, i};
        }

        // Sort by voxel coord
        const auto compare = [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; };
        std::sort(coord_pt.begin(), coord_pt.end(), compare);

        downsampled.resize(points.size());

        size_t num_points = 0;
        auto sum_pt = points[coord_pt.front().second];
        size_t sum_pt_size = 1;
        for (size_t i = 1; i < points.size(); i++) {
            if (coord_pt[i].first == invalid_coord) {
                continue;
            }

            if (coord_pt[i - 1].first != coord_pt[i].first) {
                downsampled[num_points++] = sum_pt / sum_pt_size;
                sum_pt.setZero();
                sum_pt_size = 0;
            }

            const auto &point = points[coord_pt[i].second];
            sum_pt += point;
            ++sum_pt_size;
        }

        downsampled[num_points++] = sum_pt / sum_pt_size;
        downsampled.resize(num_points);
    }

    /// @brief Voxelgrid downsampling. This function computes exact average of points in each voxel, and each voxel can contain arbitrary number of points.
    /// @note  Discretized voxel coords must be in 21bit range [-1048576, 1048575].
    ///        For example, if the downsampling resolution is 0.01 m, point coordinates must be in [-10485.76, 10485.75] m.
    ///        Points outside the valid range will be ignored.
    /// @param points      Input points
    /// @param downsampled Downsampled points
    /// @param leaf_size   Downsampling resolution
    void VoxelgridSampling::voxelgrid_sampling(const std::vector<common::Point> &points, std::vector<common::Point> &downsampled, double leaf_size) {
        if (points.empty()) {
            downsampled = points;
            return;
        }

        const double inv_leaf_size = 1.0 / leaf_size;

        constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
        constexpr int coord_bit_size = 21;                     // Bits to represent each voxel coordinate (pack 21x3=63bits in 64bit int)
        constexpr size_t coord_bit_mask = (1 << 21) - 1;       // Bit mask
        constexpr int coord_offset = 1 << (coord_bit_size - 1);// Coordinate offset to make values positive

        coord_pt.resize(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            const Eigen::Array3i coord = fast_floor(points[i].position * inv_leaf_size) + coord_offset;
            if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
                // RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "voxel coord is out of range!!");
                coord_pt[i] = {invalid_coord, i};
                continue;
            }

            // Compute voxel coord bits (0|1bit, z|21bit, y|21bit, x|21bit)
            const std::uint64_t bits =                                                               //
                    (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |//
                    (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |//
                    (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
            coord_pt[i] = {bits, i};
        }

        // Sort by voxel coord
        const auto compare = [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; };
        std::sort(coord_pt.begin(), coord_pt.end(), compare);

        downsampled.resize(points.size());

        size_t num_points = 0;
        auto sum_pt = points[coord_pt.front().second];
        size_t sum_pt_size = 1;
        for (size_t i = 1; i < points.size(); i++) {
            if (coord_pt[i].first == invalid_coord) {
                continue;
            }

            if (coord_pt[i - 1].first != coord_pt[i].first) {
                auto &point = downsampled[num_points++];
                point.position = sum_pt.position / sum_pt_size;
                point.timestamp = sum_pt.timestamp;
                sum_pt.position.setZero();
                sum_pt_size = 0;
            }

            const auto &point = points[coord_pt[i].second];
            sum_pt.position += point.position;
            sum_pt.timestamp = point.timestamp;
            ++sum_pt_size;
        }

        auto &point = downsampled[num_points++];
        point.position = sum_pt.position / sum_pt_size;
        point.timestamp = sum_pt.timestamp;
        downsampled.resize(num_points);
    }

    /// @brief Voxel grid downsampling with OpenMP backend.
    /// @note  This function has minor run-by-run non-deterministic behavior due to parallel data collection that results
    ///        in a deviation of the number of points in the downsampling results (up to 10% increase from the single-thread version).
    /// @note  Discretized voxel coords must be in 21bit range [-1048576, 1048575].
    ///        For example, if the downsampling resolution is 0.01 m, point coordinates must be in [-10485.76, 10485.75] m.
    ///        Points outside the valid range will be ignored.
    /// @param points      Input points
    /// @param downsampled Downsampled points
    /// @param leaf_size   Downsampling resolution
    void VoxelgridSampling::voxelgrid_sampling_omp(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &downsampled, double leaf_size, int num_threads) {
        if (points.size() == 0) {
            downsampled = points;
            return;
        }

        const double inv_leaf_size = 1.0 / leaf_size;

        constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
        constexpr int coord_bit_size = 21;                     // Bits to represent each voxel coordinate (pack 21x3 = 63bits in 64bit int)
        constexpr size_t coord_bit_mask = (1 << 21) - 1;       // Bit mask
        constexpr int coord_offset = 1 << (coord_bit_size - 1);// Coordinate offset to make values positive

        coord_pt.resize(points.size());
#pragma omp parallel for num_threads(num_threads) schedule(guided, 32)
        for (size_t i = 0; i < points.size(); i++) {
            const Eigen::Array3i coord = fast_floor(points[i] * inv_leaf_size) + coord_offset;
            if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
                // RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "voxel coord is out of range!!");
                coord_pt[i] = {invalid_coord, i};
                continue;
            }
            // Compute voxel coord bits (0|1bit, z|21bit, y|21bit, x|21bit)
            const std::uint64_t bits =                                                               //
                    (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |//
                    (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |//
                    (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
            coord_pt[i] = {bits, i};
        }

        // Sort by voxel coords
        quick_sort_omp(coord_pt.begin(), coord_pt.end(), [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; }, num_threads);

        downsampled.resize(points.size());

        // Take block-wise sum
        const int block_size = 1024;
        std::atomic_uint64_t num_points = 0;

#pragma omp parallel for num_threads(num_threads) schedule(guided, 4)
        for (size_t block_begin = 0; block_begin < points.size(); block_begin += block_size) {
            std::vector<Eigen::Vector3f> sub_points;
            sub_points.reserve(block_size);
            const size_t block_end = std::min<size_t>(points.size(), block_begin + block_size);
            Eigen::Vector3f sum_pt = points[coord_pt[block_begin].second];
            size_t sum_pt_size = 1;
            for (size_t i = block_begin + 1; i != block_end; i++) {
                if (coord_pt[i].first == invalid_coord) {
                    continue;
                }
                if (coord_pt[i - 1].first != coord_pt[i].first) {
                    sub_points.emplace_back(sum_pt / sum_pt_size);
                    sum_pt.setZero();
                    sum_pt_size = 0;
                }
                sum_pt += points[coord_pt[i].second];
                ++sum_pt_size;
            }
            sub_points.emplace_back(sum_pt / sum_pt_size);

            const size_t point_index_begin = num_points.fetch_add(sub_points.size());
            for (size_t i = 0; i < sub_points.size(); i++) {
                downsampled[point_index_begin + i] = sub_points[i];
            }
        }
        downsampled.resize(num_points);
    }

}// namespace voxelgrid_sampling
