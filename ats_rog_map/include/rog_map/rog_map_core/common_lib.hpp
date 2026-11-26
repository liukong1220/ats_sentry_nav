/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SUPER_UTILS_HEADER_TYPE_UTILS_HPP
#define SUPER_UTILS_HEADER_TYPE_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <super_utils/type_utils.hpp>
#include <super_utils/fmt_eigen.hpp>


namespace rog_map {
    using super_utils::Vec3f;
    using super_utils::Vec3i;
    using super_utils::vec_Vec3f;
    using super_utils::vec_E;

    template <typename Scalar_t>
    static Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
        Eigen::Quaternion<Scalar_t> q = q_.normalized();

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        return ypr;
    }

    template <typename Scalar_t>
    static Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t>& q) {
        return quaternion_to_ypr(q)(0);
    }


    inline super_utils::Vec3f lineBoxIntersectPoint(const super_utils::Vec3f& pt, const super_utils::Vec3f& pos,
                                                    const super_utils::Vec3f& box_min,
                                                    const super_utils::Vec3f& box_max) {
        Eigen::Vector3d diff = pt - pos;
        Eigen::Vector3d max_tc = box_max - pos;
        Eigen::Vector3d min_tc = box_min - pos;

        double min_t = 1000000;

        for (int i = 0; i < 3; ++i) {
            if (fabs(diff[i]) > 0) {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;

                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }

        return pos + (min_t - 1e-3) * diff;
    }
}


#endif //SUPER_UTILS_HEADER_TYPE_UTILS_HPP