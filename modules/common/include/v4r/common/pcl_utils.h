/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file pcl_utils.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2015
 * @brief Utiltity functions for PCL library
 *
 */

#pragma once

#include <pcl/common/common.h>
#include <v4r/core/macros.h>
#include <boost/dynamic_bitset.hpp>

namespace v4r {
/**
 * @brief sets the sensor origin and sensor orientation fields of the PCL pointcloud header by the given transform
 */
template <typename PointT>
V4R_EXPORTS void setCloudPose(const Eigen::Matrix4f &trans, pcl::PointCloud<PointT> &cloud);
}

namespace pcl  /// NOTE: THIS NAMESPACE IS AN EXCEPTION
{

/** \brief Extract the indices of a given point cloud as a new point cloud (instead of int types, this function uses a
 * size_t vector)
  * \param[in] cloud_in the input point cloud dataset
  * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
  * \param[out] cloud_out the resultant output point cloud dataset
  * \note Assumes unique indices.
  * \ingroup common
  */
template <typename PointT>
V4R_EXPORTS void copyPointCloud(const pcl::PointCloud<PointT> &cloud_in, const std::vector<size_t> &indices,
                                pcl::PointCloud<PointT> &cloud_out);

/** \brief Extract the indices of a given point cloud as a new point cloud (instead of int types, this function uses a
 * size_t vector)
  * \param[in] cloud_in the input point cloud dataset
  * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
  * \param[out] cloud_out the resultant output point cloud dataset
  * \note Assumes unique indices.
  * \ingroup common
  */
template <typename PointT>
V4R_EXPORTS void copyPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                const std::vector<size_t, Eigen::aligned_allocator<size_t>> &indices,
                                pcl::PointCloud<PointT> &cloud_out);

template <typename PointT>
V4R_EXPORTS void copyPointCloud(const pcl::PointCloud<PointT> &cloud_in, const std::vector<bool> &mask,
                                pcl::PointCloud<PointT> &cloud_out);

template <typename PointT>
V4R_EXPORTS void copyPointCloud(const pcl::PointCloud<PointT> &cloud_in, const boost::dynamic_bitset<> &mask,
                                pcl::PointCloud<PointT> &cloud_out);
}
