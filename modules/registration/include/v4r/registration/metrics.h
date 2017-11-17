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
 * @file metrics.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */

#ifndef V4R_REGISTRATION_METRICS_H__
#define V4R_REGISTRATION_METRICS_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <v4r/core/macros.h>
#include <v4r_config.h>

namespace v4r {

/**
 * @brief This method computes a cost function for the pairwise alignment of two point clouds.
 * It is computed using fast ICP
 * @param[in] cloud_src source cloud
 * @param[in] cloud_dst target cloud
 * @param[in] transform homogenous transformation matrix aligning the two point clouds
 * @param[out] registration cost ( the lower the better the alignment - weight range [0, 0.75] )
 * @param[out] refined_transform refined homogenous transformation matrix aligning the two point clouds based on ICP
 */
template <typename PointT>
V4R_EXPORTS void calcEdgeWeightAndRefineTf(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_src,
                                           const typename pcl::PointCloud<PointT>::ConstPtr &cloud_dst,
                                           const Eigen::Matrix4f &transform, float &registration_quality,
                                           Eigen::Matrix4f &refined_transform);

/**
 * @brief This method computes a cost function for the pairwise alignment of two point clouds.
 * It is computed using fast ICP
 * @param[in] cloud_src source cloud
 * @param[in] cloud_dst target cloud (already aligned to source, i.e. transformation equal identity matrix)
 * @param[out] registration cost ( the lower the better the alignment - weight range [0, 0.75] )
 * @param[out] refined_transform refined homogenous transformation matrix aligning the two point clouds based on ICP
 */
template <typename PointT>
V4R_EXPORTS void calcEdgeWeightAndRefineTf(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_src,
                                           const typename pcl::PointCloud<PointT>::ConstPtr &cloud_dst,
                                           float &registration_quality, Eigen::Matrix4f &refined_transform) {
  calcEdgeWeightAndRefineTf(cloud_src, cloud_dst, Eigen::Matrix4f::Identity(), registration_quality, refined_transform);
}
}
#endif
