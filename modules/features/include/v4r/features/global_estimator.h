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
 * @file global_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma
 * @date 2012
 * @brief Global feature descriptor
 *
 */

#pragma once

#include <v4r/common/faat_3d_rec_framework_defines.h>
#include <v4r/core/macros.h>
#include <v4r/features/types.h>

namespace v4r {
template <typename PointT>
class V4R_EXPORTS GlobalEstimator {
 protected:
  pcl::PointCloud<pcl::Normal>::ConstPtr normals_;
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;  ///< point cloud containing the object
  std::vector<int> indices_;                          ///< indices of the point cloud belonging to the object
  std::string descr_name_;
  size_t descr_type_;
  size_t feature_dimensions_;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transforms_;  ///< transforms (e.g. from
                                                                                        /// OUR-CVFH) aligning the input
  /// cloud with the descriptors's
  /// local referance frame

 public:
  GlobalEstimator(const std::string &descr_name = "", size_t descr_type = 0, size_t feature_dimensions = 0)
  : descr_name_(descr_name), descr_type_(descr_type), feature_dimensions_(feature_dimensions) {}

  virtual ~GlobalEstimator() {}

  /**
   * @brief global feature description
   * @return signatures describing the point cloud
   */
  virtual bool compute(Eigen::MatrixXf &signature) = 0;

  /** @brief sets the normals of the point cloud belonging to the object (optional) */
  void setNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
    normals_ = normals;
  }

  /** @brief sets the indices of the point cloud belonging to the object */
  void setIndices(const std::vector<int> &p_indices) {
    indices_ = p_indices;
  }

  /** @brief sets the input cloud containing the object to be classified */
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr &in) {
    cloud_ = in;
  }

  std::string getFeatureDescriptorName() const {
    return descr_name_;
  }

  size_t getFeatureType() const {
    return descr_type_;
  }

  size_t getFeatureDimensions() const {
    return feature_dimensions_;
  }

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> getTransforms() const {
    return transforms_;
  }

  virtual bool needNormals() const = 0;

  typedef boost::shared_ptr<GlobalEstimator<PointT>> Ptr;
  typedef boost::shared_ptr<GlobalEstimator<PointT> const> ConstPtr;
};
}
