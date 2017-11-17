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
 * @file zbuffering.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2012
 * @brief
 *
 */
#ifndef V4R_ZBUFFERING_H_
#define V4R_ZBUFFERING_H_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <v4r/common/camera.h>
#include <v4r/core/macros.h>
#include <boost/dynamic_bitset.hpp>

namespace v4r {

class ZBufferingParameter {
 public:
  int u_margin_, v_margin_;
  bool compute_focal_length_;
  bool do_smoothing_;  ///< tries to fill holes by dilating points over neighboring pixel
  bool do_noise_filtering_;
  float inlier_threshold_;
  int smoothing_radius_;
  bool force_unorganized_;  ///< re-projects points by given camera intrinsics even if point cloud is already organized
  bool use_normals_;        ///< if true, rejects points that do not point towards view point.
  ZBufferingParameter()
  : u_margin_(0), v_margin_(0), compute_focal_length_(false), do_smoothing_(true), do_noise_filtering_(false),
    inlier_threshold_(0.01f), smoothing_radius_(1), force_unorganized_(false), use_normals_(false) {}
};

/**
     * \brief Class to reason about occlusions
     * \author Thomas Faeulhammer, Aitor Aldoma
     */
template <typename PointT>
class V4R_EXPORTS ZBuffering {
 private:
  ZBufferingParameter param_;
  std::vector<float> depth_;
  std::vector<int> kept_indices_;
  Camera::ConstPtr cam_;       ///< camera parameters
  Eigen::MatrixXi index_map_;  ///< saves for each pixel which indices of the input cloud it represents. Non-occupied
                               /// pixels are labelled with index -1.
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;

 public:
  ZBuffering(const Camera::ConstPtr cam, const ZBufferingParameter &p = ZBufferingParameter()) : param_(p), cam_(cam) {}

  void setCamera(const Camera::ConstPtr cam) {
    cam_ = cam;
  }

  void setCloudNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
    cloud_normals_ = normals;
  }

  /**
   * @brief renderPointCloud renders a point cloud using the given camera parameters
   * @param cloud input point cloud
   * @param rendered_view[out] rendered point cloud
   * @param subsample subsampling step size n. If greater 1, will only use every n-th point for rendering
   */
  void renderPointCloud(const typename pcl::PointCloud<PointT> &cloud, typename pcl::PointCloud<PointT> &rendered_view,
                        int subsample = 1);

  std::vector<int> getKeptIndices() const {
    return kept_indices_;
  }

  /**
   * @brief getIndexMap
   * @return each pixel indicates which point of the input cloud it represents. Non-occupied pixels are labelled with
   * index -1.
   */
  Eigen::MatrixXi getIndexMap() const {
    return index_map_;
  }

  typedef boost::shared_ptr<ZBuffering<PointT>> Ptr;
  typedef boost::shared_ptr<const ZBuffering<PointT>> ConstPtr;
};
}

#endif
