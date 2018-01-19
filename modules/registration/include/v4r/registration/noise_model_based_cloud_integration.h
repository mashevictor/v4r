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
 * @file noise_model_based_cloud_integration.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2013
 * @brief
 *
 */

#pragma once

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/impl/octree_iterator.hpp>

#include <v4r/common/miscellaneous.h>
#include <v4r/core/macros.h>

namespace v4r {

class V4R_EXPORTS NMBasedCloudIntegrationParameter {
 public:
  size_t min_points_per_voxel_;  ///< the minimum number of points in a leaf of the octree of the big cloud.
  float octree_resolution_;      ///< resolution of the octree of the big point cloud
  float focal_length_;  ///< focal length of the cameras; used for reprojection of the points into each image plane
  bool average_;  ///< if true, takes the average color (for each color componenent) and normal within all the points in
                  /// the leaf of the octree. Otherwise, it takes the point within the octree with the best noise weight
  float threshold_explained_;  ///< Euclidean distance for a nearby point to explain a query point. Only used if
                               /// reason_about_points = true
  bool reason_about_points_;   ///< if true, projects each point into each viewpoint and checks for occlusion or if it
                               /// can be explained by the points in the other viewpoints (this should filter lonely
  /// points but is computational expensive) --> FILTER NOT IMPLEMENTED SO FAR!!
  float min_px_distance_to_depth_discontinuity_;  ///< points of the input cloud within this distance (in pixel) to its
                                                  /// closest depth discontinuity pixel will be removed
  NMBasedCloudIntegrationParameter()
  : min_points_per_voxel_(1), octree_resolution_(0.003f), focal_length_(525.f), average_(false),
    threshold_explained_(0.02f), reason_about_points_(false), min_px_distance_to_depth_discontinuity_(3.f) {}
};

/**
 * @brief reconstructs a point cloud from several input clouds. Each point of the input cloud is associated with a
 * weight
 * which states the measurement confidence ( 0... max noise level, 1... very confident). Each point is accumulated into
 * a
 * big cloud and then reprojected into the various image planes of the input clouds to check for conflicting points.
 * Conflicting points will be removed and the remaining points put into an octree
 * @author Thomas Faeulhammer, Aitor Aldoma
 * @date December 2015
 */
template <class PointT>
class V4R_EXPORTS NMBasedCloudIntegration {
 private:
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointTPtr;
  NMBasedCloudIntegrationParameter param_;

  class PointInfo {
   private:
    size_t num_pts_for_average_;  ///< number of points used for building average

   public:
    PointT pt;
    pcl::Normal normal;

    // for each point store the number of viewpoints in which the point
    //  [0] is occluded;
    //  [1] can be explained by a nearby point;
    //  [2] could be seen but does not make sense (i.e. the view-ray is not blocked, but there is no sensed point)
    size_t occluded_, explained_, violated_;
    int origin;  ///< cloud index where the point comes from
    int pt_idx;  ///< pt index in the original cloud
    float distance_to_depth_discontinuity;
    float sigma_lateral;
    float sigma_axial;
    float weight;

    bool operator<(const PointInfo other) const {
      return weight > other.weight;
    }

    PointInfo() {
      occluded_ = violated_ = 0;
      explained_ = 1;  // the point is explained at least by the original viewpoint
      origin = -1;
      num_pts_for_average_ = 1;
      weight = std::numeric_limits<float>::max();

      pt.getVector3fMap() = Eigen::Vector3f::Zero();
      pt.r = pt.g = pt.b = 0.f;
      normal.getNormalVector3fMap() = Eigen::Vector3f::Zero();
      normal.curvature = 0.f;
    }

    void moving_average(const PointInfo &new_pt) {
      num_pts_for_average_++;

      double w_old = static_cast<double>(num_pts_for_average_) / (num_pts_for_average_ + 1.f);
      double w_new = 1.f / (static_cast<double>(num_pts_for_average_) + 1.f);

      pt.getVector3fMap() = w_old * pt.getVector3fMap() + w_new * new_pt.pt.getVector3fMap();
      pt.r = w_old * pt.r + w_new * new_pt.pt.r;
      pt.g = w_old * pt.g + w_new * new_pt.pt.g;
      pt.b = w_old * pt.b + w_new * new_pt.pt.b;

      Eigen::Vector3f new_normal = new_pt.normal.getNormalVector3fMap();
      new_normal.normalize();
      normal.getNormalVector3fMap() = w_old * normal.getNormalVector3fMap() + w_new * new_normal;
      normal.curvature = static_cast<float>(w_old * normal.curvature + w_new * new_pt.normal.curvature);
    }
  };

  std::vector<PointInfo> big_cloud_info_;
  std::vector<typename pcl::PointCloud<PointT>::ConstPtr> input_clouds_;
  std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> input_normals_;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> input_clouds_used_;
  std::vector<std::vector<int>> indices_;  ///< Indices of the object in each cloud (remaining points will be ignored)
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      transformations_to_global_;  ///< transform aligning the input point clouds when multiplied
  std::vector<std::vector<std::vector<float>>> pt_properties_;  ///< for each cloud, for each pixel represent lateral
                                                                ///[idx=0] and axial [idx=1] as well as distance to
                                                                /// closest depth discontinuity [idx=2]
  pcl::PointCloud<pcl::Normal>::Ptr output_normals_;

  void cleanUp() {
    input_clouds_.clear();
    input_normals_.clear();
    indices_.clear();
    transformations_to_global_.clear();
    pt_properties_.clear();
    big_cloud_info_.clear();
  }

  void collectInfo();
  void reasonAboutPts();

 public:
  NMBasedCloudIntegration(const NMBasedCloudIntegrationParameter &p = NMBasedCloudIntegrationParameter()) : param_(p) {}

  /**
   * @brief getOutputNormals
   * @param Normals of the registered cloud
   */
  void getOutputNormals(pcl::PointCloud<pcl::Normal>::Ptr &output) const {
    output = output_normals_;
  }

  /**
   * @brief setInputClouds
   * @param organized input clouds
   */
  void setInputClouds(const std::vector<typename pcl::PointCloud<PointT>::ConstPtr> &input) {
    input_clouds_ = input;
  }

  /**
   * @brief setInputNormals
   * @param normal clouds corresponding to the input clouds
   */
  void setInputNormals(const std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> &input) {
    input_normals_ = input;
  }

  /**
   * @brief setIndices
   * @param object mask
   */
  void setIndices(const std::vector<std::vector<size_t>> &indices) {
    indices_.resize(indices.size());
    for (size_t i = 0; i < indices.size(); i++)
      indices_[i] = convertVecSizet2VecInt(indices[i]);
  }

  void setIndices(const std::vector<std::vector<int>> &indices) {
    indices_ = indices;
  }

  /**
   * @brief compute the registered point cloud taking into account the noise model of the cameras
   * @param registered cloud
   */
  void compute(typename pcl::PointCloud<PointT>::Ptr &output);

  /**
   * @brief setPointProperties
   * @param for each cloud, for each pixel represent lateral [idx=0] and axial [idx=1] as well as distance to closest
   * depth discontinuity [idx=2]
   */
  void setPointProperties(const std::vector<std::vector<std::vector<float>>> &pt_properties) {
    pt_properties_ = pt_properties;
  }

  /**
   * @brief setTransformations
   * @param transforms aligning each point cloud to a global coordinate system
   */
  void setTransformations(const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &transforms) {
    transformations_to_global_ = transforms;
  }

  /**
   * @brief returns the points used in each view in the final filtered cloud
   * @param filtered clouds
   */
  void getInputCloudsUsed(std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds) const {
    clouds = input_clouds_used_;
  }
};
}
