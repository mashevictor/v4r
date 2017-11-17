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
 * @file multiview_object_recognizer_change_detection.h
 * @author Martin Velas (ivelas@fit.vutrb.cz), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date Jan, 2016
 * @brief multiview object instance recognizer with change detection filtering
 *
 */

#ifndef V4R_MULTIVIEW_OBJECT_RECOGNIZER_CHANGE_DETECTION_H__
#define V4R_MULTIVIEW_OBJECT_RECOGNIZER_CHANGE_DETECTION_H__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <v4r/core/macros.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>
#include <v4r/recognition/multiview_object_recognizer.h>
#include <v4r/recognition/multiview_representation.h>
#include <v4r_config.h>

namespace v4r {

template <typename PointT>
class V4R_EXPORTS MultiviewRecognizerWithChangeDetection : public MultiviewRecognizer<PointT> {
 public:
  typedef Model<PointT> ModelT;
  typedef boost::shared_ptr<ModelT> ModelTPtr;
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;

 protected:
  using MultiviewRecognizer<PointT>::views_;
  using MultiviewRecognizer<PointT>::id_;

  std::map<size_t, typename pcl::PointCloud<PointT>::Ptr>
      removed_points_history_;  ///< changes detected in previous observations
  std::map<size_t, typename pcl::PointCloud<PointT>::Ptr> removed_points_cumulated_history_;  ///< changes detected in
                                                                                              /// previous observations
  ///(cumulated for
  /// reconstruction
  /// filtering)

  CloudPtr changing_scene;  ///< current status of dynamic scene

 public:
  class Parameter : public MultiviewRecognizer<PointT>::Parameter {
   public:
    int min_points_for_hyp_removal_;  ///< how many removed points must overlap hypothesis to be also considered removed
    float tolerance_for_cloud_diff_;  ///< tolerance for point cloud difference [2cm]

    Parameter(int min_points_for_hyp_removal = 50, float tolerance_for_cloud_diff = 0.02)
    : min_points_for_hyp_removal_(min_points_for_hyp_removal), tolerance_for_cloud_diff_(tolerance_for_cloud_diff) {}
  } param_;

  MultiviewRecognizerWithChangeDetection(const Parameter &p = Parameter()) : MultiviewRecognizer<PointT>(p) {}

  MultiviewRecognizerWithChangeDetection(int argc, char **argv);

 protected:
  virtual void initHVFilters();

  virtual void cleanupHVFilters();

  void findChangedPoints(Cloud observation_unposed, Eigen::Affine3f pose, pcl::PointCloud<PointT> &removed_points,
                         pcl::PointCloud<PointT> &added_points);

  virtual void reconstructionFiltering(CloudPtr observation, pcl::PointCloud<pcl::Normal>::Ptr observation_normals,
                                       const Eigen::Matrix4f &absolute_pose, size_t view_id);

  void setNan(pcl::Normal &normal);

  void setNan(PointT &pt);

  virtual std::vector<bool> getHypothesisInViewsMask(ModelTPtr model, const Eigen::Matrix4f &pose, size_t origin_id);

  template <typename K, typename V>
  std::vector<K> getMapKeys(const std::map<K, V> &container);
};
}

#endif
