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
 * @file local_rec_object_hypotheses.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2013
 * @brief
 *
 */

#ifndef LOCAL_REC_OBJECT_HYPOTHESES_H_
#define LOCAL_REC_OBJECT_HYPOTHESES_H_

#include <pcl/correspondence.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/core/macros.h>
#include <v4r/recognition/model.h>

namespace v4r {

/**
 * @brief This class represents object hypotheses coming from local feature correspondences
 * @author Thomas Faeulhammer, Aitor Aldoma
 */
template <typename PointT>
class V4R_EXPORTS LocalObjectHypothesis {
 private:
  mutable boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;

 public:
  std::string model_id_;  ///< object model identifier
  pcl::CorrespondencesPtr
      model_scene_corresp_;  ///< indices between model keypoints (index query) and scene cloud (index match)

  LocalObjectHypothesis() {}

  LocalObjectHypothesis(const LocalObjectHypothesis& other) : model_id_(other.model_id_) {
    model_scene_corresp_.reset(new pcl::Correspondences);
    *model_scene_corresp_ = *other.model_scene_corresp_;
  }

  LocalObjectHypothesis& operator=(LocalObjectHypothesis other) {
    model_id_ = other.model_id_;
    model_scene_corresp_.reset(new pcl::Correspondences);
    *model_scene_corresp_ = *other.model_scene_corresp_;
    return *this;
  }

  void visualize(const pcl::PointCloud<pcl::PointXYZRGB>& scene,
                 const pcl::PointCloud<pcl::PointXYZRGB>& scene_kp) const;

  static bool gcGraphCorrespSorter(pcl::Correspondence i, pcl::Correspondence j) {
    return i.distance < j.distance;
  }

  typedef boost::shared_ptr<LocalObjectHypothesis<PointT>> Ptr;
  typedef boost::shared_ptr<const LocalObjectHypothesis<PointT>> ConstPtr;
};
}

#endif
