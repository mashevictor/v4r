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
 * @file BoundaryRelationsBase.h
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Base class to calculate boundary relations between patches.
 */

#ifndef BOUNDARY_RELATIONS_BASE_H
#define BOUNDARY_RELATIONS_BASE_H

#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>

#include <v4r/core/macros.h>

#include "v4r/attention_segmentation//SurfaceModel.h"

namespace v4r {

class V4R_EXPORTS BoundaryRelationsBase {
 public:
 protected:
  bool computed;
  bool have_cloud;
  bool have_normals;
  bool have_boundary;

  pcl::PointIndices::Ptr indices;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  ///< Input cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;     ///< Normals (set from outside or from surfaces)
  std::vector<v4r::neighboringPair> boundary;

  int width, height;

  bool checkNaN(const pcl::PointXYZRGB p) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      return (true);
    }
    return (false);
  }

 public:
  typedef boost::shared_ptr<BoundaryRelationsBase> Ptr;

  BoundaryRelationsBase();
  ~BoundaryRelationsBase();

  /** Set input point cloud **/
  void V4R_EXPORTS setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
  // sets normals
  void V4R_EXPORTS setNormals(pcl::PointCloud<pcl::Normal>::Ptr _normals);

  // sets indices
  void V4R_EXPORTS setBoundary(std::vector<v4r::neighboringPair> &_boundary);

  /** Compare patches **/
  virtual v4r::meanVal compute();
};
}

#endif  // BOUNDARY_RELATIONS_BASE_H
