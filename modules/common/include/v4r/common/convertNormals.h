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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */ 

#ifndef KP_CONVERT_NORMALS_HPP
#define KP_CONVERT_NORMALS_HPP

#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/common/PointTypes.h>


namespace v4r 
{


inline void convertNormals(const v4r::DataMatrix2D<Eigen::Vector3f> &kp_normals, pcl::PointCloud<pcl::Normal> &pcl_normals)
{
  pcl_normals.points.resize(kp_normals.data.size());
  pcl_normals.width = kp_normals.cols;
  pcl_normals.height = kp_normals.rows;
  pcl_normals.is_dense = false;

  for (unsigned i=0; i<pcl_normals.points.size(); i++)
  {
    pcl_normals.points[i].getNormalVector3fMap() = kp_normals.data[i];
  }
}

inline void convertNormals(const pcl::PointCloud<pcl::Normal> &pcl_normals, v4r::DataMatrix2D<Eigen::Vector3f> &kp_normals)
{
  kp_normals.data.resize(pcl_normals.points.size());
  kp_normals.cols = pcl_normals.width;
  kp_normals.rows = pcl_normals.height;

  for (unsigned i=0; i<pcl_normals.points.size(); i++)
  {
    kp_normals.data[i] = pcl_normals.points[i].getNormalVector3fMap();
  }


}


} //--END--
#endif

