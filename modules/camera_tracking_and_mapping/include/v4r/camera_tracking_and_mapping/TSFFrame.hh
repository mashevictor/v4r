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

#ifndef KP_TSF_FRAME_HH
#define KP_TSF_FRAME_HH

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/shared_ptr.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/camera_tracking_and_mapping/Surfel.hh>
#include <v4r/core/macros.h>



namespace v4r
{



/**
 * TSFFrame
 */
class V4R_EXPORTS TSFFrame 
{
public:
  int idx;
  Eigen::Matrix4f pose;
  Eigen::Matrix4f delta_cloud_rgb_pose;
  v4r::DataMatrix2D<Surfel> sf_cloud;

  std::vector<cv::Point2f> points;
  std::vector<Eigen::Vector3f> points3d;
  std::vector<Eigen::Vector3f> normals;

  cv::Mat descs;
  std::vector<cv::KeyPoint> keys;
  std::vector<Eigen::Vector3f> keys3d;

  int fw_link, bw_link;
  std::vector<int> loop_links;

  bool have_track;

  std::vector< std::vector< v4r::triple<int, cv::Point2f, Eigen::Vector3f > > > projections;

  TSFFrame();
  TSFFrame(const int &_idx, const Eigen::Matrix4f &_pose, const v4r::DataMatrix2D<Surfel> &_sf_cloud, bool _have_track);
  ~TSFFrame();

  typedef boost::shared_ptr< ::v4r::TSFFrame> Ptr;
  typedef boost::shared_ptr< ::v4r::TSFFrame const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

