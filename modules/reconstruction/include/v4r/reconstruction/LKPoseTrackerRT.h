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

#ifndef KP_LK_POSE_TRACKER_RT_HH
#define KP_LK_POSE_TRACKER_RT_HH

#include <stdio.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include "v4r/common/impl/SmartPtr.hpp"
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/impl/Object.hpp>


namespace v4r
{


/**
 * LKPoseTrackerRT
 */
class V4R_EXPORTS LKPoseTrackerRT
{
public:
  class V4R_EXPORTS Parameter
  {
  public:
    bool compute_global_pose;
    cv::Size win_size;
    int max_level;
    cv::TermCriteria termcrit;
    float max_error;                     //100
    RigidTransformationRANSAC::Parameter rt_param; // 0.01 (slam: 0.03)
    Parameter(bool _compute_global_pose=true, const cv::Size &_win_size=cv::Size(21,21), int _max_level=2,
      const cv::TermCriteria &_termcrit=cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), 
      float _max_error=100, 
      const RigidTransformationRANSAC::Parameter &_rt_param=RigidTransformationRANSAC::Parameter(0.01))
    : compute_global_pose(_compute_global_pose), win_size(_win_size), max_level(_max_level),
      termcrit(_termcrit),
      max_error(_max_error),
      rt_param(_rt_param) {}
  };

private:
  Parameter param;

  cv::Mat_<unsigned char> im_gray, im_last;
  std::vector< cv::Point2f > im_points0, im_points1;
  std::vector< int > inliers;
  Eigen::Matrix4f last_pose;

  bool have_im_last;

  std::vector<unsigned char> status;
  std::vector<float> error;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;


  ObjectView::Ptr model;

  RigidTransformationRANSAC::Ptr rt;



public:
  cv::Mat dbg;

  LKPoseTrackerRT(const Parameter &p=Parameter());
  ~LKPoseTrackerRT();

  double detectIncremental(const cv::Mat &im, const DataMatrix2D<Eigen::Vector3f> &cloud, Eigen::Matrix4f &pose);
  void setLastFrame(const cv::Mat &image, const Eigen::Matrix4f &pose);


  void setModel(const ObjectView::Ptr &_model);
  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  void getProjections(std::vector< std::pair<int,cv::Point2f> > &im_pts);

  typedef SmartPtr< ::v4r::LKPoseTrackerRT> Ptr;
  typedef SmartPtr< ::v4r::LKPoseTrackerRT const> ConstPtr;
};


/***************************** inline methods *******************************/



} //--END--

#endif

