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

#ifndef KP_TSF_OPTIMIZE_BUNDLE_HH
#define KP_TSF_OPTIMIZE_BUNDLE_HH

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>
#include <v4r/core/macros.h>


namespace v4r
{



/**
 * TSFOptimizeBundle
 */
class V4R_EXPORTS TSFOptimizeBundle 
{
public:

  /**
   * Parameter
   */
  class Parameter
  {
  public:
    double depth_error_scale;

    bool use_robust_loss;
    double loss_scale;
    bool optimize_focal_length;
    bool optimize_principal_point;
    bool optimize_radial_k1;
    bool optimize_radial_k2;
    bool optimize_radial_k3;
    bool optimize_tangential_p1;
    bool optimize_tangential_p2;
    bool optimize_delta_cloud_rgb_pose_global;
    bool optimize_delta_cloud_rgb_pose;
    double px_error_scale;
    Parameter()
      : depth_error_scale(100), use_robust_loss(true), loss_scale(2.),
        optimize_focal_length(false), optimize_principal_point(false),
        optimize_radial_k1(false), optimize_radial_k2(false), optimize_radial_k3(false),
        optimize_tangential_p1(false), optimize_tangential_p2(false),
        optimize_delta_cloud_rgb_pose_global(false), optimize_delta_cloud_rgb_pose(false),
        px_error_scale(1) {}
  };

private:
  Parameter param;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;
  std::vector<double> lm_intrinsics;

  Eigen::Matrix<double, 6, 1> delta_pose;
  std::vector<Eigen::Matrix<double, 6, 1> > poses_Rt;
  std::vector<Eigen::Matrix<double, 6, 1> > poses_Rt_RGB;
  std::vector< std::vector<Eigen::Vector3d> > points3d;

  std::vector<int> const_intrinsics;
  bool const_all_intrinsics;

  void convertPosesToRt(const std::vector<TSFFrame::Ptr> &map);
  void convertPosesFromRt(std::vector<TSFFrame::Ptr> &map);
  void convertPosesFromRtRGB(std::vector<TSFFrame::Ptr> &map);
  void convertPoints(const std::vector<TSFFrame::Ptr> &map);
  void optimizePoses(std::vector<TSFFrame::Ptr> &map);
  void optimizeCloudPosesRGBPoses(std::vector<TSFFrame::Ptr> &map);
  void optimizeCloudPosesDeltaRGBPose(std::vector<TSFFrame::Ptr> &map);

  void setCameraParameterConst();


public:
  TSFOptimizeBundle( const Parameter &p=Parameter());
  ~TSFOptimizeBundle();

  void optimize(std::vector<TSFFrame::Ptr> &map);

  void getCameraParameter(cv::Mat &_intrinsic, cv::Mat &_dist_coeffs);

  inline const std::vector< std::vector<Eigen::Vector3d> > &getOptiPoints() const {return points3d;}

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setParameter(const Parameter &p=Parameter());
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

