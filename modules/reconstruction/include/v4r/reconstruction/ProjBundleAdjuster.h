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

#ifndef KP_PROJ_BUNDLE_ADJUSTER_HH
#define KP_PROJ_BUNDLE_ADJUSTER_HH

#include <float.h>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#ifndef KP_NO_CERES_AVAILABLE
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

#include <v4r/core/macros.h>
#include <v4r/keypoints/impl/Object.hpp>

namespace v4r {

class V4R_EXPORTS ProjBundleAdjuster {
 public:
  class Parameter {
   public:
    bool optimize_intrinsic;
    bool optimize_dist_coeffs;
    bool use_depth_prior;
    double depth_error_weight;
    double depth_inl_dist;
    double depth_cut_off;
    Parameter(bool _optimize_intrinsic = false, bool _optimize_dist_coeffs = false, bool _use_depth_prior = true,
              double _depth_error_weight = 100., double _depth_inl_dist = 0.02, double _depth_cut_off = 2.)
    : optimize_intrinsic(_optimize_intrinsic), optimize_dist_coeffs(_optimize_dist_coeffs),
      use_depth_prior(_use_depth_prior), depth_error_weight(_depth_error_weight), depth_inl_dist(_depth_inl_dist),
      depth_cut_off(_depth_cut_off) {}
  };
  class Camera {
   public:
    int idx;
    Eigen::Matrix<double, 6, 1> pose_Rt;
  };

 private:
  Parameter param;

  double sqr_depth_inl_dist;

  std::vector<Camera> cameras;

  void getCameras(const Object &data, std::vector<Camera> &cameras);
  void setCameras(const std::vector<Camera> &cameras, Object &data);
  void bundle(Object &data, std::vector<Camera> &cameras);

  inline void getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R);
  inline void getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t);
  inline void setPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4f &pose);
  inline bool isnan(const Eigen::Vector3f &pt);

 public:
  cv::Mat dbg;

  ProjBundleAdjuster(const Parameter &p = Parameter());
  ~ProjBundleAdjuster();

  void optimize(Object &data);

  typedef SmartPtr<::v4r::ProjBundleAdjuster> Ptr;
  typedef SmartPtr<::v4r::ProjBundleAdjuster const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

inline void ProjBundleAdjuster::getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R) {
  R(0, 0) = pose(0, 0);
  R(0, 1) = pose(0, 1);
  R(0, 2) = pose(0, 2);
  R(1, 0) = pose(1, 0);
  R(1, 1) = pose(1, 1);
  R(1, 2) = pose(1, 2);
  R(2, 0) = pose(2, 0);
  R(2, 1) = pose(2, 1);
  R(2, 2) = pose(2, 2);
}

inline void ProjBundleAdjuster::getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t) {
  t(0) = pose(0, 3);
  t(1) = pose(1, 3);
  t(2) = pose(2, 3);
}

inline void ProjBundleAdjuster::setPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4f &pose) {
  pose.setIdentity();
  pose(0, 0) = R(0, 0);
  pose(0, 1) = R(0, 1);
  pose(0, 2) = R(0, 2);
  pose(0, 3) = t(0);
  pose(1, 0) = R(1, 0);
  pose(1, 1) = R(1, 1);
  pose(1, 2) = R(1, 2);
  pose(1, 3) = t(1);
  pose(2, 0) = R(2, 0);
  pose(2, 1) = R(2, 1);
  pose(2, 2) = R(2, 2);
  pose(2, 3) = t(2);
}

inline bool ProjBundleAdjuster::isnan(const Eigen::Vector3f &pt) {
  if (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]))
    return true;
  return false;
}

}  //--END--

#endif
