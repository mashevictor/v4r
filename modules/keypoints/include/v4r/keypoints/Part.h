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

#ifndef KP_PART_HH
#define KP_PART_HH

#include <float.h>
#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#include <vector>

namespace v4r {

class V4R_EXPORTS Part {
 public:
  enum Type { STATIC, ROTATION_1D, TRANSLATION_1D, MOTION_6D, MOTION_PLANAR_3D, MAX_TYPE, UNDEF = MAX_TYPE };

 public:
  typedef SmartPtr<::v4r::Part> Ptr;
  typedef SmartPtr<::v4r::Part const> ConstPtr;

  Type type;
  int idx;  // index to ObjectModel::parts

  bool is_hyp;  // is a part hypothesis

  static double pi_2;

  Eigen::Matrix4f pose;  // global pose

  std::vector<std::pair<int, int>> features;  // <view_idx,feature_idx>
  std::vector<std::vector<triple<int, cv::Point2f, Eigen::Vector3f>>>
      projs;  // size of features <cam, proj, current_3d_location>

  std::vector<int> subparts;  // sub parts

  Part();
  Part(Type _type);

  virtual ~Part();

  virtual void setBaseTransform(const Eigen::Matrix4f &_pose) {
    (void)_pose;
  }
  virtual void setBaseTransform(const Eigen::Matrix<double, 6, 1> &_pose) {
    (void)_pose;
  }
  virtual void getBaseTransform(Eigen::Matrix4f &_pose) {
    (void)_pose;
  }

  virtual Eigen::VectorXd getParameter() {
    return Eigen::VectorXd();
  }
  virtual void setParameter(const Eigen::VectorXd &_param) {
    (void)_param;
  }
  virtual void updatePose(const Eigen::Matrix4f &_pose);
  virtual void initParameter() {}

  inline Eigen::Matrix4f &getPose() {
    return pose;
  }
  inline const Eigen::Matrix4f &getPose() const {
    return pose;
  }

  virtual void getDeltaPose(Eigen::Matrix4f &delta_pose) {
    delta_pose.setIdentity();
  }
};

/*************************** INLINE METHODES **************************/

}  //--END--

#endif
