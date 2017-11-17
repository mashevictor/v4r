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

#ifndef KP_CONVERT_POSE_HPP
#define KP_CONVERT_POSE_HPP

#include <v4r/common/rotation.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace v4r {

inline void convertPose(const Eigen::Matrix4f &pose, Eigen::Matrix<double, 6, 1> &rt) {
  Eigen::Matrix3d R = pose.topLeftCorner<3, 3>().cast<double>();
  RotationMatrixToAngleAxis(&R(0, 0), &rt[0]);
  rt.tail<3>() = pose.block<3, 1>(0, 3).cast<double>();
}

inline void convertPose(const Eigen::Matrix4f &pose, Eigen::VectorXd &rt) {
  rt.resize(6);
  Eigen::Matrix3d R = pose.topLeftCorner<3, 3>().cast<double>();
  RotationMatrixToAngleAxis(&R(0, 0), &rt[0]);
  rt.tail<3>() = pose.block<3, 1>(0, 3).cast<double>();
}

inline void convertPose(const Eigen::Matrix<double, 6, 1> &rt, Eigen::Matrix4f &pose) {
  Eigen::Matrix3d R;
  pose.setIdentity();
  AngleAxisToRotationMatrix(&rt[0], &R(0, 0));
  pose.topLeftCorner<3, 3>() = R.cast<float>();
  pose.block<3, 1>(0, 3) = rt.tail<3>().cast<float>();
}

/**
 * convertPose
 */
inline void convertPose(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose) {
  pose.setIdentity();

  pose(0, 0) = R(0, 0);
  pose(0, 1) = R(0, 1);
  pose(0, 2) = R(0, 2);
  pose(1, 0) = R(1, 0);
  pose(1, 1) = R(1, 1);
  pose(1, 2) = R(1, 2);
  pose(2, 0) = R(2, 0);
  pose(2, 1) = R(2, 1);
  pose(2, 2) = R(2, 2);

  pose(0, 3) = t(0, 0);
  pose(1, 3) = t(1, 0);
  pose(2, 3) = t(2, 0);
}

}  //--END--

#endif
