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

#ifndef KP_PROJ_LK_POSE_TRACKER_R2_HH
#define KP_PROJ_LK_POSE_TRACKER_R2_HH

#include <stdio.h>
#include <v4r/reconstruction/RefineProjectedPointLocationLK.h>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/Object.hpp>

namespace v4r {

/**
 * ProjLKPoseTrackerR2
 */
class V4R_EXPORTS ProjLKPoseTrackerR2 {
 public:
  class V4R_EXPORTS Parameter {
   public:
    double inl_dist;
    double eta_ransac;         // eta for pose ransac
    unsigned max_rand_trials;  // max. number of trials for pose ransac
    int pnp_method;            // cv::ITERATIVE, cv::P3P
    int nb_ransac_points;
    RefineProjectedPointLocationLK::Parameter plk_param;
    Parameter(double _inl_dist = 2, double _eta_ransac = 0.01, unsigned _max_rand_trials = 2000,
              int _pnp_method = INT_MIN, int _nb_ransac_points = 4,
              const RefineProjectedPointLocationLK::Parameter &_plk_param = RefineProjectedPointLocationLK::Parameter())
    : inl_dist(_inl_dist), eta_ransac(_eta_ransac), max_rand_trials(_max_rand_trials), pnp_method(_pnp_method),
      nb_ransac_points(_nb_ransac_points), plk_param(_plk_param) {}
  };

 private:
  Parameter param;

  float sqr_inl_dist;

  cv::Mat_<double> src_dist_coeffs, tgt_dist_coeffs;
  cv::Mat_<double> src_intrinsic, tgt_intrinsic;

  cv::Mat_<unsigned char> im_gray;
  std::vector<cv::Point2f> im_points;
  std::vector<int> inliers, converged;

  ObjectView::Ptr model;

  RefineProjectedPointLocationLK::Ptr plk;

  void ransacSolvePnP(const std::vector<cv::Point3f> &points, const std::vector<cv::Point2f> &im_points,
                      Eigen::Matrix4f &pose, std::vector<int> &inliers);
  void getRandIdx(int size, int num, std::vector<int> &idx);
  unsigned countInliers(const std::vector<cv::Point3f> &points, const std::vector<cv::Point2f> &_im_points,
                        const Eigen::Matrix4f &pose);
  void getInliers(const std::vector<cv::Point3f> &points, const std::vector<cv::Point2f> &im_points,
                  const Eigen::Matrix4f &pose, std::vector<int> &inliers);

  inline void cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose);
  inline bool contains(const std::vector<int> &idx, int num);

 public:
  cv::Mat dbg;

  ProjLKPoseTrackerR2(const Parameter &p = Parameter());
  ~ProjLKPoseTrackerR2();

  double detect(const cv::Mat &image, Eigen::Matrix4f &pose);

  ObjectView::Ptr getModel() {
    return model;
  }

  void setModel(const ObjectView::Ptr &_model, const Eigen::Matrix4f &_pose);
  void getProjections(std::vector<std::pair<int, cv::Point2f>> &im_pts);

  void setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr<::v4r::ProjLKPoseTrackerR2> Ptr;
  typedef SmartPtr<::v4r::ProjLKPoseTrackerR2 const> ConstPtr;
};

/***************************** inline methods *******************************/
/**
 * cvToEigen
 */
inline void ProjLKPoseTrackerR2::cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t,
                                           Eigen::Matrix4f &pose) {
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

inline bool ProjLKPoseTrackerR2::contains(const std::vector<int> &idx, int num) {
  for (unsigned i = 0; i < idx.size(); i++)
    if (idx[i] == num)
      return true;
  return false;
}

}  //--END--

#endif
