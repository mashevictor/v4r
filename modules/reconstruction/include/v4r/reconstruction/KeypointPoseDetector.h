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

#ifndef KP_KEYPOINT_POSE_DETECTOR_HH
#define KP_KEYPOINT_POSE_DETECTOR_HH

#include <stdio.h>
#include <v4r/features/FeatureDetector.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/Object.hpp>

namespace v4r {

/**
 * KeypointPoseDetector
 */
class V4R_EXPORTS KeypointPoseDetector {
 public:
  class V4R_EXPORTS Parameter {
   public:
    int iterationsCount;      // 1000
    float reprojectionError;  // 3
    int minInliersCount;      // 100
    float nnr;
    Parameter(int _iterationsCount = 5000, float _reprojectionError = 3, int _minInliersCount = 50, float _nnr = .9)
    : iterationsCount(_iterationsCount), reprojectionError(_reprojectionError), minInliersCount(_minInliersCount),
      nnr(_nnr) {}
  };

 private:
  Parameter param;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;

  cv::Mat_<unsigned char> im_gray;
  cv::Mat descs;
  std::vector<cv::KeyPoint> keys;
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::Point2f> query_pts;
  std::vector<cv::Point3f> model_pts;
  std::vector<int> inliers;

  ObjectView::Ptr model;

  // cv::Ptr<cv::BFMatcher> matcher;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  v4r::FeatureDetector::Ptr detector;
  v4r::FeatureDetector::Ptr descEstimator;

  inline void cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose);

 public:
  cv::Mat dbg;

  KeypointPoseDetector(const Parameter &p, const v4r::FeatureDetector::Ptr &_detector,
                       const v4r::FeatureDetector::Ptr &_descEstimator);
  ~KeypointPoseDetector();

  double detect(const cv::Mat &image, Eigen::Matrix4f &pose);

  void setModel(const ObjectView::Ptr &_model);

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr<::v4r::KeypointPoseDetector> Ptr;
  typedef SmartPtr<::v4r::KeypointPoseDetector const> ConstPtr;
};

/***************************** inline methods *******************************/
/**
 * cvToEigen
 */
inline void KeypointPoseDetector::cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t,
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

}  //--END--

#endif
