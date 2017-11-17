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

#ifndef KP_KEYPOINT_POSE_DETECTOR_RT_HH
#define KP_KEYPOINT_POSE_DETECTOR_RT_HH

#include <stdio.h>
#include <v4r/features/FeatureDetectorHeaders.h>
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/Object.hpp>

namespace v4r {

/**
 * KeypointPoseDetectorRT
 */
class V4R_EXPORTS KeypointPoseDetectorRT {
 public:
  class V4R_EXPORTS Parameter {
   public:
    float nnr;
    bool compute_global_pose;
    RigidTransformationRANSAC::Parameter rt_param;  // 0.01 (slam: 0.03)
    Parameter(float _nnr = .9, bool _compute_global_pose = true,
              const RigidTransformationRANSAC::Parameter &_rt_param = RigidTransformationRANSAC::Parameter(0.01))
    : nnr(_nnr), compute_global_pose(_compute_global_pose), rt_param(_rt_param) {}
  };

 private:
  Parameter param;

  cv::Mat_<unsigned char> im_gray;
  cv::Mat descs;
  std::vector<cv::KeyPoint> keys;
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<Eigen::Vector3f> query_pts;
  std::vector<Eigen::Vector3f> model_pts;
  std::vector<int> inliers;

  ObjectView::Ptr model;

  // cv::Ptr<cv::BFMatcher> matcher;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  RigidTransformationRANSAC::Ptr rt;

  v4r::FeatureDetector::Ptr detector;
  v4r::FeatureDetector::Ptr descEstimator;

 public:
  cv::Mat dbg;

  KeypointPoseDetectorRT(const Parameter &p = Parameter(),
                         const v4r::FeatureDetector::Ptr &_detector = v4r::FeatureDetector::Ptr(),
                         const v4r::FeatureDetector::Ptr &_descEstimator = new v4r::FeatureDetector_KD_FAST_IMGD(
                             v4r::FeatureDetector_KD_FAST_IMGD::Parameter(1000, 1.44, 2, 17)));
  ~KeypointPoseDetectorRT();

  double detect(const cv::Mat &image, const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, Eigen::Matrix4f &pose);

  void setModel(const ObjectView::Ptr &_model);

  typedef SmartPtr<::v4r::KeypointPoseDetectorRT> Ptr;
  typedef SmartPtr<::v4r::KeypointPoseDetectorRT const> ConstPtr;
};

/***************************** inline methods *******************************/

}  //--END--

#endif
