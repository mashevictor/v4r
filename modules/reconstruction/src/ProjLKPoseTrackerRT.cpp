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

#include <v4r/reconstruction/ProjLKPoseTrackerRT.h>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include "opencv2/video/tracking.hpp"

#if CV_MAJOR_VERSION < 3
#define HAVE_OCV_2
#else
#include <opencv2/core/ocl.hpp>
#endif

namespace v4r {

using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
ProjLKPoseTrackerRT::ProjLKPoseTrackerRT(const Parameter &p) : param(p) {
  plk.reset(new RefineProjectedPointLocationLK(p.plk_param));
  rt.reset(new RigidTransformationRANSAC(param.rt_param));
}

ProjLKPoseTrackerRT::~ProjLKPoseTrackerRT() {}

/******************************* PUBLIC ***************************************/

/**
 * detect
 */
double ProjLKPoseTrackerRT::detect(const cv::Mat &image, const DataMatrix2D<Eigen::Vector3f> &cloud,
                                   Eigen::Matrix4f &pose) {
  if (model.get() == 0)
    throw std::runtime_error("[ProjLKPoseTrackerRT::detect] No model available!");
  if (src_intrinsic.empty() || tgt_intrinsic.empty())
    throw std::runtime_error("[ProjLKPoseTrackerRT::detect] Intrinsic camera parameter not set!");

#ifndef HAVE_OCV_2
  cv::ocl::setUseOpenCL(false);
#endif

  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray, CV_RGB2GRAY);
  else
    im_gray = image;

  ObjectView &m = *model;

  cv::Mat_<double> R(3, 3), rvec, tvec;
  std::vector<Eigen::Vector3f> points, normals;
  std::vector<int> lk_inliers, rt_inliers;
  model_pts.clear();
  query_pts.clear();

  if (param.compute_global_pose) {
    m.getPoints(points);
    m.getNormals(normals);
  } else {
    points = m.cam_points;
    m.getNormals(normals);
    Eigen::Matrix3f _R = model->getCamera().topLeftCorner<3, 3>();
    for (unsigned i = 0; i < normals.size(); i++)
      normals[i] = _R * normals[i];
  }

  inliers.clear();

  // tracking
  plk->setTargetImage(im_gray, pose);

  plk->refineImagePoints(points, normals, im_points, converged);

  for (unsigned z = 0; z < im_points.size(); z++) {
    if (converged[z] == 1) {
      const cv::Point2f &im_pt = im_points[z];
      const Eigen::Vector3f &pt = cloud(int(im_pt.y + .5), int(im_pt.x + .5));
      if (!isnan(pt[0])) {
        lk_inliers.push_back(z);
        query_pts.push_back(pt);
        model_pts.push_back(points[z]);
        if (!dbg.empty())
          cv::line(dbg, model->keys[z].pt, im_points[z], CV_RGB(0, 0, 0));
      }
    }
  }

  if (int(query_pts.size()) < 4)
    return 0.;

  rt->compute(model_pts, query_pts, pose, rt_inliers);

  for (unsigned i = 0; i < rt_inliers.size(); i++) {
    inliers.push_back(lk_inliers[rt_inliers[i]]);
    if (!dbg.empty())
      cv::circle(dbg, im_points[inliers.back()], 2, CV_RGB(255, 255, 0));
  }

  return double(inliers.size()) / double(model->points.size());
}

/**
 * getProjections
 * @param im_pts <model_point_index, projection>
 */
void ProjLKPoseTrackerRT::getProjections(std::vector<std::pair<int, cv::Point2f>> &im_pts) {
  im_pts.clear();
  if (model.get() == 0 || im_points.size() != model->points.size() || inliers.size() > im_points.size())
    return;

  for (unsigned i = 0; i < inliers.size(); i++)
    im_pts.push_back(make_pair(inliers[i], im_points[inliers[i]]));
}

/**
 * setModel
 */
void ProjLKPoseTrackerRT::setModel(const ObjectView::Ptr &_model, const Eigen::Matrix4f &_pose) {
  model = _model;
  im_points.resize(model->keys.size());

  if (param.compute_global_pose)
    plk->setSourceImage(model->image, _pose);
  else
    plk->setSourceImage(model->image, Eigen::Matrix4f::Identity());
}

/**
 * setSourceCameraParameter
 */
void ProjLKPoseTrackerRT::setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs) {
  src_dist_coeffs = cv::Mat_<double>();
  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(src_intrinsic, CV_64F);
  else
    src_intrinsic = _intrinsic;
  if (!_dist_coeffs.empty()) {
    src_dist_coeffs = cv::Mat_<double>::zeros(1, 8);
    for (int i = 0; i < _dist_coeffs.cols * _dist_coeffs.rows; i++)
      src_dist_coeffs(0, i) = _dist_coeffs.at<double>(0, i);
  }
  plk->setSourceCameraParameter(src_intrinsic, src_dist_coeffs);
}

/**
 * setTargetCameraParameter
 */
void ProjLKPoseTrackerRT::setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs) {
  tgt_dist_coeffs = cv::Mat_<double>();
  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(tgt_intrinsic, CV_64F);
  else
    tgt_intrinsic = _intrinsic;
  if (!_dist_coeffs.empty()) {
    tgt_dist_coeffs = cv::Mat_<double>::zeros(1, 8);
    for (int i = 0; i < _dist_coeffs.cols * _dist_coeffs.rows; i++)
      tgt_dist_coeffs(0, i) = _dist_coeffs.at<double>(0, i);
  }
  plk->setTargetCameraParameter(tgt_intrinsic, tgt_dist_coeffs);
}
}
