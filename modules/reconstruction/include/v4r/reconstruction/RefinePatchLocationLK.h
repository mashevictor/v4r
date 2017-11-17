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

#ifndef KP_REFINE_PATCH_LOCATION_LK_HH
#define KP_REFINE_PATCH_LOCATION_LK_HH

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <v4r/common/impl/SmartPtr.hpp>
#include <vector>

namespace v4r {

/**
 * RefinePatchLocationLK
 */
class V4R_EXPORTS RefinePatchLocationLK {
 public:
  class V4R_EXPORTS Parameter {
   public:
    float step_factor;  // 2.???
    float min_determinant;
    float min_displacement;  // 0.1
    int max_iterations;      // 10
    int max_residual;
    Parameter(float _step_factor = 10., float _min_determinant = 0.01, float _min_displacement = 0.1,
              int _max_iterations = 10, int _max_residual = 15.)
    : step_factor(_step_factor), min_determinant(_min_determinant), min_displacement(_min_displacement),
      max_iterations(_max_iterations), max_residual(_max_residual) {}
  };

 private:
  Parameter param;

  cv::Mat_<unsigned char> im_gray;
  cv::Mat_<float> im_dx, im_dy;

  cv::Mat_<float> patch_dx, patch_dy, diff, sum_dx, sum_dy;
  cv::Mat_<unsigned char> roi_patch;
  cv::Mat_<float> roi_dx, roi_dy;

  cv::Mat_<unsigned char> im_src, im_tgt;
  Eigen::Matrix4f pose_src, pose_tgt;

  void getIntensityDifference(const cv::Mat_<unsigned char> &im1, cv::Mat_<unsigned char> &im2, const cv::Point2f &pt1,
                              const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &diff);
  void getGradientSum(const cv::Mat_<float> &dx1, const cv::Mat_<float> &dy1, const cv::Mat_<float> &dx2,
                      const cv::Mat_<float> &dy2, const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height,
                      cv::Mat_<float> &dx, cv::Mat_<float> &dy);
  void getGradientMatrix22(const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, float &gxx, float &gxy, float &gyy);
  void getErrorVector2(const cv::Mat_<float> &diff, const cv::Mat_<float> &dx, const cv::Mat_<float> &dy,
                       cv::Point2f &err);
  bool solve(const cv::Point2f &err, float gxx, float gxy, float gyy, cv::Point2f &delta);

  inline float getInterpolated(const cv::Mat_<unsigned char> &im, const float &x, const float &y);
  inline float getInterpolated(const cv::Mat_<float> &im, const float &x, const float &y);

 public:
  RefinePatchLocationLK(const Parameter &p = Parameter());
  ~RefinePatchLocationLK();

  bool optimize(const cv::Mat_<unsigned char> &patch, cv::Point2f &pt);
  void setImage(const cv::Mat_<unsigned char> &im);

  typedef SmartPtr<::v4r::RefinePatchLocationLK> Ptr;
  typedef SmartPtr<::v4r::RefinePatchLocationLK const> ConstPtr;
};

/*********************** INLINE METHODES **************************/

/**
 * getInterpolated
 * get a bilinear interpolated pixel
 */
inline float RefinePatchLocationLK::getInterpolated(const cv::Mat_<unsigned char> &im, const float &x, const float &y) {
  int xt = (int)x;
  int yt = (int)y;
  float ax = x - xt;
  float ay = y - yt;

  return ((1. - ax) * (1. - ay) * im(yt, xt) + ax * (1. - ay) * im(yt, xt + 1) + (1. - ax) * ay * im(yt + 1, xt) +
          ax * ay * im(yt + 1, xt + 1));
}

/**
 * getInterpolated
 * get a bilinear interpolated pixel
 */
inline float RefinePatchLocationLK::getInterpolated(const cv::Mat_<float> &im, const float &x, const float &y) {
  int xt = (int)x;
  int yt = (int)y;
  float ax = x - xt;
  float ay = y - yt;

  return ((1. - ax) * (1. - ay) * im(yt, xt) + ax * (1. - ay) * im(yt, xt + 1) + (1. - ax) * ay * im(yt + 1, xt) +
          ax * ay * im(yt + 1, xt + 1));
}
}

#endif
