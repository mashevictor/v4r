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

#ifndef KP_REFINE_PROJECTED_POINT_LOCATION_LK_HH
#define KP_REFINE_PROJECTED_POINT_LOCATION_LK_HH

#include <v4r/core/macros.h>
#include <v4r/reconstruction/RefineProjectedPointLocationLKbase.h>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <v4r/common/impl/SmartPtr.hpp>
#include <vector>

namespace v4r {

/**
 * RefineProjectedPointLocationLK
 */
class V4R_EXPORTS RefineProjectedPointLocationLK : public RefineProjectedPointLocationLKbase {
 public:
  class Parameter {
   public:
    float step_factor;  // 2.???
    float min_determinant;
    float min_displacement;  // 0.1
    int max_iterations;      // 10
    int max_residual;
    float ncc_residual;  // 0.3
    bool use_ncc;
    cv::Size patch_size;
    Parameter(float _step_factor = 10., float _min_determinant = 0.01, float _min_displacement = 0.1,
              int _max_iterations = 10, int _max_residual = 15., float _ncc_residual = 0.3, bool _use_ncc = true,
              const cv::Size &_patch_size = cv::Size(15, 15))
    : step_factor(_step_factor), min_determinant(_min_determinant), min_displacement(_min_displacement),
      max_iterations(_max_iterations), max_residual(_max_residual), ncc_residual(_ncc_residual), use_ncc(_use_ncc),
      patch_size(_patch_size) {}
  };

 private:
  Parameter param;

  int num_threads;
  int use_initial_flow;

  cv::Mat_<double> src_intrinsic, tgt_intrinsic;
  cv::Mat_<double> src_dist_coeffs, tgt_dist_coeffs;
  Eigen::Matrix3f src_C, tgt_C;

  cv::Mat_<unsigned char> im_src, im_tgt;
  cv::Mat_<float> im_tgt_dx, im_tgt_dy;
  Eigen::Matrix4f pose_src, pose_tgt;
  Eigen::Matrix4f inv_pose_tgt, delta_pose;
  Eigen::Matrix3f R_tgt, delta_R;
  Eigen::Vector3f t_tgt, delta_t;

  std::vector<float> residuals;

  void getIntensityDifference(const cv::Mat_<unsigned char> &im1, cv::Mat_<unsigned char> &im2, const cv::Point2f &pt1,
                              const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &diff);
  void getPatchInterpolated(const cv::Mat_<unsigned char> &image, const cv::Point2f &pt, cv::Mat_<unsigned char> &patch,
                            int width, int height);
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
  RefineProjectedPointLocationLK(const Parameter &p = Parameter());
  virtual ~RefineProjectedPointLocationLK();

  virtual void setSourceImage(const cv::Mat_<unsigned char> &_im_src, const Eigen::Matrix4f &_pose_src);
  virtual void setTargetImage(const cv::Mat_<unsigned char> &_im_tgt, const Eigen::Matrix4f &_pose_tgt);
  virtual void refineImagePoints(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals,
                                 std::vector<cv::Point2f> &im_pts_tgt, std::vector<int> &converged);

  virtual const std::vector<float> &getResiduals() const {
    return residuals;
  }

  virtual void setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  virtual void setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  void setParameter(const Parameter &p);
  void setNumberOfThreads(int n) {
    num_threads = n;
  }
  void useInitialFlow(int _use_initial_flow) {
    use_initial_flow = _use_initial_flow;
  }

  typedef SmartPtr<::v4r::RefineProjectedPointLocationLK> Ptr;
  typedef SmartPtr<::v4r::RefineProjectedPointLocationLK const> ConstPtr;
};

/*********************** INLINE METHODES **************************/

/**
 * getInterpolated
 * get a bilinear interpolated pixel
 */
inline float RefineProjectedPointLocationLK::getInterpolated(const cv::Mat_<unsigned char> &im, const float &x,
                                                             const float &y) {
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
inline float RefineProjectedPointLocationLK::getInterpolated(const cv::Mat_<float> &im, const float &x,
                                                             const float &y) {
  int xt = (int)x;
  int yt = (int)y;
  float ax = x - xt;
  float ay = y - yt;

  return ((1. - ax) * (1. - ay) * im(yt, xt) + ax * (1. - ay) * im(yt, xt + 1) + (1. - ax) * ay * im(yt + 1, xt) +
          ax * ay * im(yt + 1, xt + 1));
}
}

#endif
