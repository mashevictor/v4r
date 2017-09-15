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
 * @file RansacSolvePnPdepth.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */


#ifndef KP_RANSAC_SOLVE_PNP_DEPTH_HH
#define KP_RANSAC_SOLVE_PNP_DEPTH_HH

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/random.hpp>
#include "v4r/keypoints/RigidTransformationRANSAC.h"
#include <v4r/core/macros.h>



namespace v4r
{


/**
 * RansacSolvePnPdepth
 */
class V4R_EXPORTS RansacSolvePnPdepth
{
public:
  class Parameter
  {
  public:
    double inl_dist_px;
    double eta_ransac;              // eta for pose ransac
    unsigned max_rand_trials;       // max. number of trials for pose ransac
    int pnp_method;                 // cv::ITERATIVE, cv::P3P
    int nb_ransac_points;
    double inl_dist_z;              // depth value inlier dist
    bool use_robust_loss;
    double loss_scale;
    double depth_error_scale;
    Parameter(double _inl_dist_px=3, double _eta_ransac=0.01, unsigned _max_rand_trials=5000,
      int _pnp_method=INT_MIN, int _nb_ransac_points=4, double _inl_dist_z=0.03)
    : inl_dist_px(_inl_dist_px), eta_ransac(_eta_ransac), max_rand_trials(_max_rand_trials),
      pnp_method(_pnp_method), nb_ransac_points(_nb_ransac_points), inl_dist_z(_inl_dist_z),
      use_robust_loss(true), loss_scale(1.5), depth_error_scale(50) {}
  };


private:
  Parameter param;
  boost::mt19937 rg;

  float sqr_inl_dist_px;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;
  std::vector<double> lm_intrinsics;
  
  cv::Mat_<unsigned char> im_gray;
  std::vector< cv::Point2f > im_points;
  std::vector< int > inliers;
  std::vector<float> inv_depth;
  std::vector<Eigen::Vector3d> points3d;
  Eigen::Matrix<double, 6, 1> pose_Rt;
  std::vector<cv::Point3f> cv_pts0;
  std::vector<int> ind3d;

  RigidTransformationRANSAC rt;

  void getRandIdx(int size, int num, std::vector<int> &idx);
  unsigned countInliers(const std::vector<Eigen::Vector3f> &points, const std::vector<cv::Point2f> &im_points, const std::vector<float> &_inv_depth, const Eigen::Matrix4f &pose);
  void getInliers(const std::vector<Eigen::Vector3f> &points, const std::vector<cv::Point2f> &im_points, const std::vector<float> &_inv_depth, const Eigen::Matrix4f &pose, std::vector<int> &inliers);
  void convertToLM(const std::vector<Eigen::Vector3f> &points, Eigen::Matrix4f &pose);
  void convertFromLM(Eigen::Matrix4f &pose);
  void optimizePoseLM(std::vector<Eigen::Vector3d> &_points3d, const std::vector<cv::Point2f> &_im_points, const std::vector<float> &_inv_depth, Eigen::Matrix<double, 6, 1> &_pose_Rt, const std::vector<int> &_inliers);


  inline void cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose);
  inline bool contains(const std::vector<int> &idx, int num);



public:
  cv::Mat dbg;

  RansacSolvePnPdepth(const Parameter &p=Parameter());
  ~RansacSolvePnPdepth();

  int ransac(const std::vector<Eigen::Vector3f> &points, const std::vector<cv::Point2f> &im_points, Eigen::Matrix4f &pose, std::vector<int> &inliers, const std::vector<float> &_depth=std::vector<float>());
  int ransac(const std::vector<Eigen::Vector3f> &_points0, const std::vector<cv::Point2f> &_im_points1, const std::vector<Eigen::Vector3f> &_points3d1, Eigen::Matrix4f &pose, std::vector<int> &inliers);
  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setParameter(const Parameter &_p);

  typedef boost::shared_ptr< ::v4r::RansacSolvePnPdepth> Ptr;
  typedef boost::shared_ptr< ::v4r::RansacSolvePnPdepth const> ConstPtr;
};


/***************************** inline methods *******************************/
/**
 * cvToEigen
 */
inline void RansacSolvePnPdepth::cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose)
{
  pose.setIdentity();

  pose(0,0) = R(0,0); pose(0,1) = R(0,1); pose(0,2) = R(0,2);
  pose(1,0) = R(1,0); pose(1,1) = R(1,1); pose(1,2) = R(1,2);
  pose(2,0) = R(2,0); pose(2,1) = R(2,1); pose(2,2) = R(2,2);

  pose(0,3) = t(0,0);
  pose(1,3) = t(1,0);
  pose(2,3) = t(2,0);
}

inline bool RansacSolvePnPdepth::contains(const std::vector<int> &idx, int num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}




} //--END--

#endif

