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

#ifndef KP_OPTIMIZE_BUNDLE_COLOUR_HH
#define KP_OPTIMIZE_BUNDLE_COLOUR_HH

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>
#include <v4r/core/macros.h>
#include <radical/radiometric_response.h>



namespace v4r
{

/**
 * @brief The TukeyLoss class
 */
class TukeyLoss : public ceres::LossFunction {
 public:
  explicit TukeyLoss(double a) : a_squared_(a * a) { }
  virtual void Evaluate(double, double*) const;

 private:
  const double a_squared_;
};


/**
 * OptimizeBundleColours
 */
class V4R_EXPORTS OptimizeBundleColours 
{
public:

  /**
   * Parameter
   */
  class Parameter
  {
  public:
    bool use_robust_loss;
    double loss_scale;
    int nb_col_samples_per_linked_frame;
    unsigned max_sampling_trials;
    bool use_projection_links;
    int saturation_offs_low, saturation_offs_high;
    int grad_threshold;
    double inv_inl_dist;
    Parameter()
      : use_robust_loss(true), loss_scale(0.1), nb_col_samples_per_linked_frame(10000), max_sampling_trials(10000000),
      use_projection_links(false), saturation_offs_low(5),  saturation_offs_high(250), grad_threshold(50), inv_inl_dist(0.1){
      //path_to_crf_file = std::string("ps1080.1403190110-sa-cam.crf");
    }
  };

private:
  Parameter param;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;

  cv::Mat_<unsigned char> mask;

  std::vector< double > colour_scales_data;
  std::vector< std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > > rgb_correspondences; /// size of map: <rgb_current_frame, frame_idx, rgb_frame_idx>

  std::vector<cv::Mat> lin_images;
  std::vector< cv::Mat_<unsigned char> > edges;

  std::string path_to_crf_file;

  void sampleCorrespondences(const std::vector<TSFFrame::Ptr> &map, const std::vector<cv::Mat> &_lin_images);
  void getLinkedFrames(const unsigned &idx, const std::vector< std::vector< v4r::triple<int, cv::Point2f, Eigen::Vector3f > > >  &projections, std::set<int> &links);
  void sampelColourCorrespondences(const std::vector<TSFFrame::Ptr> &map, const std::vector<cv::Mat> &_lin_images, int idx0, int idx1, std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > &corrs);
  void optimizeColourScales(int idx);

  bool pickCol(const Eigen::Vector3f &pt, const Eigen::Matrix4f &delta_pose, const cv::Mat &im_lin, cv::Point2f &_im_pt, cv::Vec3f &col);
  bool pickCol(const Eigen::Vector3f &pt, const Eigen::Matrix4f &delta_pose, const v4r::DataMatrix2D<Surfel> &sf_cloud, const cv::Mat &im_lin, cv::Point2f &_im_pt, cv::Vec3f &col);



public:
  OptimizeBundleColours( const Parameter &p=Parameter());
  ~OptimizeBundleColours();

  void optimize(const std::vector<TSFFrame::Ptr> &map);

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setParameter(const Parameter &p=Parameter());
  void setCRFfile(const std::string &file) { path_to_crf_file = file; }
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

