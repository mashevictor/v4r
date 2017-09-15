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
 * @file IMKRecognizer.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef KP_OBJECT_RECOGNIZER_HH
#define KP_OBJECT_RECOGNIZER_HH

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/keypoints/CodebookMatcher.h>
#include <v4r/features/FeatureDetector.h>
#include <v4r/features/ImGradientDescriptor.h>
#include <v4r/keypoints/impl/triple.hpp>
#include "IMKView.h"
#include "IMKObjectVotesClustering.h"
#include "RansacSolvePnPdepth.h"



namespace v4r
{


/**
 * IMKRecognizer
 */
class V4R_EXPORTS IMKRecognizer
{
public:
  class Parameter
  {
  public:
    int use_n_clusters;
    double min_cluster_size;
    int image_size_conf_desc;
    CodebookMatcher::Parameter cb_param;
    IMKObjectVotesClustering::Parameter vc_param;
    RansacSolvePnPdepth::Parameter pnp_param;
    Parameter(int _use_n_clusters=10,
      const CodebookMatcher::Parameter &_cb_param=CodebookMatcher::Parameter(0.25, .98, 1.),
      const RansacSolvePnPdepth::Parameter &_pnp_param=RansacSolvePnPdepth::Parameter())
    : use_n_clusters(_use_n_clusters), min_cluster_size(5), image_size_conf_desc(66),
      cb_param(_cb_param), pnp_param(_pnp_param){}
  };


private:
  Parameter param;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;
  
  cv::Mat image;
  cv::Mat_<cv::Vec3b> im_lab;
  std::vector< cv::Mat_<unsigned char> > im_channels;
  cv::Mat_<unsigned char> im_warped_scaled;
  std::vector<cv::Mat_<unsigned char> > ims_warped;
  std::vector< cv::Point2f > im_points;
  std::vector< int > inliers;
  std::vector<int> cnt_view_matches;
  std::vector<float> desc;

  cv::Mat descs;
  std::vector<cv::KeyPoint> keys;
  std::vector< std::vector< cv::DMatch > > matches;
  std::vector< boost::shared_ptr<v4r::triple<unsigned, double, std::vector< cv::DMatch > > > > clusters; // <object_id, clustered matches>
  std::vector< cv::Point2f > query_pts;
  std::vector< cv::Point3f > model_pts;

  std::string base_dir;
  std::string codebookFilename;
  std::vector<std::string> object_names;
  std::vector<IMKView> object_models;

  CodebookMatcher::Ptr cbMatcher;
  v4r::FeatureDetector::Ptr detector;
  v4r::FeatureDetector::Ptr descEstimator;

  ImGradientDescriptor cp;

  v4r::IMKObjectVotesClustering votesClustering;
  v4r::RansacSolvePnPdepth pnp;

  void createObjectModel(const unsigned &idx);
  bool loadObjectIndices(const std::string &_filename, cv::Mat_<unsigned char> &_mask, const cv::Size &_size);
  void convertImage(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &_image);
  void addView(const unsigned &idx, const std::vector<cv::KeyPoint> &keys, const cv::Mat &descs, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat_<unsigned char> &mask, const Eigen::Matrix4f &pose, Eigen::Vector3d &centroid, unsigned &cnt);
  void poseEstimation(const std::vector< cv::Mat_<unsigned char> > &_im_channels, const std::vector<std::string> &object_names, const std::vector<IMKView> &views, const std::vector<cv::KeyPoint> &keys, const cv::Mat &descs,
                      const std::vector< std::vector< cv::DMatch > > &matches,
                      const std::vector< boost::shared_ptr<v4r::triple<unsigned, double, std::vector< cv::DMatch > > > > &clusters,
                      std::vector<v4r::triple<std::string, double, Eigen::Matrix4f> > &objects, const pcl::PointCloud<pcl::PointXYZRGB> &_cloud);
  int getMaxViewIndex(const std::vector<IMKView> &views, const std::vector<cv::DMatch> &matches, const std::vector<int> &inliers);
  void getNearestNeighbours(const Eigen::Vector2f &pt, const std::vector<cv::KeyPoint> &keys, const float &sqr_inl_radius_conf, std::vector<int> &nn_indices);
  float getMinDescDist32F(const cv::Mat &desc, const cv::Mat &descs, const std::vector<int> &indices);
  void setViewDescriptor(const std::vector< cv::Mat_<unsigned char> > &_im_channels, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat_<unsigned char> &mask, const Eigen::Matrix4f &pose, IMKView &view);
  double computeGradientHistogramConf(const std::vector< cv::Mat_<unsigned char> > &_im_channels, const IMKView &view, const Eigen::Matrix4f &pose);




public:
  cv::Mat dbg;

  IMKRecognizer(const Parameter &p,
                           const v4r::FeatureDetector::Ptr &_detector,
                           const v4r::FeatureDetector::Ptr &_descEstimator);
  ~IMKRecognizer();

  void recognize(const cv::Mat &_image, std::vector<v4r::triple<std::string, double, Eigen::Matrix4f> > &objects);
  void recognize(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, std::vector<v4r::triple<std::string, double, Eigen::Matrix4f> > &objects);

  void clear();
  void setDataDirectory(const std::string &_base_dir);
  void setCodebookFilename(const std::string &_codebookFilename);
  void addObject(const std::string &_object_name);
  void initModels();

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef boost::shared_ptr< ::v4r::IMKRecognizer> Ptr;
  typedef boost::shared_ptr< ::v4r::IMKRecognizer const> ConstPtr;
};


/***************************** inline methods *******************************/



} //--END--

#endif

