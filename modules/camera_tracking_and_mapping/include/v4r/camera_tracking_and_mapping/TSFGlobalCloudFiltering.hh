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

#ifndef KP_GLOBAL_CLOUD_FILTERING_HH
#define KP_GLOBAL_CLOUD_FILTERING_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud.h>
#include <v4r/camera_tracking_and_mapping/OctreeVoxelCentroidContainerXYZRGBNormal.hpp>
#include <boost/smart_ptr.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>
#include <v4r/core/macros.h>


namespace v4r
{



/**
 * TSFGlobalCloudFiltering
 */
class V4R_EXPORTS TSFGlobalCloudFiltering 
{
public:

  /**
   * Parameter
   */
  class Parameter
  {
  public:
    float thr_angle;
    float max_weight;
    float sigma_pts;
    float sigma_depth;
    float z_cut_off_integration;
    float voxel_size;
    float max_dist_integration;
    Parameter()
      : thr_angle(80), max_weight(20), sigma_pts(10), sigma_depth(0.008), z_cut_off_integration(0.02), voxel_size(0.001), max_dist_integration(7.) {}
  };

 

private:
  Parameter param;

  float cos_thr_angle;
  float neg_inv_sqr_sigma_pts;

  cv::Mat_<double> intrinsic;

  std::vector<cv::Vec4i> npat;
  std::vector<float> exp_error_lookup;

  std::vector< cv::Mat_<float> > reliability;

  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBNormal,pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal> >::Ptr octree;
  typedef pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGBNormal,pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal> >::AlignedPointTVector AlignedPointXYZRGBNormalVector;
  boost::shared_ptr< AlignedPointXYZRGBNormalVector > oc_cloud;

//  void integrateData(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose, const Eigen::Matrix4f &filt_pose, v4r::DataMatrix2D<TSFData::Surfel> &filt_cloud);
  void computeReliability(const std::vector<TSFFrame::Ptr> &frames);
  void maxReliabilityIndexing(const std::vector<TSFFrame::Ptr> &frames);
  void getMaxPoints(const std::vector<TSFFrame::Ptr> &frames, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);
  void computeNormals(v4r::DataMatrix2D<v4r::Surfel> &sf_cloud);

  inline float sqr(const float &d) {return d*d;}


public:
  cv::Mat dbg;

  TSFGlobalCloudFiltering(const Parameter &p=Parameter());
  ~TSFGlobalCloudFiltering();

  void start();
  void stop();

  void computeRadius(v4r::DataMatrix2D<v4r::Surfel> &sf_cloud);

  void filter(const std::vector<TSFFrame::Ptr> &frames, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::Matrix4f &base_transform=Eigen::Matrix4f::Identity());

  void setCameraParameter(const cv::Mat &_intrinsic);
  void setParameter(const Parameter &p);

  typedef boost::shared_ptr< ::v4r::TSFGlobalCloudFiltering> Ptr;
  typedef boost::shared_ptr< ::v4r::TSFGlobalCloudFiltering const> ConstPtr;
};



/*************************** INLINE METHODES **************************/


} //--END--

#endif

