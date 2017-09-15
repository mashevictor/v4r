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

#ifndef KP_TSF_DATA_HH
#define KP_TSF_DATA_HH

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/common/impl/DataMatrix2D.hpp> 
#include <v4r/camera_tracking_and_mapping/Surfel.hh>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>
#include <queue>
#include <v4r/core/macros.h>

namespace v4r
{



/**
 * TSFData
 */
class V4R_EXPORTS TSFData 
{
public:
  boost::mutex mtx_shm;

  //// -> track (tf), -> init-tracking, -> integrate cloud

  bool need_init;
  int init_points;
  int lk_flags;

  cv::Mat image;
  cv::Mat prev_gray, gray;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;  ///// new cloud
  uint64_t timestamp;

  Eigen::Matrix4f pose;      /// global pose of the current frame (depth, gray, points[1], ....)
  bool have_pose;

  // used by klt pose tracker
  uint64_t kf_timestamp;
  Eigen::Matrix4f kf_pose;   /// pose of the keyframe (points[0], points3d[0], normals, prev_gray
  std::vector<cv::Point2f> points[2];
  std::vector<Eigen::Vector3f> points3d[2];

  // log frames for mapping (if activated)
  std::queue<TSFFrame::Ptr> map_frames;
  Eigen::Matrix4f last_pose_map;
  bool lost_track;

  TSFData();
  ~TSFData();

  void reset();

  inline void lock() { mtx_shm.lock(); }
  inline void unlock() { mtx_shm.unlock(); }

  static void convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const double &thr_weight=-1000000, const double &thr_delta_angle=180. );
  static void convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, pcl::PointCloud<pcl::Normal> &cloud, const double &thr_weight=-1000000, const double &thr_delta_angle=180. );
  static void convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, pcl::PointCloud<pcl::PointXYZRGB> &cloud, const double &thr_weight=-1000000, const double &thr_delta_angle=180. );
  static void convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, cv::Mat &image);
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

