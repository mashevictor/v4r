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

#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/common/convertImage.h>

namespace v4r {

using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
TSFData::TSFData()
: need_init(false), init_points(0), lk_flags(0), timestamp(std::numeric_limits<uint64_t>::max()),
  pose(Eigen::Matrix4f::Identity()), have_pose(false), kf_timestamp(std::numeric_limits<uint64_t>::max()),
  kf_pose(Eigen::Matrix4f::Identity()), lost_track(false) {
  last_pose_map(0, 0) = std::numeric_limits<float>::quiet_NaN();
}

TSFData::~TSFData() {}

/***************************************************************************************/

/**
 * @brief TSFData::reset
 */
void TSFData::reset() {
  lock();
  need_init = false;
  init_points = 0;
  lk_flags = 0;
  gray = cv::Mat();
  prev_gray = cv::Mat();
  points[0].clear();
  points[1].clear();
  points3d[0].clear();
  points3d[1].clear();
  cloud.clear();
  pose = Eigen::Matrix4f::Identity();
  kf_pose = Eigen::Matrix4f::Identity();
  timestamp = std::numeric_limits<uint64_t>::max();
  kf_timestamp = std::numeric_limits<uint64_t>::max();
  have_pose = false;
  map_frames = std::queue<TSFFrame::Ptr>();
  lost_track = false;
  last_pose_map(0, 0) = std::numeric_limits<float>::quiet_NaN();
  unlock();
}

/**
 * @brief convert
 * @param sf_cloud
 * @param cloud
 * @param thr_weight
 * @param thr_delta_angle
 */
void TSFData::convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud,
                      const double &thr_weight, const double &thr_delta_angle) {
  cloud.resize(sf_cloud.data.size());
  cloud.width = sf_cloud.cols;
  cloud.height = sf_cloud.rows;
  cloud.is_dense = false;
  double cos_rad_thr_delta_angle = cos(thr_delta_angle * M_PI / 180.);
  for (unsigned i = 0; i < sf_cloud.data.size(); i++) {
    const v4r::Surfel &s = sf_cloud.data[i];
    pcl::PointXYZRGBNormal &o = cloud.points[i];
    if (s.weight >= thr_weight && s.n.dot(-s.pt.normalized()) > cos_rad_thr_delta_angle) {
      o.getVector3fMap() = s.pt;
      o.getNormalVector3fMap() = s.n;
    } else {
      o.getVector3fMap() =
          Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN());
      o.getNormalVector3fMap() =
          Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN());
    }
    o.r = s.r;
    o.g = s.g;
    o.b = s.b;
  }
}

/**
 * @brief convert
 * @param sf_cloud
 * @param cloud
 * @param thr_weight
 * @param thr_delta_angle
 */
void TSFData::convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, pcl::PointCloud<pcl::Normal> &cloud,
                      const double &thr_weight, const double &thr_delta_angle) {
  cloud.resize(sf_cloud.data.size());
  cloud.width = sf_cloud.cols;
  cloud.height = sf_cloud.rows;
  cloud.is_dense = false;
  double cos_rad_thr_delta_angle = cos(thr_delta_angle * M_PI / 180.);
  for (unsigned i = 0; i < sf_cloud.data.size(); i++) {
    const v4r::Surfel &s = sf_cloud.data[i];
    pcl::Normal &o = cloud.points[i];
    if (s.weight >= thr_weight && s.n.dot(-s.pt.normalized()) > cos_rad_thr_delta_angle) {
      o.getNormalVector3fMap() = s.n;
    } else {
      o.getNormalVector3fMap() =
          Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN());
    }
  }
}

void TSFData::convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                      const double &thr_weight, const double &thr_delta_angle) {
  cloud.resize(sf_cloud.data.size());
  cloud.width = sf_cloud.cols;
  cloud.height = sf_cloud.rows;
  cloud.is_dense = false;
  double cos_rad_thr_delta_angle = cos(thr_delta_angle * M_PI / 180.);
  for (unsigned i = 0; i < sf_cloud.data.size(); i++) {
    const v4r::Surfel &s = sf_cloud.data[i];
    pcl::PointXYZRGB &o = cloud.points[i];
    if (s.weight >= thr_weight && s.n.dot(-s.pt.normalized()) > cos_rad_thr_delta_angle) {
      o.getVector3fMap() = s.pt;
    } else {
      o.getVector3fMap() =
          Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN());
    }
    o.r = s.r;
    o.g = s.g;
    o.b = s.b;
  }
}

/**
 * @brief TSFData::convert
 * @param sf_cloud
 * @param image
 */
void TSFData::convert(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud, cv::Mat &image) {
  image = cv::Mat_<cv::Vec3b>(sf_cloud.rows, sf_cloud.cols);

  for (int v = 0; v < sf_cloud.rows; v++) {
    for (int u = 0; u < sf_cloud.cols; u++) {
      cv::Vec3b &cv_pt = image.at<cv::Vec3b>(v, u);
      const v4r::Surfel &s = sf_cloud(v, u);

      cv_pt[2] = (unsigned char)s.r;
      cv_pt[1] = (unsigned char)s.g;
      cv_pt[0] = (unsigned char)s.b;
    }
  }
}
}
