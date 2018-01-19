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
 * @file MultiSession.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */

#ifndef _MULTI_SESSION_H
#define _MULTI_SESSION_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>
#include <QMutex>
#include <QThread>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include "sensor.h"
#endif

class MultiSession : public QThread {
  Q_OBJECT

 public:
  enum Command { MULTI_SESSION_ALIGNMENT, MULTI_SESSION_MULTI_VIEW, MAX_COMMAND, UNDEF = MAX_COMMAND };

  MultiSession();
  ~MultiSession();

  void start();
  void stop();
  bool isRunning();
  void alignSequences();
  void optimizeSequences();
  void addSequences(
      const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &_cameras,
      const boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> &_clouds,
      const std::vector<std::vector<int>> &_object_indices,
      const Eigen::Matrix4f &_object_base_transform = Eigen::Matrix4f::Identity());
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &getCameras() {
    return cameras;
  }
  const boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> &getClouds() const {
    return clouds;
  }
  const std::vector<cv::Mat_<unsigned char>> &getMasks() {
    return masks;
  }
  void clear();
  bool savePointClouds(const std::string &_folder, const std::string &_modelname);
  void setUseFeatures(bool b) {
    use_features_ = b;
  }
  void setUseStablePlanes(bool b) {
    use_stable_planes_ = b;
  }

 public slots:
  void object_modelling_parameter_changed(const ObjectModelling &param);

 signals:
  void printStatus(const std::string &_txt);
  void finishedAlignment(bool ok);
  void update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> &_oc_cloud);
  void update_visualization();

 private:
  Command cmd;
  bool m_run;

  ObjectModelling om_params;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> cameras;
  boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> clouds;
  std::vector<cv::Mat_<unsigned char>> masks;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> output_poses;

  std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> normals;
  std::vector<std::pair<int, int>> session_ranges_;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> sessions_clouds_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> sessions_cloud_poses_;
  std::vector<std::vector<int>> sessions_cloud_indices_;

  bool use_features_;
  bool use_stable_planes_;

  double vx_size;
  double max_dist;
  int max_iterations;
  int diff_type;
  double max_point_dist;

  pcl::PointCloud<pcl::Normal>::Ptr big_normals;
  boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> oc_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr octree_cloud;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ncloud_filt;

  void run();

  void createMask(const std::vector<int> &indices, cv::Mat_<unsigned char> &mask, int width, int height);
  void optimizeDenseMultiview();
  void createObjectCloudFiltered();

  inline bool isnan(const Eigen::Vector3f &pt);
};

/**
 * @brief MultiSession::isnan
 * @param pt
 * @return
 */
inline bool MultiSession::isnan(const Eigen::Vector3f &pt) {
  return std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]);
}

#endif
