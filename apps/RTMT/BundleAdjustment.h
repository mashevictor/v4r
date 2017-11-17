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
 * @file BundleAdjustment.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */

#ifndef _BUNDLE_ADJUSTMENT_H
#define _BUNDLE_ADJUSTMENT_H

#ifndef Q_MOC_RUN
#include <QMutex>
#include <QThread>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/reconstruction/ProjBundleAdjuster.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include "params.h"
#include "sensor.h"
#endif

class BundleAdjustment : public QThread {
  Q_OBJECT

 public:
  enum Command { PROJ_BA_CAM_STRUCT, MAX_COMMAND, UNDEF = MAX_COMMAND };

  BundleAdjustment();
  ~BundleAdjustment();

  void start();
  void stop();
  bool isRunning();

  void optimizeCamStructProj(
      v4r::Object::Ptr &_model, boost::shared_ptr<std::vector<Sensor::CameraLocation>> &_cam_trajectory,
      boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> &_log_clouds,
      boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> &_oc_cloud);
  bool restoreCameras();

 public slots:
  void cam_tracker_params_changed(const CamaraTrackerParameter &_cam_tracker_params);

 signals:
  void printStatus(const std::string &_txt);
  void update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> &_oc_cloud);
  void update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> &_cam_trajectory);
  void update_visualization();
  void finishedOptimizeCameras(int num_cameras);

 private:
  void run();
  void optimizeCamStructProj();
  void renewPrevCloud(const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &poses,
                      const std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>> &clouds);

  Command cmd;
  bool m_run;

  CamaraTrackerParameter cam_tracker_params;

  v4r::Object::Ptr model;
  boost::shared_ptr<std::vector<Sensor::CameraLocation>> cam_trajectory;
  boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> log_clouds;
  boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> oc_cloud;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> stored_cameras;
  std::vector<std::vector<double>> stored_camera_parameter;
  std::vector<v4r::GlobalPoint> stored_points;
};

#endif
