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
 * @file StoreTrackingModel.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */

#ifndef _STORE_TRACKING_MODEL_H
#define _STORE_TRACKING_MODEL_H

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/common/ZAdaptiveNormals.h>
#include <v4r/common/convertCloud.h>
#include <v4r/common/convertImage.h>
#include <v4r/features/FeatureDetectorHeaders.h>
#include <v4r/keypoints/ArticulatedObject.h>
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <v4r/keypoints/io.h>
#include <QMutex>
#include <QThread>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include "params.h"

class StoreTrackingModel : public QThread {
  Q_OBJECT

 public:
  enum Command { STORE_TRACKING_MODEL, MAX_COMMAND, UNDEF = MAX_COMMAND };

  StoreTrackingModel();
  ~StoreTrackingModel();

  void start();
  void stop();
  bool isRunning();

  void storeTrackingModel(
      const std::string &_folder, const std::string &_objectname,
      const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &_cameras,
      const boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> &_clouds,
      const std::vector<cv::Mat_<unsigned char>> &_masks,
      const Eigen::Matrix4f &_object_base_transform = Eigen::Matrix4f::Identity());

 public slots:
  void cam_params_changed(const RGBDCameraParameter &_cam_params);
  void set_object_base_transform(const Eigen::Matrix4f &_object_base_transform);
  void set_cb_param(bool create_cb, float rnn_thr);

 signals:
  void printStatus(const std::string &_txt);
  void finishedStoreTrackingModel();

 private:
  void run();

  void createTrackingModel();
  void saveTrackingModel();
  void addObjectView(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const v4r::DataMatrix2D<Eigen::Vector3f> &normals,
                     const cv::Mat_<unsigned char> &im, const cv::Mat_<unsigned char> &mask,
                     const Eigen::Matrix4f &pose, v4r::ArticulatedObject &model);
  void detectCoordinateSystem(Eigen::Matrix4f &pose);

  Command cmd;
  bool m_run;

  cv::Mat_<double> intrinsic;
  cv::Mat_<double> dist_coeffs;

  // data to compute the tracking model
  std::string folder;
  std::string objectname;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> cameras;
  boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> clouds;
  std::vector<cv::Mat_<unsigned char>> masks;

  Eigen::Matrix4f object_base_transform;

  int create_codebook;
  float thr_desc_rnn;

  v4r::ArticulatedObject::Ptr model;
  v4r::FeatureDetector::Ptr keyDet;
  v4r::FeatureDetector::Ptr keyDesc;
};

#endif
