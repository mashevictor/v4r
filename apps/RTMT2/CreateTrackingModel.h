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

#ifndef _CREATE_TRACKING_MODEL_H
#define _CREATE_TRACKING_MODEL_H


#include <queue>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include "params.h"
#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/common/convertCloud.h>
#include <v4r/keypoints/ArticulatedObject.h>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/common/convertImage.h>
#include <v4r/features/FeatureDetectorHeaders.h>
#include <v4r/keypoints/io.h>
#include <v4r/common/ZAdaptiveNormals.h>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>
#include <v4r/camera_tracking_and_mapping/TSFData.h>



class CreateTrackingModel
{

public:

  CreateTrackingModel();
  ~CreateTrackingModel();

  void createTrackingModel(const std::vector<v4r::TSFFrame::Ptr> &map_frames, const std::vector< cv::Mat_<unsigned char> > &masks, const Eigen::Matrix4f &object_base_transform);
  void saveTrackingModel(const std::string &folder, const std::string &objectname);
  void setCameraParameter(const cv::Mat_<double> &_intrinsic, const cv::Mat_<double> &_dist_coeffs);


private:

  void addObjectView(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const v4r::DataMatrix2D<Eigen::Vector3f> &normals,
                     const cv::Mat_<unsigned char> &im, const cv::Mat_<unsigned char> &mask,
                     const Eigen::Matrix4f &pose, v4r::ArticulatedObject &model);
  void detectCoordinateSystem(Eigen::Matrix4f &pose);

  cv::Mat_<double> intrinsic;
  cv::Mat_<double> dist_coeffs;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  int create_codebook;
  float thr_desc_rnn;

  v4r::ArticulatedObject::Ptr model;
  v4r::FeatureDetector::Ptr keyDet;
  v4r::FeatureDetector::Ptr keyDesc;

};

#endif
