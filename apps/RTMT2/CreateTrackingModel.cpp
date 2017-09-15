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

#include "CreateTrackingModel.h"
#include <pcl/common/io.h>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/toString.hpp>
#include <v4r/common/convertCloud.h>
#include <v4r/io/filesystem.h>
#include <v4r/keypoints/CodebookMatcher.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using namespace std;


/**
 * @brief CreateTrackingModel::CreateTrackingModel
 */
CreateTrackingModel::CreateTrackingModel() :
  create_codebook(1), thr_desc_rnn(0.55)
{
  cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

/**
 * @brief CreateTrackingModel::~CreateTrackingModel
 */
CreateTrackingModel::~CreateTrackingModel()
{
}



/******************************** public *******************************/

void CreateTrackingModel::setCameraParameter(const cv::Mat_<double> &_intrinsic, const cv::Mat_<double> &_dist_coeffs)
{
  _intrinsic.copyTo(intrinsic);
  _dist_coeffs.copyTo(dist_coeffs);
}






/**
 * @brief CreateTrackingModel::createTrackingModel
 */
void CreateTrackingModel::createTrackingModel(const std::vector<v4r::TSFFrame::Ptr> &map_frames, const std::vector< cv::Mat_<unsigned char> > &masks, const Eigen::Matrix4f &object_base_transform)
{
  Eigen::Matrix4f pose0, pose;

  cv::Mat_<cv::Vec3b> image;
  cv::Mat_<unsigned char> im_gray;
  v4r::DataMatrix2D<Eigen::Vector3f>::Ptr kp_cloud( new v4r::DataMatrix2D<Eigen::Vector3f>() );
  v4r::DataMatrix2D<Eigen::Vector3f>::Ptr kp_normals( new v4r::DataMatrix2D<Eigen::Vector3f>() );

  v4r::ZAdaptiveNormals::Parameter n_param;
  n_param.adaptive = true;
  v4r::ZAdaptiveNormals::Ptr nest(new v4r::ZAdaptiveNormals(n_param));

  //v4r::FeatureDetector_KD_FAST_IMGD::Parameter param(10000, 1.44, 2, 17);
  v4r::FeatureDetector_KD_FAST_IMGD::Parameter param(10000, 1.2, 6, 13);
  //v4r::FeatureDetector_KD_FAST_IMGD::Parameter param(10000, 1.32, 4, 15);
  param.do_feature_selection = true;
  keyDet.reset(new v4r::FeatureDetector_KD_FAST_IMGD(param));
  keyDesc = keyDet;

  model.reset(new v4r::ArticulatedObject());
  model->addCameraParameter(intrinsic, dist_coeffs);

  pose0 = object_base_transform;

  for (unsigned i=0; i<map_frames.size(); i++)
  {
    v4r::TSFData::convert(map_frames[i]->sf_cloud, *cloud);
    v4r::convertImage(*cloud, image);

    if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
    else im_gray = image;

    pose = map_frames[i]->pose;

    v4r::convertCloud(*cloud, *kp_cloud);
    nest->compute(*kp_cloud, *kp_normals);

    addObjectView(*kp_cloud, *kp_normals, im_gray, masks[i], pose*pose0, *model);

  }

  if (create_codebook==1)
  {
    v4r::CodebookMatcher cm = v4r::CodebookMatcher( v4r::CodebookMatcher::Parameter(thr_desc_rnn) );

    for (unsigned i=0; i<model->views.size(); i++)
      cm.addView(model->views[i]->descs,i);

    cm.createCodebook(model->cb_centers, model->cb_entries);
  }

  keyDet = v4r::FeatureDetector::Ptr();
  keyDesc = v4r::FeatureDetector::Ptr();
}

/**
 * @brief CreateTrackingModel::saveTrackingModel
 */
void CreateTrackingModel::saveTrackingModel(const std::string &folder, const std::string &objectname)
{
  if (model.get()==0)
      return;

  boost::filesystem::create_directories(folder);

  std::cout << "Writing model to " << folder << "/models/" << objectname << "/tracking_model.ao" << std::endl;

  std::string model_name = folder + "/models/" + objectname + "/tracking_model.ao";

  v4r::io::createDirForFileIfNotExist(model_name);
  v4r::io::write(model_name, model);

  std::cout << "Tracking model saved!" << std::endl;

  model = v4r::ArticulatedObject::Ptr();
}


/**
 * @brief CreateTrackingModel::addObjectView
 * @param cloud
 * @param normals
 * @param im
 * @param mask
 * @param pose
 * @param model
 */
void CreateTrackingModel::addObjectView(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const v4r::DataMatrix2D<Eigen::Vector3f> &normals, const cv::Mat_<unsigned char> &im, const cv::Mat_<unsigned char> &mask, const Eigen::Matrix4f &pose, v4r::ArticulatedObject &_model)
{
  // get and transform 3d points
  Eigen::Matrix4f inv_pose;
  Eigen::Vector3f pt_model, n_model, vr_model;
  static const unsigned MIN_KEYPOINTS = 20;

  v4r::invPose(pose, inv_pose);

  Eigen::Matrix3f R = inv_pose.topLeftCorner<3,3>();
  Eigen::Vector3f t = inv_pose.block<3, 1>(0,3);

  std::vector<cv::KeyPoint> keys;
  cv::Mat descs;
  unsigned cnt=0;

  // detect keypoints
  keyDet->detect(im, keys);
  keyDesc->extract(im, keys, descs);

  for (unsigned i=0; i<keys.size(); i++)
  {
    cv::KeyPoint &key = keys[i];

    int idx = int(key.pt.y+.5)*cloud.cols+int(key.pt.x+.5);

    const Eigen::Vector3f &pt =  cloud[idx];
    const Eigen::Vector3f &n = normals[idx];

    if(!isnan(pt[0]) && !isnan(n[0]) && mask(key.pt.y,key.pt.x)>128)
      cnt++;
  }


  if (cnt<MIN_KEYPOINTS) return;

  v4r::ObjectView &view = _model.addObjectView(pose, im);


  for (unsigned i=0; i<keys.size(); i++)
  {
    cv::KeyPoint &key = keys[i];

    int idx = int(key.pt.y+.5)*cloud.cols+int(key.pt.x+.5);

    const Eigen::Vector3f &pt =  cloud[idx];
    const Eigen::Vector3f &n = normals[idx];

    if(!isnan(pt[0]) && !isnan(n[0]) && mask(key.pt.y,key.pt.x)>128)
    {
      pt_model = R*pt + t;
      n_model = (R*n).normalized();
      vr_model = -(R*pt).normalized();

      view.add(keys[i], &descs.at<float>(i,0), descs.cols, pt_model, n_model, vr_model);
    }
  }

  view.descs.copyTo(descs);
  view.descs = descs;
}










