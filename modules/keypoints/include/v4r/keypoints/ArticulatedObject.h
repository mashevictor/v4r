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

#ifndef KP_ARTICULATED_OBJECT_HH
#define KP_ARTICULATED_OBJECT_HH

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Dense>
#include <v4r/keypoints/PartMotion6D.h>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/common/convertPose.h>

//A makro to get rid of the unused warning
#ifndef UNUSED
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

namespace v4r
{

/**
 * feature pairs to compute the pnp pose transformation
 */
class V4R_EXPORTS FeatureGroup
{
public:
  int part_idx;
  int view_idx;
  std::vector<cv::Point2f> im_points;
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> normals;
  std::vector< int > part_feature_indices;
  std::vector< int > view_feature_indices;
  FeatureGroup() {}
  /** clear **/
  inline void clear() { 
    im_points.clear();
    points.clear();
    normals.clear();
    part_feature_indices.clear();
    view_feature_indices.clear();
  }
  /** add data **/
  inline void push_back(const cv::Point2f &im_pt, const Eigen::Vector3f &pt3, const Eigen::Vector3f &n, int _part_feature_idx, int _view_feature_idx) {
    UNUSED(im_pt);
    points.push_back(pt3);
    normals.push_back(n);
    part_feature_indices.push_back(_part_feature_idx);
    view_feature_indices.push_back(_view_feature_idx);
  }
  /** resize **/
  inline void resize(int z) {
    im_points.resize(z);
    points.resize(z);
    normals.resize(z);
    part_feature_indices.resize(z);
    view_feature_indices.resize(z);
  }
  /** filter **/
  inline int filter(const std::vector<int> &valid_indices)
  {
    int z=0;
    for (unsigned i=0; i<valid_indices.size(); i++)
    {
      if (valid_indices[i]==1)
      {
        im_points[z] = im_points[i];
        points[z] = points[i];
        normals[z] = normals[i];
        part_feature_indices[z] = part_feature_indices[i];
        view_feature_indices[z] = view_feature_indices[i];
        z++;
      }
    }
    resize(z);
    return z;
  }
};



/*************************************************************************** 
 * ArticulatedObject 
 */
class V4R_EXPORTS ArticulatedObject : public Object, public PartMotion6D
{
public:
  std::string version;
  std::vector< std::vector< Eigen::VectorXd > > part_parameter; // parameter for articulated scenes (objects)

  std::vector<Part::Ptr> parts;         // the first part is the object itself

  ArticulatedObject() : version(std::string("1.0")) {};

  /* clear */
  void clearArticulatedObject();

  /* add a new object view */
  ObjectView &addArticulatedView(const Eigen::Matrix4f &_pose, const cv::Mat_<unsigned char> &im=cv::Mat_<unsigned char>(), const std::vector<Eigen::VectorXd> &_part_parameter=std::vector<Eigen::VectorXd>());

  /* add projections */
  void addArticulatedProjections(ObjectView &view, const std::vector< std::pair<int,cv::Point2f> > &im_pts, const Eigen::Matrix4f &pose, const std::vector<Eigen::VectorXd> &_part_parameter=std::vector<Eigen::VectorXd>());

  /* get parameters of the articulated parts */
  void getParameters(std::vector<Eigen::VectorXd> &_params);

  /* set parameters of the articulated parts */
  void setParameters(const std::vector<Eigen::VectorXd> &_params);

  /* updatePoseRecursive */
  void updatePoseRecursive(const Eigen::Matrix4f &_pose, Part &part, std::vector<Part::Ptr> &parts);

  /* updatePoseRecursive */
  void updatePoseRecursive(const Eigen::Matrix4f &_pose=Eigen::Matrix4f::Identity());

  /* getKinematicChain */
  void getChainRecursive(const Part &part, const std::vector<Part::Ptr> &parts, int idx, std::vector< std::vector<int> > &kinematics);


  /* getKinematicChain */
  void getKinematicChain(std::vector< std::vector<int> > &kinematics);

  /* getFeatures */
  void getFeatures(int part_idx, int view_idx, FeatureGroup &features);

  /** getDescriptors */
  void getDescriptors(const FeatureGroup &features, cv::Mat &descs);

  /** addCamera **/
  int addCamera(const std::vector<Eigen::VectorXd> &_part_parameter);

  typedef SmartPtr< ::v4r::ArticulatedObject> Ptr;
  typedef SmartPtr< ::v4r::ArticulatedObject const> ConstPtr;
};





} //--END--

#endif

