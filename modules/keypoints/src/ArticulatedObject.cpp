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

#include <v4r/keypoints/ArticulatedObject.h>

namespace v4r {

/* clear */
void ArticulatedObject::clearArticulatedObject() {
  clear();
  part_parameter.clear();
  parts.clear();
}

/* add a new object view */
ObjectView &ArticulatedObject::addArticulatedView(const Eigen::Matrix4f &_pose, const cv::Mat_<unsigned char> &im,
                                                  const std::vector<Eigen::VectorXd> &_part_parameter) {
  part_parameter.push_back(_part_parameter);
  return addObjectView(_pose, im);
}

/* add projections */
void ArticulatedObject::addArticulatedProjections(ObjectView &view,
                                                  const std::vector<std::pair<int, cv::Point2f>> &im_pts,
                                                  const Eigen::Matrix4f &pose_,
                                                  const std::vector<Eigen::VectorXd> &_part_parameter) {
  addProjections(view, im_pts, pose_);
  part_parameter.push_back(_part_parameter);
}

/* get parameters of the articulated parts */
void ArticulatedObject::getParameters(std::vector<Eigen::VectorXd> &_params) {
  _params.resize(parts.size());

  for (unsigned i = 0; i < parts.size(); i++)
    _params[i] = parts[i]->getParameter();
}

/* set parameters of the articulated parts */
void ArticulatedObject::setParameters(const std::vector<Eigen::VectorXd> &_params) {
  if (parts.size() != _params.size())
    throw std::runtime_error("[ArticulatedObjectModel::setParameters] Invalid number of parameters!");

  for (unsigned i = 0; i < parts.size(); i++)
    parts[i]->setParameter(_params[i]);
}

/* updatePoseRecursive */
void ArticulatedObject::updatePoseRecursive(const Eigen::Matrix4f &_pose, Part &part, std::vector<Part::Ptr> &_parts) {
  part.updatePose(_pose);
  for (unsigned i = 0; i < part.subparts.size(); i++) {
    updatePoseRecursive(part.pose, *_parts[part.subparts[i]], _parts);
  }
}

/* updatePoseRecursive */
void ArticulatedObject::updatePoseRecursive(const Eigen::Matrix4f &_pose) {
  updatePose(_pose);
  for (unsigned i = 0; i < subparts.size(); i++) {
    updatePoseRecursive(pose, *parts[subparts[i]], parts);
  }
}

/* getKinematicChain */
void ArticulatedObject::getChainRecursive(const Part &part, const std::vector<Part::Ptr> &parts_, int _idx,
                                          std::vector<std::vector<int>> &kinematics) {
  kinematics[part.idx] = kinematics[_idx];
  kinematics[part.idx].push_back(part.idx);

  for (unsigned i = 0; i < part.subparts.size(); i++)
    getChainRecursive(*parts_[part.subparts[i]], parts_, part.idx, kinematics);
}

/* getKinematicChain */
void ArticulatedObject::getKinematicChain(std::vector<std::vector<int>> &kinematics) {
  kinematics.clear();
  kinematics.resize(parts.size());
  kinematics[this->idx].push_back(this->idx);

  for (unsigned i = 0; i < subparts.size(); i++)
    getChainRecursive(*parts[subparts[i]], parts, this->idx, kinematics);
}

/* getFeatures */
void ArticulatedObject::getFeatures(int part_idx, int view_idx, FeatureGroup &features_) {
  features_.part_idx = part_idx;
  features_.view_idx = view_idx;

  features_.clear();
  ObjectView &view = *views[view_idx];
  Part &part = *parts[part_idx];

  for (unsigned i = 0; i < part.features.size(); i++) {
    std::pair<int, int> &f = part.features[i];
    if (f.first == view_idx)
      features_.push_back(view.keys[f.second].pt, view.getPt(f.second).pt.cast<float>(),
                          view.getPt(f.second).n.cast<float>(), i, f.second);
  }
}

/**
 * getDescriptors
 */
void ArticulatedObject::getDescriptors(const FeatureGroup &features_, cv::Mat &descs) {
  descs = cv::Mat();
  if (features_.points.size() == 0)
    return;

  cv::Mat &dst = views[features_.view_idx]->descs;

  descs = cv::Mat_<float>(features_.view_feature_indices.size(), dst.cols);

  for (unsigned i = 0; i < features_.view_feature_indices.size(); i++) {
    std::memcpy(descs.ptr<float>(i, 0), dst.ptr<float>(features_.view_feature_indices[i], 0), dst.cols * sizeof(float));
  }
}

/**
 * addCamera
 */
int ArticulatedObject::addCamera(const std::vector<Eigen::VectorXd> &_part_parameter) {
  if (_part_parameter.size() == 0)
    return -1;

  Eigen::Matrix4f _pose;

  convertPose(_part_parameter[0], _pose);
  part_parameter.push_back(_part_parameter);
  cameras.push_back(_pose);

  return part_parameter.size() - 1;
}

}  //--END--
