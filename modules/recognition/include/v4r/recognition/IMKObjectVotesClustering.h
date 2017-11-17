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
 * @file IMKObjectVotesClustering.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef KP_IMK_OBJECT_VOTES_CLUSTERING_HH
#define KP_IMK_OBJECT_VOTES_CLUSTERING_HH

#include <stdio.h>
#include <v4r/common/ClusteringRNN.h>
#include <v4r/recognition/IMKView.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <v4r/keypoints/impl/triple.hpp>

namespace v4r {

/**
 * IMKObjectVotesClustering
 */
class V4R_EXPORTS IMKObjectVotesClustering {
 public:
  class Parameter {
   public:
    double cluster_dist;
    Parameter() : cluster_dist(70) {}
  };

 private:
  Parameter param;

  std::vector<std::vector<int>> indices;
  DataMatrix2Df votes;
  std::vector<cv::DMatch> voting_matches;
  std::vector<double> weights;

  ClusteringRNN rnn;

  static const float two_pi;

  void createVotes(unsigned id, const std::vector<IMKView> &views, const std::vector<cv::KeyPoint> &keys,
                   const std::vector<std::vector<cv::DMatch>> &matches, DataMatrix2Df &votes,
                   std::vector<cv::DMatch> &voting_matches, std::vector<double> &weights);
  inline float diffAngle_0_2pi(float b, float a);
  inline float scaleAngle_0_2pi(float a);

 public:
  cv::Mat dbg;

  IMKObjectVotesClustering(const Parameter &p = Parameter());
  ~IMKObjectVotesClustering();

  void operate(const std::vector<std::string> &object_names, const std::vector<IMKView> &views,
               const std::vector<cv::KeyPoint> &keys, const std::vector<std::vector<cv::DMatch>> &matches,
               std::vector<boost::shared_ptr<v4r::triple<unsigned, double, std::vector<cv::DMatch>>>> &clusters);

  void setParameter(const Parameter &_param) {
    param = _param;
  }

  typedef boost::shared_ptr<::v4r::IMKObjectVotesClustering> Ptr;
  typedef boost::shared_ptr<::v4r::IMKObjectVotesClustering const> ConstPtr;
};

/***************************** inline methods *******************************/

inline float IMKObjectVotesClustering::diffAngle_0_2pi(float b, float a) {
  return IMKObjectVotesClustering::scaleAngle_0_2pi(b - a);
}

inline float IMKObjectVotesClustering::scaleAngle_0_2pi(float a) {
  while (a >= two_pi)
    a -= two_pi;
  while (a < 0.)
    a += two_pi;
  return a;
}

}  //--END--

#endif
