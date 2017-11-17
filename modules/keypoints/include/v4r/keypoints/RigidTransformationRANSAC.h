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

#ifndef KP_RIGID_TRANSFORMATION_RANSAC_HH
#define KP_RIGID_TRANSFORMATION_RANSAC_HH

#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <v4r/common/impl/SmartPtr.hpp>
#include <vector>

namespace v4r {

/**
 * RigidTransformationRANSAC
 */
class V4R_EXPORTS RigidTransformationRANSAC {
 public:
  class V4R_EXPORTS Parameter {
   public:
    double inl_dist;
    double eta_ransac;         // eta for pose ransac
    unsigned max_rand_trials;  // max. number of trials for pose ransac

    Parameter(double _inl_dist = 0.01, double _eta_ransac = 0.01, unsigned _max_rand_trials = 10000)
    : inl_dist(_inl_dist), eta_ransac(_eta_ransac), max_rand_trials(_max_rand_trials) {}
  };

 private:
  Eigen::Matrix4f invPose;

  void DemeanPoints(const std::vector<Eigen::Vector3f> &inPts, const std::vector<int> &indices,
                    const Eigen::Vector3f &centroid, Eigen::MatrixXf &outPts);

  void ComputeCentroid(const std::vector<Eigen::Vector3f> &pts, const std::vector<int> &indices,
                       Eigen::Vector3f &centroid);

  int Ransac(const std::vector<Eigen::Vector3f> &srcPts, const std::vector<Eigen::Vector3f> &tgtPts,
             Eigen::Matrix4f &transform, std::vector<int> &inliers);

  void GetDistances(const std::vector<Eigen::Vector3f> &srcPts, const std::vector<Eigen::Vector3f> &tgtPts,
                    const Eigen::Matrix4f &transform, std::vector<float> &dists);

  void GetInliers(std::vector<float> &dists, std::vector<int> &inliers);
  unsigned CountInliers(std::vector<float> &dists);
  void GetRandIdx(int size, int num, std::vector<int> &idx);

  inline bool Contains(const std::vector<int> &idx, int num);
  inline void InvPose(const Eigen::Matrix4f &pose, Eigen::Matrix4f &invPose);

 public:
  Parameter param;

  RigidTransformationRANSAC(Parameter p = Parameter());
  ~RigidTransformationRANSAC();

  void estimateRigidTransformationSVD(const std::vector<Eigen::Vector3f> &srcPts, const std::vector<int> &srcIndices,
                                      const std::vector<Eigen::Vector3f> &tgtPts, const std::vector<int> &tgtIndices,
                                      Eigen::Matrix4f &transform);

  int compute(const std::vector<Eigen::Vector3f> &srcPts, const std::vector<Eigen::Vector3f> &tgtPts,
              Eigen::Matrix4f &transform, std::vector<int> &inliers);

  typedef SmartPtr<::v4r::RigidTransformationRANSAC> Ptr;
  typedef SmartPtr<::v4r::RigidTransformationRANSAC const> ConstPtr;
};

/*********************** INLINE METHODES **************************/
inline bool RigidTransformationRANSAC::Contains(const std::vector<int> &idx, int num) {
  for (unsigned i = 0; i < idx.size(); i++)
    if (idx[i] == num)
      return true;
  return false;
}

inline void RigidTransformationRANSAC::InvPose(const Eigen::Matrix4f &pose, Eigen::Matrix4f &invPose_) {
  invPose_.setIdentity();
  invPose_.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
  invPose_.block<3, 1>(0, 3) = -1 * (invPose_.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3));
}
}

#endif
