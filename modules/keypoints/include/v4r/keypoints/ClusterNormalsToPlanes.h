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

#ifndef KP_CLUSTER_NORMALS_TO_PLANES_HH
#define KP_CLUSTER_NORMALS_TO_PLANES_HH

#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <set>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/common/impl/SmartPtr.hpp>
#include <vector>
#include "v4r/keypoints/PlaneEstimationRANSAC.h"

namespace v4r {

/**
 * ClusterNormalsToPlanes
 */
class V4R_EXPORTS ClusterNormalsToPlanes {
 public:
  /**
   * @brief The Parameter class
   */
  class V4R_EXPORTS Parameter {
   public:
    double thrAngle;  // Threshold of angle for normal clustering
    double inlDist;   // Maximum inlier distance
    int minPoints;    // Minimum number of points for a plane
    bool least_squares_refinement;
    bool smooth_clustering;
    double thrAngleSmooth;  // Threshold of angle for normal clustering
    double inlDistSmooth;   // Maximum inlier distance
    int minPointsSmooth;

    Parameter(double thrAngleNC = 30, double _inlDist = 0.01, int _minPoints = 9, bool _least_squares_refinement = true,
              bool _smooth_clustering = false, double _thrAngleSmooth = 30, double _inlDistSmooth = 0.02,
              int _minPointsSmooth = 3)
    : thrAngle(thrAngleNC), inlDist(_inlDist), minPoints(_minPoints),
      least_squares_refinement(_least_squares_refinement), smooth_clustering(_smooth_clustering),
      thrAngleSmooth(_thrAngleSmooth), inlDistSmooth(_inlDistSmooth), minPointsSmooth(_minPointsSmooth) {}
  };

  /**
   * @brief The Plane class
   */
  class V4R_EXPORTS Plane {
   public:
    bool is_plane;
    Eigen::Vector3f pt;
    Eigen::Vector3f normal;
    std::vector<int> indices;
    /** clear **/
    void clear() {
      pt.setZero();
      normal.setZero();
      indices.clear();
    }
    /** init **/
    inline void init(const Eigen::Vector3f &_pt, const Eigen::Vector3f &_n, int idx) {
      pt = _pt;
      normal = _n;
      indices.resize(1);
      indices[0] = idx;
    }
    /** add **/
    inline void add(const Eigen::Vector3f &_pt, const Eigen::Vector3f &_n, int idx) {
      pt *= float(indices.size());
      normal *= float(indices.size());
      pt += _pt;
      normal += _n;
      indices.push_back(idx);
      pt /= float(indices.size());
      normal /= float(indices.size());
    }
    /** size **/
    inline unsigned size() {
      return indices.size();
    }
    /** Plane **/
    Plane(bool _is_plane = false) : is_plane(_is_plane) {}
    ~Plane() {}
    typedef SmartPtr<::v4r::ClusterNormalsToPlanes::Plane> Ptr;
    typedef SmartPtr<::v4r::ClusterNormalsToPlanes::Plane const> ConstPtr;
  };

 private:
  Parameter param;
  float cos_rad_thr_angle, cos_rad_thr_angle_smooth;

  std::vector<bool> mask;
  std::vector<int> queue;

  PlaneEstimationRANSAC pest;

  // cluster normals
  void doClustering(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const v4r::DataMatrix2D<Eigen::Vector3f> &normals,
                    std::vector<Plane::Ptr> &planes);
  // cluster normals from point
  void clusterNormals(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud,
                      const v4r::DataMatrix2D<Eigen::Vector3f> &normals, int idx, Plane &plane);
  // do a smooth clustering
  void smoothClustering(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud,
                        const v4r::DataMatrix2D<Eigen::Vector3f> &normals, int idx, Plane &plane);
  // least square plane
  void computeLeastSquarePlanes(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, std::vector<Plane::Ptr> &planes);
  // adds normals to each point of segmented patches

  inline bool isnan(const Eigen::Vector3f &pt);

 public:
  ClusterNormalsToPlanes(const Parameter &_p = Parameter());
  ~ClusterNormalsToPlanes();

  /** Compute planes by surface normal grouping **/
  void compute(const v4r::DataMatrix2D<Eigen::Vector3f> &_cloud, const v4r::DataMatrix2D<Eigen::Vector3f> &_normals,
               std::vector<Plane::Ptr> &planes);

  /** Compute a plane starting from a seed point **/
  void compute(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const v4r::DataMatrix2D<Eigen::Vector3f> &normals,
               int x, int y, Plane &plane);

  typedef SmartPtr<::v4r::ClusterNormalsToPlanes> Ptr;
  typedef SmartPtr<::v4r::ClusterNormalsToPlanes const> ConstPtr;
};

/**
 * @brief ClusterNormalsToPlanes::isnan
 * @param pt
 * @return
 */
inline bool ClusterNormalsToPlanes::isnan(const Eigen::Vector3f &pt) {
  if (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]))
    return true;
  return false;
}
}

#endif
