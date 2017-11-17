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

#ifndef NURBS_TOOLS_H
#define NURBS_TOOLS_H

#include <opennurbs/opennurbs.h>
#include "v4r/attention_segmentation/nurbs_data.h"

#undef Success
#include <Eigen/Dense>

namespace pcl {
namespace on_nurbs {

enum { NORTH = 1, NORTHEAST = 2, EAST = 3, SOUTHEAST = 4, SOUTH = 5, SOUTHWEST = 6, WEST = 7, NORTHWEST = 8 };

/** \brief Some useful tools for initialization, point search, ... */
class NurbsTools {
 public:
  //      static std::list<unsigned>
  //      getClosestPoints (const Eigen::Vector2d &p, const vector_vec2d &data, unsigned s);

  /** \brief Get the closest point with respect to 'point'
   *  \param[in] point The point to which the closest point is searched for.
   *  \param[in] data Vector containing the set of points for searching. */
  static unsigned getClosestPoint(const Eigen::Vector2d &point, const vector_vec2d &data);

  /** \brief Get the closest point with respect to 'point'
   *  \param[in] point The point to which the closest point is searched for.
   *  \param[in] data Vector containing the set of points for searching. */
  static unsigned getClosestPoint(const Eigen::Vector3d &point, const vector_vec3d &data);

  /** \brief Get the closest point with respect to 'point' in Non-Euclidean metric
   *  \brief Related paper: TODO
   *  \param[in] point The point to which the closest point is searched for.
   *  \param[in] dir The direction defining 'inside' and 'outside'
   *  \param[in] data Vector containing the set of points for searching.
   *  \param[out] idxcp Closest point with respect to Euclidean metric. */
  static unsigned getClosestPoint(const Eigen::Vector2d &point, const Eigen::Vector2d &dir, const vector_vec2d &data,
                                  unsigned &idxcp);

  /** \brief Compute the mean of a set of points
   *  \param[in] data Set of points.     */
  static Eigen::Vector3d computeMean(const vector_vec3d &data);
  static Eigen::Vector3d computeMean(const vector_vec3d &data, const std::vector<int> &indices);
  /** \brief Compute the mean of a set of points
   *  \param[in] data Set of points.     */
  static Eigen::Vector2d computeMean(const vector_vec2d &data);

  /** \brief Compute the variance of a set of points
   *  \param[in] data Set of points       */
  static Eigen::Vector3d computeVariance(const Eigen::Vector3d &mean, const vector_vec3d &data);
  /** \brief Compute the variance of a set of points
   *  \param[in] data Set of points       */
  static Eigen::Vector2d computeVariance(const Eigen::Vector2d &mean, const vector_vec2d &data);

  /** compute bounding box of curve control points */
  static void computeBoundingBox(const ON_NurbsCurve &nurbs, Eigen::Vector3d &_min, Eigen::Vector3d &_max);
  static void computeBoundingBox(const ON_NurbsSurface &nurbs, Eigen::Vector3d &_min, Eigen::Vector3d &_max);

  static double computeRScale(const Eigen::Vector3d &_min, const Eigen::Vector3d &_max);

  /** \brief PCA - principal-component-analysis
   *  \param[in] data Set of points.
   *  \param[in] indices Indices for point cloud.
   *  \param[out] mean The mean of the set of points.
   *  \param[out] eigenvectors Matrix containing column-wise the eigenvectors of the set of points.
   *  \param[out] eigenvalues The eigenvalues of the set of points with respect to the eigenvectors. */
  static void pca(const vector_vec3d &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors,
                  Eigen::Vector3d &eigenvalues);
  static void pca(const vector_vec3d &data, const std::vector<int> &indices, Eigen::Vector3d &mean,
                  Eigen::Matrix3d &eigenvectors, Eigen::Vector3d &eigenvalues);

  /** \brief PCA - principal-component-analysis
   *  \param[in] data Set of points.
   *  \param[out] mean The mean of the set of points.
   *  \param[out] eigenvectors Matrix containing column-wise the eigenvectors of the set of points.
   *  \param[out] eigenvalues The eigenvalues of the set of points with respect to the eigenvectors. */
  static void pca(const vector_vec2d &data, Eigen::Vector2d &mean, Eigen::Matrix2d &eigenvectors,
                  Eigen::Vector2d &eigenvalues);

  /** \brief Downsample data points to a certain size.
   *  \param[in] data1 The original set of points.
   *  \param[out] data2 The downsampled set of points of size 'size'.
   *  \param[in] size The desired size of the resulting set of points.       */
  static void downsample_random(const vector_vec3d &data1, vector_vec3d &data2, unsigned size);
  /** \brief Downsample data points to a certain size.
   *  \param[in/out] data1 The set of points for downsampling;
   *  will be replaced by the resulting set of points of size 'size'.
   *  \param[in] size The desired size of the resulting set of points.       */
  static void downsample_random(vector_vec3d &data1, unsigned size);
};
}
}

#endif /* NTOOLS_H_ */
