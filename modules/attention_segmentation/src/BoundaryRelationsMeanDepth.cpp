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
 * @file BoundaryRelationsMeanDepth.h
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Class to calculate boundary depth and standart deviation.
 */

#include "v4r/attention_segmentation/BoundaryRelationsMeanDepth.h"

namespace v4r {

/************************************************************************************
 * Constructor/Destructor
 */

BoundaryRelationsMeanDepth::BoundaryRelationsMeanDepth() : BoundaryRelationsBase() {}

BoundaryRelationsMeanDepth::~BoundaryRelationsMeanDepth() {}

v4r::meanVal BoundaryRelationsMeanDepth::compute() {
  //@ep: TODO check reconditions
  if (!have_cloud) {
    printf("[BoundaryRelationsMeanDepth::compute] Error: No input cloud set.\n");
    exit(0);
  }

  if (!have_boundary) {
    printf("[BoundaryRelationsMeanDepth::compute] Error: No input border.\n");
    exit(0);
  }

  v4r::meanVal meanDepth;

  if (boundary.size() <= 0) {
    printf(
        "[BoundaryRelationsMeanDepth::compute] Warning: Boundary size is 0. This means that constants are different "
        "everywhere!\n");
    meanDepth.mean = 0;
    meanDepth.stddev = 0;
    return meanDepth;
  }

  int boundaryLength = boundary.size();

  // calculate mean depth
  double totalDepth = 0.;
  double totalDepthStdDev = 0.;
  std::vector<double> valuesDepth;
  valuesDepth.reserve(boundaryLength);
  for (unsigned int i = 0; i < boundary.size(); i++) {
    pcl::PointXYZRGB p1 = cloud->points.at(boundary.at(i).idx1);
    pcl::PointXYZRGB p2 = cloud->points.at(boundary.at(i).idx2);

    if (checkNaN(p1) || checkNaN(p2)) {
      boundaryLength--;
      continue;
    }

    double depth = fabs(p1.z - p2.z);
    valuesDepth.push_back(depth);
    totalDepth += depth;
  }

  // normalize depth sum and calculate depth variance
  //@ep: this shoule be separate function in the utils
  if (boundaryLength > 0) {
    totalDepth /= boundaryLength;
    for (unsigned i = 0; i < valuesDepth.size(); i++) {
      totalDepthStdDev += fabs(valuesDepth.at(i) - totalDepth);
    }
    totalDepthStdDev /= boundaryLength;
  } else {
    std::printf(
        "[BoundaryRelationsMeanDepth::compute] Warning: Number of valid depth points is zero: totalDepth: %4.3f\n",
        totalDepth);
    totalDepth = 0.;
    totalDepthStdDev = 0.;
  }

  meanDepth.mean = totalDepth;
  meanDepth.stddev = totalDepthStdDev;

  return meanDepth;
}

}  // end surface
