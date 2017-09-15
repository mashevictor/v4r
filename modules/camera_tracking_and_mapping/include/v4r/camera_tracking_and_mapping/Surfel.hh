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

#ifndef KP_TSF_SURFEL_HH
#define KP_TSF_SURFEL_HH

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <v4r/core/macros.h>

namespace v4r
{



/**
   * @brief The Surfel class
   */
class V4R_EXPORTS Surfel
{
public:
  Eigen::Vector3f pt;
  Eigen::Vector3f n;
  float weight;
  float radius;
  int r, g, b;
  Surfel()
    : pt(Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN())),
      n(Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN())),
      weight(0), radius(0) {}
  Surfel(const pcl::PointXYZRGB &_pt) : pt(_pt.getArray3fMap()), weight(1), radius(0), r(_pt.r), g(_pt.g), b(_pt.b) {
    if (!std::isnan(pt[0]) && !std::isnan(pt[1]) &&!std::isnan(pt[2])) {
      n = -pt.normalized();
    }
    else
    {
      n = Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
      weight = 0;
    }
  }
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

