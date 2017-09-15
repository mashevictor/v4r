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

#ifndef V4R_CLUSTERING_HH
#define V4R_CLUSTERING_HH

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <v4r/core/macros.h>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/common/impl/SmartPtr.hpp>


namespace v4r 
{

/**
 * Cluster
 */
class V4R_EXPORTS Cluster
{
public:
  float sqr_sigma;
  Eigen::VectorXf data;
  std::vector<int> indices;

  Cluster() : sqr_sigma(0) {};
  Cluster(const Eigen::VectorXf &d) : sqr_sigma(0), data(d) {}
  Cluster(const Eigen::VectorXf &d, int idx) : sqr_sigma(0), data(d) {
    indices.push_back(idx);
  }

  typedef SmartPtr< ::v4r::Cluster> Ptr;
  typedef SmartPtr< ::v4r::Cluster const> ConstPtr;
};

/**
 * Clustering
 */
class V4R_EXPORTS Clustering
{
protected:
  std::vector< Cluster::Ptr > clusters;

public:
  Clustering() {}
  virtual ~Clustering() {}

  virtual void cluster(const DataMatrix2Df &) = 0;
  virtual void getClusters(std::vector<std::vector<int> > &) = 0;
  virtual void getCenters(DataMatrix2Df &) = 0;
};

}

#endif

