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
 * @file Graph.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Create graph from 3D features in KinectCore.
 */

#ifndef GC_GRAPH_H
#define GC_GRAPH_H

#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "v4r/attention_segmentation/Edge.h"
#include "v4r/attention_segmentation/Relation.h"
#include "v4r/attention_segmentation/SurfaceModel.h"

namespace gc {

template <typename T1, typename T2>
extern T1 Dot3(const T1 v1[3], const T2 v2[3]);

template <typename T1, typename T2, typename T3>
extern void Add3(const T1 v1[3], const T2 v2[3], T3 r[3]);

/**
 * @brief Class Graph
 */
class Graph {
 private:
  unsigned nodes;               ///< number of surface patches
  std::vector<gc::Edge> edges;  ///< edges of the graph (wiht node numbers and probability)

  std::vector<v4r::Relation> relations;              ///< relations between features
  std::vector<v4r::SurfaceModel::Ptr> surfaces;      ///< relations between features
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;  ///< point cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;         ///< Normals of the point cloud

  std::vector<int> surfaces_reindex;
  // std::vector<int> surfaces_reindex2;

 public:
 private:
  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);

 public:
  Graph();
  Graph(unsigned nrNodes, std::vector<v4r::Relation> &rel);
  Graph(std::vector<v4r::SurfaceModel::Ptr> &surf, std::vector<v4r::Relation> &rel);
  ~Graph();

  void BuildFromSVM(std::vector<gc::Edge> &e, unsigned &num_edges, std::vector<int> &surfaces_reindex2);
  void BuildFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr &_normals, std::vector<gc::Edge> &e, unsigned &num_edges);
};

/*************************** INLINE METHODES **************************/
/** Return index for coordinates x,y **/
inline int Graph::GetIdx(short x, short y) {
  return y * pcl_cloud->width + x;
}

/** Return x coordinate for index **/
inline short Graph::X(int idx) {
  return idx % pcl_cloud->width;
}

/** Return y coordinate for index **/
inline short Graph::Y(int idx) {
  return idx / pcl_cloud->width;
}
}

#endif
