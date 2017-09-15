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
 * @file GraphCut.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */

#ifndef GC_GRAPHCUT_H
#define GC_GRAPHCUT_H

#include <set>
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <algorithm>

#include "Graph.h"
#include "Edge.h"
#include "disjoint-set.h"

#include "v4r/attention_segmentation//Relation.h"
#include "v4r/attention_segmentation//SurfaceModel.h"

namespace gc
{

// THRESHOLD_CONSTANT:
// CAREFULL: Float value!!!
#define THRESHOLD_CONSTANT 0.5    /// good value 0.5 
#define MIN_SIZE 1                /// minimum size of element-sets

#define THRESHOLD(size, c) (c/size)

/**
 * @brief Class GraphCut
 */
class GraphCut
{
public:
  
private:
  bool print;
  bool createAllRelations;                          ///< create all combinatorial combinations of relations (if graph is not fully connected)

  bool initialized;                                 ///< flag to process
  bool processed;                                   ///< flag to get results
  unsigned num_edges;                               ///< Number of edges
  std::vector<int> surfaces_reindex2;

  bool have_surfaces;
  std::vector<v4r::SurfaceModel::Ptr> surfaces; ///< Surfaces
  
  bool have_relations;
  std::vector<v4r::Relation> relations;
  
  gc::Edge *edges;                                  ///< Edges between the nodes, representing a probability
  universe *u;                                      ///< universe to cut graph
  
  std::string ClassName;
  
  public:
  GraphCut();
  ~GraphCut();
  
  /** Initialize the graph cut algorithm with number of nodes and with all available relations **/
  bool init();

  /** Process graph cut **/
  void process();
  void process2();
  
  /** Print the results of the graph cut **/
  void printResults(bool _printResults) {print = _printResults;}
  
  void setSurfaces(const std::vector<v4r::SurfaceModel::Ptr> _surfaces);
  void setRelations(std::vector<v4r::Relation> _relations);
  
  /** Return modified surfaces **/
  inline std::vector<v4r::SurfaceModel::Ptr> getSurfaces();
};

inline std::vector<v4r::SurfaceModel::Ptr> GraphCut::getSurfaces()
{
  return surfaces;
}

}

#endif

