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
 * @file   entangled_tree.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#pragma once

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <array>
#include <fstream>
#include <memory>
#include <random>
#include <vector>

#include <boost/serialization/vector.hpp>

#include <v4r/core/macros.h>
#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_definitions.h>
#include <v4r/semantic_segmentation/entangled_node.h>
#include <v4r/semantic_segmentation/entangled_split_feature.h>

namespace v4r {

class EntangledForestNode;
class EntangledForestSplitFeature;

class V4R_EXPORTS EntangledForestTree {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    ar& mNodes;
    ar& mTreeIdx;
    //        ar & mClassWeights;
  }

  int mTreeIdx;

  // uniform distribution for threshold
  std::uniform_real_distribution<double> mUniformDistThresh;

  std::mt19937* mRandomGenerator;
  std::vector<EntangledForestNode*> mNodes;

  // only needed in training
  int mCurrentDepth;

  static bool SmallerThan(float x, float t);

  // weights to rebalance label dist at leaf nodes
  //    std::vector<double> mClassWeights;
 public:
  EntangledForestTree(int treeIdx = 0);
  EntangledForestTree(std::mt19937* randomGenerator, int treeIdx = 0);
  inline std::mt19937* GetRandomGenerator() {
    return mRandomGenerator;
  }
  void SetRandomGenerator(std::mt19937* randomGenerator);  // neccessary after loading from file
  EntangledForestNode* GetRootNode();

  inline int GetIndex() {
    return mTreeIdx;
  }
  void SetTreeIndex(int index);

  void Classify(EntangledForestData* data, std::vector<std::vector<double>>& result,
                int depth = -1);  // classify whole image

  void GetFeatureMinMax(EntangledForestSplitFeature* f, EntangledForestData* data, const ClusterIdxItr start,
                        const ClusterIdxItr end, const std::vector<double>& parameters, double& minValue,
                        double& maxValue);
  double GetBestFeatureConfiguration(EntangledForestSplitFeature* f, EntangledForestData* data, ClusterIdxItr start,
                                     ClusterIdxItr end, int nParameterSamples, int nThresholdSamples,
                                     int minPointsForSplit, double currentEntropy);

  int GetLastNodeIDOfPrevLevel();
  // int GetCurrentDepth();
  void Train(EntangledForestData* trainingData, int maxDepth, int sampledSplitFunctionParameters,
             int sampledSplitFunctionThresholds, double minInformationGAin, int minPointsForSplit);

  void UpdateLeafs(EntangledForestData* data, int updateDepth, double updateWeight);

  void saveMatlab(std::string filename);

  void Clone(EntangledForestTree* t);
  EntangledForestNode* GetNode(int nodeIdx);
  int GetNrOfNodes();
  bool DoNodesShareAncestor(int nodeIdx1, int nodeIdx2, int maxSteps);
  virtual ~EntangledForestTree();

  inline int GetCurrentDepth() {
    /*// if nodes available
    if(nodes.size() > 0)
    {
        // get depth of last node and decrease by 1 (== target depth)
        return nodes[nodes.size()-1]->GetDepth();
    }
    else
    {
        return -1;
    }*/

    return mCurrentDepth;
  }
};
}
