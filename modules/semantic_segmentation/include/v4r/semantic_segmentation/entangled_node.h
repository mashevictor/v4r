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
 * @file   entangled_node.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include <boost/serialization/vector.hpp>

#include <v4r/core/macros.h>
#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_split_feature.h>
#include <v4r/semantic_segmentation/entangled_tree.h>

namespace v4r {

class EntangledForestSplitFeature;
class EntangledForestNode;
class EntangledForestTree;

class V4R_EXPORTS EntangledForestNode {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    ar& mParent;
    ar& mSplitFeature;
    ar& mDepth;
    ar& mLeftChildIdx;
    ar& mRightChildIdx;
    ar& mLabelDistribution;
    ar& mAbsLabelDistribution;
    ar& mIsSplitNode;
    ar& mOnFrontier;
  }

  EntangledForestNode* mParent;

  bool mOnFrontier;

  bool mIsSplitNode;
  EntangledForestSplitFeature* mSplitFeature;

  int mDepth;

  int mLeftChildIdx;
  int mRightChildIdx;

  std::vector<double> mLabelDistribution;
  std::vector<unsigned int> mAbsLabelDistribution;

  ClusterIdxItr SplitOnFeature(EntangledForestSplitFeature* f, EntangledForestData* data, ClusterIdxItr start,
                               ClusterIdxItr end);

 public:
  std::pair<ClusterIdxItr, ClusterIdxItr> mTrainingDataIterators;

  EntangledForestNode();
  EntangledForestNode(EntangledForestData* data);
  EntangledForestNode(EntangledForestData* data, ClusterIdxItr dataStart, ClusterIdxItr dataEnd,
                      EntangledForestNode* parent);

  inline unsigned int GetNrOfPoints() {
    return std::distance(mTrainingDataIterators.first, mTrainingDataIterators.second);
  }
  double GetWeightedNrOfPoints(std::vector<double>& classWeights);

  void ResetLabelDistribution();
  void ClearSplitNodeLabelDistribution();
  void AddToAbsLabelDistribution(int labelIdx);
  void UpdateLabelDistribution(std::vector<int> labels, std::map<int, unsigned int>& pointsPerLabel);

  void Split(EntangledForestData* data, EntangledForestSplitFeature* f, EntangledForestNode** leftChild,
             EntangledForestNode** rightChild);

  inline int GetDepth() {
    return mDepth;
  }
  inline int GetLeftChildIdx();
  inline int GetRightChildIdx();
  inline void SetLeftChildIdx(int idx);
  inline void SetRightChildIdx(int idx);
  inline std::vector<double>& GetLabelDistribution();
  inline std::vector<unsigned int>& GetAbsLabelDistribution();
  inline bool IsSplitNode();
  inline bool IsOnFrontier() {
    return mOnFrontier;
  }
  void SetAsLeafNode();
  bool IsDescendantOf(EntangledForestNode* n);
  int evaluate(EntangledForestData* data, int imageIdx, int clusterIdx);  // returns child idx
  EntangledForestNode* GetParent();

  EntangledForestSplitFeature* GetSplitFeature();

  void SetParent(EntangledForestNode* par);
  bool IsTopClass(int classIdx);
  bool IsAmongTopN(unsigned int classIdx, unsigned int N);
  void ApplyClusterLabelDistribution(std::vector<double>& labeldist);
  void UpdateLabelDistribution(std::vector<double>& labeldist);

  void SetRandomGenerator(std::mt19937* randomGenerator);  // neccessary after load

  void Clone(EntangledForestNode* n, EntangledForestTree* newTree);
  virtual ~EntangledForestNode();
};

inline std::vector<double>& EntangledForestNode::GetLabelDistribution() {
  return mLabelDistribution;
}

inline std::vector<unsigned int>& EntangledForestNode::GetAbsLabelDistribution() {
  return mAbsLabelDistribution;
}

inline bool EntangledForestNode::IsSplitNode() {
  return mIsSplitNode;
}

inline int EntangledForestNode::GetLeftChildIdx() {
  return mLeftChildIdx;
}

inline int EntangledForestNode::GetRightChildIdx() {
  return mRightChildIdx;
}

inline void EntangledForestNode::SetLeftChildIdx(int idx) {
  mLeftChildIdx = idx;
}

inline void EntangledForestNode::SetRightChildIdx(int idx) {
  mRightChildIdx = idx;
}
}
