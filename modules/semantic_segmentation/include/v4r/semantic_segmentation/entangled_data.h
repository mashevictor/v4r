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
 * @file   entangled_data.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#pragma once

#include <array>
#include <iomanip>
#include <random>
#include <unordered_map>
#include <vector>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <v4r/core/macros.h>
#include <v4r/semantic_segmentation/entangled_definitions.h>

namespace v4r {
class V4R_EXPORTS EntangledForestData {
 private:
  // data containers
  std::vector<unsigned int> mNrOfClusters;                       // number of clusters for each image
  std::vector<std::vector<int>> mClusterLabels;                  // class label for each cluster of each image
  std::vector<std::vector<std::vector<double>>> mUnaryFeatures;  // unary features for each cluster of each image
  std::vector<std::vector<std::vector<std::pair<double, int>>>>
      mPairwiseVangle;  // pairwise vertical angle difference between all clusters of each image
  std::vector<std::vector<std::vector<std::pair<double, int>>>>
      mPairwiseHangle;  // pairwise horizontal angle difference between all clusters of each image
  std::vector<std::vector<std::vector<std::pair<double, int>>>> mPairwiseEdist;  // pairwise euclidean distance between
                                                                                 // all clusters of each image (single
                                                                                 // linkage to closest supervoxel
                                                                                 // centroid)
  std::vector<std::vector<std::vector<std::pair<double, int>>>> mPairwisePtPldist;   // pairwise point to plane distance
                                                                                     // between all clusters of each
                                                                                     // image (closest supervoxel
                                                                                     // centroid)
  std::vector<std::vector<std::vector<std::pair<double, int>>>> mPairwiseIPtPldist;  // inverse pairwise point to plane
                                                                                     // distance between all clusters of
                                                                                     // each image (closest supervoxel
                                                                                     // centroid)

  std::vector<double> mClassWeights;
  int mNrOfLabels;
  int mNrOfImages;
  int mTotalNrOfClusters;

  std::vector<long long int> mClustersPerLabel;  // how many clusters are available in the training set for each label
  std::vector<std::vector<unsigned int>> mLabelsPerImage;  // how often is every label found in each image

  std::vector<std::vector<int>> mBags;  // each bag is a vector of image indices

  ClusterIndices mClusters;  // contains all available clusters (image idx + cluster idx)
  std::vector<std::vector<std::vector<int>>> mClusterNodeIdxs;  // keeps track of the current node of each cluster in
                                                                // each tree (image idx -> cluster idx -> tree idx)

  std::map<int, int> mLabelMap;          // maps class labels to sorted label idx
  std::vector<std::string> mLabelNames;  // names corresponding to label index

  void FillPairwiseFeatures();
  bool LoadPairwiseFeatures(std::string pairwiseDir, std::vector<std::string> &imagefiles,
                            std::vector<std::vector<std::vector<std::pair<double, int>>>> &pairwiseFeatures,
                            std::string featurefileending);
  bool LoadUnaryFeatures(std::string unaryDir, std::vector<std::string> &imagefiles);
  bool LoadLabels(std::string labelDir, std::vector<std::string> &imagefiles);

  bool LoadPairwiseFeaturesBinary(std::string pairwiseDir, std::vector<std::string> &imagefiles,
                                  std::vector<std::vector<std::vector<std::pair<double, int>>>> &pairwiseFeatures,
                                  std::string featurefileending);
  bool LoadUnaryFeaturesBinary(std::string unaryDir, std::vector<std::string> &imagefiles);
  bool LoadLabelsBinary(std::string labelDir, std::vector<std::string> &imagefiles);

  bool LoadLabelNames(std::string labelNameFile);

  static bool PairwiseComparator(const std::pair<double, int> &l, const std::pair<double, int> r);
  static bool SmallerThan(const std::pair<double, int> &l, const double &r);
  static bool LargerThan(const double &r, const std::pair<double, int> &l);
  static int SecondElement(const std::pair<double, int> &p);

  // OLD STUFF STARTS HERE //////////////////

  double mNormFactor;
  double CheckIfAllLabelsAvailable(std::vector<int> imageIndices);

 public:
  EntangledForestData();
  bool LoadTrainingData(std::string trainingDataDir, std::string idxfile, std::string labelNameFile = std::string());
  bool LoadTestData(std::string dataDir, std::string filename);
  bool LoadTestDataLive(std::vector<std::vector<double>> &unaryFeaturesLive,
                        std::vector<std::vector<std::pair<double, int>>> &pointPlaneDistancesLive,
                        std::vector<std::vector<std::pair<double, int>>> &inversePointPlaneDistancesLive,
                        std::vector<std::vector<std::pair<double, int>>> &verticalAngleDifferencesLive,
                        std::vector<std::vector<std::pair<double, int>>> &horizontalAngleDifferencesLive,
                        std::vector<std::vector<std::pair<double, int>>> &euclideanDistancesLive);
  bool LoadGroundtruth(std::string gtDir, std::string filename, std::vector<int> &gt);

  inline int GetNrOfClusters(int imageIdx) {
    return mNrOfClusters[imageIdx];
  }
  inline int GetNrOfLabels() {
    return mNrOfLabels;
  }
  inline std::map<int, int> &GetLabelMap() {
    return mLabelMap;
  }
  inline std::vector<std::string> &GetLabelNames() {
    return mLabelNames;
  }
  void SetLabelMap(std::map<int, int> &labelMapping);

  void CalculateHistogram(const ClusterIdxItr dataBegin, const ClusterIdxItr dataEnd, std::vector<unsigned int> &hist);
  double CalculateEntropy(const std::vector<unsigned int> &unnormalizedHistogram, double &entropy);
  double CalculateEnergy(const std::vector<unsigned int> &unnormalizedHistogram, double &energy);
  double CalculateEntropy(ClusterIdxItr dataBegin, ClusterIdxItr dataEnd, double &entropy);
  double CalculateEnergy(ClusterIdxItr dataBegin, ClusterIdxItr dataEnd, double &energy);

  void GetBeginAndEndIterator(ClusterIdxItr &begin, ClusterIdxItr &end);

  void CalculateAbsoluteLabelDistribution(ClusterIdxItr dataBegin, ClusterIdxItr dataEnd,
                                          std::vector<unsigned int> &labelDistribution);
  void CalculateRelativeLabelDistribution(ClusterIdxItr dataBegin, ClusterIdxItr dataEnd,
                                          std::vector<double> &labelDistribution);
  void CalculateLabelDistributions(ClusterIdxItr dataBegin, ClusterIdxItr dataEnd,
                                   std::vector<unsigned int> &absLabelDistribution,
                                   std::vector<double> &relLabelDistribution);

  void AddTreesToClusterNodeIdx(int ntrees);
  void ResetClusterNodeIdxs();
  void ResetClusterNodeIdxs(int imageIdx);

  void SetClusterNodeIdx(int imageIdx, int clusterIdx, int treeIdx, int nodeIdx);
  void SetClusterNodeIdx(ClusterIdx &datapoint, int treeIdx, int nodeIdx);
  inline int GetClusterNodeIdx(int imageIdx, int clusterIdx, int treeIdx) {
    return mClusterNodeIdxs[imageIdx][clusterIdx][treeIdx];
  }

  inline int GetLabel(ClusterIdx &datapoint) {
    return GetLabel(datapoint[0], datapoint[1]);
  }
  inline int GetLabel(int imageIdx, int clusterIdx) {
    return mClusterLabels[imageIdx][clusterIdx];
  }
  inline int GetLabelIdx(ClusterIdx &datapoint) {
    return GetLabelIdx(datapoint[0], datapoint[1]);
  }
  inline int GetLabelIdx(int imageIdx, int clusterIdx) {
    return mLabelMap[mClusterLabels[imageIdx][clusterIdx]];
  }

  inline std::vector<double> &GetClassWeights() {
    return mClassWeights;
  }
  inline double GetUnaryFeature(ClusterIdx &datapoint, int featureIdx) {
    return GetUnaryFeature(datapoint[0], datapoint[1], featureIdx);
  }
  inline double GetUnaryFeature(int imageIdx, int clusterIdx, int featureIdx) {
    return mUnaryFeatures[imageIdx][clusterIdx][featureIdx];
  }

  void FilterClustersByGeometry(int imageIdx, int clusterIdx, bool horizontal, double minAngleDiff, double maxAngleDiff,
                                double minPtPlaneDist, double maxPtPlaneDist, double maxEuclidDist,
                                std::vector<int> &remainingClusters);
  void FilterClustersByInversePtPl(int imageIdx, int clusterIdx, double minAngle, double maxAngle,
                                   double minIPtPlaneDist, double maxIPtPlaneDist, double maxEuclidDist,
                                   std::vector<int> &remainingClusters);

  void UpdateClusterNodeIndices(int imageIdx, int treeIdx, std::vector<int> &clusterNodeIndices);
  void UpdateClusterNodeIndicesPerTree(int treeIdx, std::vector<std::vector<int>> &clusterNodeIndices);
  void GetClusterNodeIndices(int treeIdx, std::vector<int> &clusterNodeIndices);
  void GetClusterNodeIndicesPerTree(int treeIdx, std::vector<std::vector<int>> &clusterNodeIndices);

  // OLD STUFF STARTS HERE //////

  void NewBag(std::mt19937 *randomGenerator, double baggingRatio);

  void GenerateBags(std::mt19937 *randomGenerator, double baggingRatio, int trees, bool tryUniformBags);
  void LoadBag(int bagidx);
};
}
