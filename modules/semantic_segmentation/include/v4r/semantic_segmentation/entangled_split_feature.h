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
 * @file   entangled_split_feature.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#pragma once

#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>

#include <v4r/core/macros.h>
#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_definitions.h>
#include <v4r/semantic_segmentation/entangled_tree.h>

//#define ENTANGLED_FOREST_MAX_FEATURE_VALUE  std::numeric_limits<double>::max() //__DBL_MAX__
//#define ENTANGLED_FOREST_MIN_FEATURE_VALUE  std::numeric_limits<double>::lowest() //-__DBL_MAX__
//#define NUM_COLOR_CHANNELS                  3
//#define CONTEXT_PIXEL_RANGE_1METER          200  // TODO: Test different ranges
#define MAX_STEPS_COMMON_ANCESTOR 10
#define MAX_N_TOP_CLASSES 4
#define MAX_PAIRWISE_DISTANCE 1.0                    // in meters
#define SIGMA_PAIRWISE_DISTANCE 0.2                  // in meters
#define SIGMA_PAIRWISE_ANGLE 3.141592654 * 10 / 180  // in radians

namespace v4r {

class EntangledForestTree;

class V4R_EXPORTS EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    ar& mTree;
    ar& mName;
    ar& mThreshold;
    ar& mDynamicThreshold;
    ar& mMaxParameterSamplings;
  }

  std::string mName;

 protected:
  EntangledForestTree* mTree;
  std::mt19937* mRandomGenerator;
  std::uniform_int_distribution<int> mCoinFlip;

  bool mDynamicThreshold;
  int mMaxParameterSamplings;
  double mThreshold;

  const int mMinDepthForFeature;
  const int mMaxDepthForFeature;
  std::vector<unsigned int> mActiveDepthLevels;

  bool IsAmongTopN(const std::vector<double> probabilities, int classIdx, int N);

 public:
  EntangledForestSplitFeature();
  // Feature(std::string name, EntangledForestTree* tree, std::mt19937* mRandomGenerator, const int minDepth, const int
  // maxDepth);
  EntangledForestSplitFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                              std::vector<unsigned int> activeDepthLevels = std::vector<unsigned int>());
  EntangledForestSplitFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                              const int minDepth, const int maxDepth = std::numeric_limits<int>::max());
  EntangledForestSplitFeature(const EntangledForestSplitFeature& f);

  int GetMinDepth() const {
    return mMinDepthForFeature;
  }
  int GetMaxDepth() const {
    return mMaxDepthForFeature;
  }
  std::vector<unsigned int>& GetActiveDepthLevels() {
    return mActiveDepthLevels;
  }
  bool HasDynamicThreshold();
  int GetMaxParameterSamples();
  std::string GetName();
  void SetThreshold(double t);
  void SetRandomGenerator(std::mt19937* randomGenerator);  // neccessary after load from file

  // to be overridden
  virtual std::string ToString() = 0;
  virtual void SetParameters(const std::vector<double>& parameters) = 0;
  virtual void SampleRandomParameters(std::vector<double>& parameters) = 0;

  virtual bool computeTraining(EntangledForestData* data, int imageIdx, int clusterIdx,
                               const std::vector<double>& parameters,
                               double& value);  // only overwritten by feature classes with threshold
  virtual bool evaluateTraining(EntangledForestData* data, int imageIdx, int clusterIdx,
                                const std::vector<double>& parameters,
                                bool& result);  // decision function for training, only overwritten by
  // feature classes without threshold

  virtual bool evaluateInference(EntangledForestData* data, int imageIdx,
                                 int clusterIdx) = 0;  // decision function for inference, has to be overwritten
  virtual EntangledForestSplitFeature* Clone() = 0;
  virtual EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) = 0;
};

class V4R_EXPORTS EntangledForestUnaryFeature : public EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    // serialize base class information
    ar& boost::serialization::base_object<EntangledForestSplitFeature>(*this);
    ar& mFeatureIdx;
  }

  int mFeatureIdx;  // which unary feature? height, orientation...?
  bool compute(EntangledForestData* data, int imageIdx, int clusterIdx, double& value);

 public:
  EntangledForestUnaryFeature();
  EntangledForestUnaryFeature(const EntangledForestUnaryFeature& f);
  EntangledForestUnaryFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                              int featureIdx);
  std::string ToString() override;
  EntangledForestSplitFeature* Clone() override;
  EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) override;
  void SetParameters(const std::vector<double>& parameters) override;
  void SampleRandomParameters(std::vector<double>& parameters) override;
  bool computeTraining(EntangledForestData* data, int imageIdx, int clusterIdx, const std::vector<double>& parameters,
                       double& value) override;
  bool evaluateInference(EntangledForestData* data, int imageIdx, int clusterIdx) override;

  inline int GetFeatureIdx() {
    return mFeatureIdx;
  }
};

class V4R_EXPORTS EntangledForestClusterExistsFeature : public EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    // serialize base class information
    ar& boost::serialization::base_object<EntangledForestSplitFeature>(*this);
    ar& mMinAngle;
    ar& mMaxAngle;
    ar& mMinPtPlDist;
    ar& mMaxPtPlDist;
    ar& mMaxEDist;
    ar& mHorizontal;
  }

  double mMinAngle;
  double mMaxAngle;
  double mMinPtPlDist;
  double mMaxPtPlDist;
  double mMaxEDist;
  bool mHorizontal;

  // uniform distribution for dx, dy parameters
  std::uniform_real_distribution<double> mUniformDistPtPl;
  std::uniform_real_distribution<double> mUniformDistAngle;
  std::normal_distribution<double> mNormalDistAngle;
  std::normal_distribution<double> mNormalDistPtPl;

  bool evaluate(EntangledForestData* data, int imageIdx, int clusterIdx, double minangle, double maxangle,
                double minptpl, double maxptpl, double maxeuclid, bool& result);

 public:
  EntangledForestClusterExistsFeature();
  EntangledForestClusterExistsFeature(const EntangledForestClusterExistsFeature& f);
  EntangledForestClusterExistsFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                                      bool horizontal);
  std::string ToString() override;

  EntangledForestSplitFeature* Clone() override;
  EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) override;
  void SetParameters(const std::vector<double>& parameters) override;
  void SampleRandomParameters(std::vector<double>& parameters) override;
  bool evaluateTraining(EntangledForestData* data, int imageIdx, int clusterIdx, const std::vector<double>& parameters,
                        bool& result) override;
  bool evaluateInference(EntangledForestData* data, int imageIdx, int clusterIdx) override;

  inline bool IsHorizontal() {
    return mHorizontal;
  }
  inline std::vector<double> GetGeometryParameters() {
    return {mMinAngle, mMaxAngle, mMinPtPlDist, mMaxPtPlDist, mMaxEDist};
  }
};

class V4R_EXPORTS EntangledForestTopNFeature : public EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    // serialize base class information
    ar& boost::serialization::base_object<EntangledForestSplitFeature>(*this);
    ar& mMinAngle;
    ar& mMaxAngle;
    ar& mMinPtPlDist;
    ar& mMaxPtPlDist;
    ar& mMaxEDist;
    ar& mHorizontal;
    ar& mN;
    ar& mLabel;
  }

  double mMinAngle;
  double mMaxAngle;
  double mMinPtPlDist;
  double mMaxPtPlDist;
  double mMaxEDist;
  bool mHorizontal;
  unsigned int mN;
  unsigned int mLabel;

  // uniform distribution for dx, dy parameters
  std::uniform_real_distribution<double> mUniformDistPtPl;
  std::uniform_real_distribution<double> mUniformDistAngle;
  std::normal_distribution<double> mNormalDistAngle;
  std::normal_distribution<double> mNormalDistPtPl;

  // uniform distribution for label parameters (up to labelnr-1)
  std::uniform_int_distribution<int> mUniformDistLabel;
  // uniform distribution for N parameters
  std::uniform_int_distribution<int> mUniformDistN;

  std::uniform_int_distribution<int> mDontcare;

  bool evaluate(EntangledForestData* data, int imageIdx, int clusterIdx, double minangle, double maxangle,
                double minptpl, double maxptpl, double maxeuclid, unsigned int label, unsigned int N, bool& result);

 public:
  EntangledForestTopNFeature();
  EntangledForestTopNFeature(const EntangledForestTopNFeature& f);
  EntangledForestTopNFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                             bool horizontal, int nLabels);
  std::string ToString() override;
  EntangledForestSplitFeature* Clone() override;
  EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) override;
  void SetParameters(const std::vector<double>& parameters) override;
  void SampleRandomParameters(std::vector<double>& parameters) override;
  bool evaluateTraining(EntangledForestData* data, int imageIdx, int clusterIdx, const std::vector<double>& parameters,
                        bool& result) override;
  bool evaluateInference(EntangledForestData* data, int imageIdx, int clusterIdx) override;

  // to analyze tree
  inline bool IsHorizontal() {
    return mHorizontal;
  }
  inline unsigned int GetLabel() {
    return mLabel;
  }
  inline unsigned int GetN() {
    return mN;
  }
  inline std::vector<double> GetGeometryParameters() {
    return {mMinAngle, mMaxAngle, mMinPtPlDist, mMaxPtPlDist, mMaxEDist};
  }
};

class V4R_EXPORTS EntangledForestInverseTopNFeature : public EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    // serialize base class information
    ar& boost::serialization::base_object<EntangledForestSplitFeature>(*this);
    ar& mMinAngle;
    ar& mMaxAngle;
    ar& mMinIPtPlDist;
    ar& mMaxIPtPlDist;
    ar& mMaxEDist;
    ar& mN;
    ar& mLabel;
  }

  double mMinAngle;
  double mMaxAngle;
  double mMinIPtPlDist;
  double mMaxIPtPlDist;
  double mMaxEDist;
  unsigned int mN;
  unsigned int mLabel;

  // uniform distribution for dx, dy parameters
  std::uniform_real_distribution<double> mUniformDistPtPl;
  std::uniform_real_distribution<double> mUniformDistAngle;
  std::normal_distribution<double> mNormalDistAngle;
  std::normal_distribution<double> mNormalDistPtPl;

  // uniform distribution for label parameters (up to labelnr-1)
  std::uniform_int_distribution<int> mUniformDistLabel;
  // uniform distribution for N parameters
  std::uniform_int_distribution<int> mUniformDistN;

  std::uniform_int_distribution<int> mDontcare;

  bool evaluate(EntangledForestData* data, int imageIdx, int clusterIdx, double minangle, double maxangle,
                double minptpl, double maxiptpl, double maxeuclid, unsigned int label, unsigned int N, bool& result);

 public:
  EntangledForestInverseTopNFeature();
  EntangledForestInverseTopNFeature(const EntangledForestInverseTopNFeature& f);
  EntangledForestInverseTopNFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                                    int nLabels);
  std::string ToString() override;
  EntangledForestSplitFeature* Clone() override;
  EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) override;
  void SetParameters(const std::vector<double>& parameters) override;
  void SampleRandomParameters(std::vector<double>& parameters) override;
  bool evaluateTraining(EntangledForestData* data, int imageIdx, int clusterIdx, const std::vector<double>& parameters,
                        bool& result) override;
  bool evaluateInference(EntangledForestData* data, int imageIdx, int clusterIdx) override;

  // to analyze tree
  inline unsigned int GetN() {
    return mN;
  }
  inline unsigned int GetLabel() {
    return mLabel;
  }
  inline std::vector<double> GetGeometryParameters() {
    return {mMinAngle, mMaxAngle, mMinIPtPlDist, mMaxIPtPlDist, mMaxEDist};
  }
};

class V4R_EXPORTS EntangledForestCommonAncestorFeature : public EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    // serialize base class information
    ar& boost::serialization::base_object<EntangledForestSplitFeature>(*this);
    ar& mMinAngle;
    ar& mMaxAngle;
    ar& mMinPtPlDist;
    ar& mMaxPtPlDist;
    ar& mMaxEDist;
    ar& mHorizontal;
    ar& mMaxSteps;
  }

  double mMinAngle;
  double mMaxAngle;
  double mMinPtPlDist;
  double mMaxPtPlDist;
  double mMaxEDist;
  bool mHorizontal;
  unsigned int mMaxSteps;

  // uniform distribution for dx, dy parameters
  std::uniform_real_distribution<double> mUniformDistPtPl;
  std::uniform_real_distribution<double> mUniformDistAngle;
  std::normal_distribution<double> mNormalDistAngle;
  std::normal_distribution<double> mNormalDistPtPl;

  std::uniform_int_distribution<int> mDontcare;

  bool evaluate(EntangledForestData* data, int imageIdx, int clusterIdx, double minangle, double maxangle,
                double minptpl, double maxptpl, double maxeuclid, unsigned int maxSteps, bool& result);

 public:
  EntangledForestCommonAncestorFeature();
  EntangledForestCommonAncestorFeature(const EntangledForestCommonAncestorFeature& f);
  EntangledForestCommonAncestorFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                                       bool horizontal);
  std::string ToString() override;
  EntangledForestSplitFeature* Clone() override;
  EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) override;
  void SetParameters(const std::vector<double>& parameters) override;
  void SampleRandomParameters(std::vector<double>& parameters) override;
  bool evaluateTraining(EntangledForestData* data, int imageIdx, int clusterIdx, const std::vector<double>& parameters,
                        bool& result) override;
  bool evaluateInference(EntangledForestData* data, int imageIdx, int clusterIdx) override;

  inline bool IsHorizontal() {
    return mHorizontal;
  }
  inline unsigned int GetMaxSteps() {
    return mMaxSteps;
  }
  inline std::vector<double> GetGeometryParameters() {
    return {mMinAngle, mMaxAngle, mMinPtPlDist, mMaxPtPlDist, mMaxEDist};
  }
};

class V4R_EXPORTS EntangledForestNodeDescendantFeature : public EntangledForestSplitFeature {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version __attribute__((unused))) {
    // serialize base class information
    ar& boost::serialization::base_object<EntangledForestSplitFeature>(*this);
    ar& mMinAngle;
    ar& mMaxAngle;
    ar& mMinPtPlDist;
    ar& mMaxPtPlDist;
    ar& mMaxEDist;
    ar& mHorizontal;
    ar& mAncestorNode;
  }

  double mMinAngle;
  double mMaxAngle;
  double mMinPtPlDist;
  double mMaxPtPlDist;
  double mMaxEDist;
  bool mHorizontal;
  unsigned int mAncestorNode;

  // uniform distribution for dx, dy parameters
  std::uniform_real_distribution<double> mUniformDistPtPl;
  std::uniform_real_distribution<double> mUniformDistAngle;
  std::normal_distribution<double> mNormalDistAngle;
  std::normal_distribution<double> mNormalDistPtPl;

  std::uniform_int_distribution<int> mDontcare;

  bool evaluate(EntangledForestData* data, int imageIdx, int clusterIdx, double minangle, double maxangle,
                double minptpl, double maxptpl, double maxeuclid, unsigned int ancestorNode, bool& result);

 public:
  EntangledForestNodeDescendantFeature();
  EntangledForestNodeDescendantFeature(const EntangledForestNodeDescendantFeature& f);
  EntangledForestNodeDescendantFeature(std::string name, EntangledForestTree* tree, std::mt19937* randomGenerator,
                                       bool horizontal);
  std::string ToString() override;
  EntangledForestSplitFeature* Clone() override;
  EntangledForestSplitFeature* Clone(EntangledForestTree* newTree) override;
  void SetParameters(const std::vector<double>& parameters) override;
  void SampleRandomParameters(std::vector<double>& parameters) override;
  bool evaluateTraining(EntangledForestData* data, int imageIdx, int clusterIdx, const std::vector<double>& parameters,
                        bool& result) override;
  bool evaluateInference(EntangledForestData* data, int imageIdx, int clusterIdx) override;

  inline bool IsHorizontal() {
    return mHorizontal;
  }
  inline unsigned int GetAncestorNodeID() {
    return mAncestorNode;
  }
  inline std::vector<double> GetGeometryParameters() {
    return {mMinAngle, mMaxAngle, mMinPtPlDist, mMaxPtPlDist, mMaxEDist};
  }
};
}
