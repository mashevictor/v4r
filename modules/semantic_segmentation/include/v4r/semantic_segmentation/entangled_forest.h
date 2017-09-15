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
 * @file   entangled_forest.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */


#pragma once

#include <algorithm>
#include <time.h>
#include <random>
#include <fstream>
#include <string.h>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

#include <v4r/core/macros.h>
#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_tree.h>

#ifdef _OPENMP
#include <omp.h>
#else
#define omp_get_thread_num() 0
#endif

namespace v4r {

// how many data points to load into RAM at once
#define MAX_DATAPOINTS_TO_LOAD 1000000

class EntangledForestTree;

class V4R_EXPORTS EntangledForest
{   
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mTrees;
        ar & mLabelMap;
        ar & mMaxDepth;
        ar & mNTrees;
        ar & mSampledSplitFunctionParameters;
        ar & mSampledSplitFunctionThresholds;
        ar & mMinInformationGain;
        ar & mMinPointsForSplit;
        ar & mBaggingRatio;

        if(version >= 2)
        {
            ar & mLabelNames;
        }
    }

    std::mt19937 mRandomGenerator;
    std::vector<EntangledForestTree*> mTrees;
    int mMaxDepth;
    int mNTrees;
    int mSampledSplitFunctionParameters;   // new
    int mSampledSplitFunctionThresholds;   // new
    float mMinInformationGain;
    int mMinPointsForSplit;
    float mBaggingRatio;

    // for evaluation, split nodes can also store label distributions, to be able to traverse
    // trees only down to a certain depth for classification
    bool mSplitNodesStoreLabelDistribution;

    int mNLabels;
    std::map<int, int> mLabelMap;
    std::vector<std::string> mLabelNames;

public:
    EntangledForest();
    // new
    EntangledForest(int nTrees, int maxDepth = 8, float baggingRatio = 0.5, int sampledSplitFunctionParameters = 100, int sampledSplitFunctionThresholds = 50, float minInformationGain = 0.02, int minPointsForSplit = 5);
    void Train(EntangledForestData *trainingData, bool tryUniformBags, int verbosityLevel = 1);

    void UpdateRandomGenerator();   // neccessary after load from file

    void Classify(EntangledForestData* data, std::vector<int> &result, int maxDepth = -1, int useNTrees = -1); //, bool reweightLeafDistributions = false);
    void GetHardClassificationResult(std::vector<std::vector<double> > &softResult, std::vector<int> &result);
    void SoftClassify(EntangledForestData *data, std::vector<std::vector<double> > &result, int maxDepth = -1, int useNTrees = -1); //, bool reweightLeafDistributions = false);   // new version)

    void SaveToFile(std::string filename);
    void SaveToBinaryFile(std::string filename);
    static void LoadFromFile(std::string filename, EntangledForest &f);
    static void LoadFromBinaryFile(std::string filename, EntangledForest &f);
    std::vector<int> GetLabels();
    std::map<int, int> GetLabelMap();
    int GetNrOfLabels();
    std::vector<std::string>& GetLabelNames();
    bool SetLabelNames(std::vector<std::string>& names);

    EntangledForestTree* GetTree(int idx);
    int GetNrOfTrees();
    void saveMatlab(std::string filename);
    void Merge(EntangledForest &f);

    void updateLeafs(EntangledForestData *trainingData, int updateDepth, double updateWeight, bool tryUniformBags);

    void correctTreeIndices();
    virtual ~EntangledForest();
};

}

BOOST_CLASS_VERSION(v4r::EntangledForest, 2)
